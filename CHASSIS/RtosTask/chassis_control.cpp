#include "chassis_control.hpp"
#include "chassis_kinematics_config.hpp"
#include "user/core/Alg/Omni/chassis_ff.hpp"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cmath>

/**
 * @brief 构造底盘运动学求解器和逆动力学模型。
 */
ChassisCtrl::ChassisCtrl()
    : kinematics_(App::Chassis::make_kinematics_config()), dynamics_(make_dynamics())
{
}

/**
 * @brief 绑定四轮反馈与指令，配置功率限制、轮速 PID 和前馈速度规划。
 * @return 初始化成功返回 true，已启用的功率限制配置无效时返回 false。
 */
bool ChassisCtrl::Init()
{
    if (chassis_cfg::chassis.power.enabled && !power_control_config_valid())
    {
        return false;
    }
    ALG::Omni::PowerControlStrategy::RefereeConfig power_config{};
    power_config.fallback_w = chassis_cfg::chassis.power.fallback_w;
    power_config.margin_w = chassis_cfg::chassis.power.margin_w;
    power_config.timeout_ms = chassis_cfg::chassis.power.referee_timeout_ms;
    power_config.buffer_low_j = chassis_cfg::chassis.power.buffer_low_j;
    power_config.buffer_recover_j = chassis_cfg::chassis.power.buffer_recover_j;
    power_config.low_buffer_scale = chassis_cfg::chassis.power.low_buffer_scale;
    if (!power_strategy_.ConfigureReferee(power_config) && chassis_cfg::chassis.power.enabled)
    {
        return false;
    }
    power_control_.configure({chassis_cfg::chassis.power.model.absolute_speed_loss, false,
                              chassis_cfg::chassis.wheel.motor.current_max_a /
                                  chassis_cfg::chassis.wheel.motor.command_max_raw});

    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        wheel_[i].fb = &sentry_data.wheel_fb[i];
        wheel_[i].cmd = &sentry_data.wheel_cmd[i];
        wheel_[i].pid.set_config(WHEEL_PID_DEFAULT);
    }

    planner_vx_.configure(make_linear_planner_config());
    planner_vy_.configure(make_linear_planner_config());
    planner_wz_.configure(make_angular_planner_config());

    reset_control();
    mode_fsm_.init(OnmiChassisMode::STOP, Sentry::Time::now_ms());

    return true;
}

/**
 * @brief 读取当前周期输入，更新底盘模式并在满足输出条件时执行四轮闭环控制。
 */
void ChassisCtrl::Update()
{
    const auto in = readInputs();

    mode_fsm_.update(select_mode(in.remote), in.nowMs);
    if (mode_fsm_.changed())
    {
        reset_control();
    }

    const auto mode = mode_fsm_.state();
    if (mode == OnmiChassisMode::STOP || mode == OnmiChassisMode::COUNT)
    {
        stop_all();
        return;
    }

    if (!power_output_allowed_ || !system_safe_for_output(in.remote, mode, in.command))
    {
        reset_control();
        stop_all();
        return;
    }

    run_motion_command(mode, in.command, in.feedback);
}

/**
 * @brief 将底盘切换到停止模式，复位控制状态并发布四轮零电流指令。
 */
void ChassisCtrl::Stop()
{
    mode_fsm_.force(OnmiChassisMode::STOP, Sentry::Time::now_ms());
    reset_control();
    stop_all();
}

/**
 * @brief 读取遥控器、运动指令和四轮反馈，同时刷新功率预算及轮子离线状态。
 * @return 本轮控制使用的时间戳、遥控数据、运动指令和轮速反馈。
 */
ChassisCtrl::Inputs ChassisCtrl::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.command = sentry_data.motion_command.read();

    update_power_budget(in.nowMs);

    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        in.feedback[i] = wheel_[i].fb->read();
        wheel_[i].fb_online = in.feedback[i].timestamp != 0U &&
                              in.feedback[i].is_online(chassis_cfg::chassis.timeouts.motor_fb_soft_ms);

        const bool was_offline = wheel_[i].offline;
        wheel_[i].offline = feedback_hard_offline(in.feedback[i], in.nowMs);
        if (wheel_[i].offline != was_offline)
        {
            reset_wheel_control(i);
        }
    }

    return in;
}

/**
 * @brief 处理机体速度目标，依次计算轮速目标、力矩前馈和四轮电流。
 * @param mode 当前底盘控制模式。
 * @param command 本周期的线速度及角速度指令。
 * @param feedback 四轮电机反馈，按左前、右前、左后、右后排列。
 */
void ChassisCtrl::run_motion_command(OnmiChassisMode mode,
                                    const App::Chassis::ChassisMotionCommand &command,
                                    const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT])
{
    float vx = Sentry::Utils::clamp(command.vx_mps, -chassis_cfg::chassis.limits.max_planar_speed_mps,
                                    chassis_cfg::chassis.limits.max_planar_speed_mps);
    float vy = Sentry::Utils::clamp(command.vy_mps, -chassis_cfg::chassis.limits.max_planar_speed_mps,
                                    chassis_cfg::chassis.limits.max_planar_speed_mps);
    float wz = Sentry::Utils::clamp(command.wz_radps, -chassis_cfg::chassis.limits.max_angular_speed_radps,
                                    chassis_cfg::chassis.limits.max_angular_speed_radps);
    Sentry::Utils::normalize_planar_speed(vx, vy, chassis_cfg::chassis.limits.max_planar_speed_mps);

    if ((mode == OnmiChassisMode::FOLLOW || mode == OnmiChassisMode::NO_FOLLOW) &&
        command.source == App::Chassis::ChassisMotionSource::Manual &&
        chassis_cfg::chassis.limits.max_planar_speed_mps > 0.0f)
    {
        apply_manual_translation(vx, vy);
    }

    float wheel_radps[WHEEL_COUNT]{};
    calculate_wheel_targets(vx, vy, wz, wheel_radps);
    const auto ff = calculate_feedforward(vx, vy, wz, wheel_radps);
    run_wheel_speed_control(wheel_radps, ff, feedback);
}

/**
 * @brief 根据云台偏航角和里程计速度对手动平移指令做坐标变换与动态补偿。
 * @param[in,out] vx X 方向平移目标，单位为 m/s。
 * @param[in,out] vy Y 方向平移目标，单位为 m/s。
 */
void ChassisCtrl::apply_manual_translation(float &vx, float &vy)
{
    const auto yaw = sentry_data.yaw_joint.read();
    const auto odometry = sentry_data.odometry.read();
    float actual_vx = 0.0f;
    float actual_vy = 0.0f;
    float actual_wz = 0.0f;
    if (odometry.is_online(chassis_cfg::chassis.manual.odometry_timeout_ms) &&
        std::isfinite(odometry.vx_body_mps) && std::isfinite(odometry.vy_body_mps) &&
        std::isfinite(odometry.wz_radps))
    {
        actual_vx = odometry.vx_body_mps;
        actual_vy = odometry.vy_body_mps;
        actual_wz = odometry.wz_radps;
    }

    const bool yaw_valid = yaw.is_online(chassis_cfg::chassis.manual.gimbal_yaw_timeout_ms) &&
                           std::isfinite(yaw.yaw_angle_rad);
    const auto &manual = chassis_cfg::chassis.manual;
    const float yaw_angle = yaw_valid ? yaw.yaw_angle_rad : manual.gimbal_yaw_zero_rad;
    const float sign = manual.gimbal_yaw_sign;
    const float max_speed = chassis_cfg::chassis.limits.max_planar_speed_mps;
    const float joystick_x = Sentry::Utils::clamp(-vy / max_speed, -1.0f, 1.0f);
    const float joystick_y = Sentry::Utils::clamp(vx / max_speed, -1.0f, 1.0f);
    Sentry::Utils::calculate_translation_xy(
        sign * yaw_angle, joystick_x, joystick_y, sign * manual.gimbal_yaw_zero_rad, max_speed,
        manual.gimbal_angle_comp_rad, manual.gimbal_phase_comp_rad, actual_vx, actual_vy, actual_wz,
        &vx, &vy, manual.dynamic_k_base);
    Sentry::Utils::normalize_planar_speed(vx, vy, max_speed);
}

/**
 * @brief 逆解四轮转速，将硬离线轮的目标置零，对其余车轮统一限速并同步缩放机体速度。
 * @param[in,out] vx 底盘 X 方向速度，单位为 m/s。
 * @param[in,out] vy 底盘 Y 方向速度，单位为 m/s。
 * @param[in,out] wz 底盘绕 Z 轴角速度，单位为 rad/s。
 * @param[out] wheel_radps 四轮转速目标，单位为 rad/s。
 */
void ChassisCtrl::calculate_wheel_targets(float &vx, float &vy, float &wz,
                                         float (&wheel_radps)[WHEEL_COUNT])
{
    ALG::Omni::WheelOutput output[WHEEL_COUNT]{};
    kinematics_.inverse_kinematics(vx, vy, wz, output);

    float max_abs_ref = 0.0f;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        const float speed_radps =
            output[i].speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
        wheel_radps[i] = wheel_[i].offline ? 0.0f : speed_radps;
        max_abs_ref = std::fmax(max_abs_ref, std::fabs(wheel_radps[i]));
    }
    const float max_wheel_speed_radps =
        chassis_cfg::chassis.limits.max_wheel_speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
    if (max_wheel_speed_radps > 0.0f && max_abs_ref > max_wheel_speed_radps)
    {
        const float scale = max_wheel_speed_radps / max_abs_ref;
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            wheel_radps[i] *= scale;
        }
        vx *= scale;
        vy *= scale;
        wz *= scale;
    }
}

/**
 * @brief 计算四轮惯性、转向平移耦合及摩擦补偿力矩。
 * @param vx 限速后的底盘 X 方向速度，单位为 m/s。
 * @param vy 限速后的底盘 Y 方向速度，单位为 m/s。
 * @param wz 限速后的底盘绕 Z 轴角速度，单位为 rad/s。
 * @param wheel_radps 限速后的四轮转速目标，单位为 rad/s。
 * @return 前馈启用状态及各轮惯性、速度相关的补偿力矩，单位为 N·m。
 */
ChassisCtrl::FeedforwardTorques ChassisCtrl::calculate_feedforward(
    float vx, float vy, float wz, const float (&wheel_radps)[WHEEL_COUNT])
{
    FeedforwardTorques ff{};
    ff.enabled = force_ff_config_valid() && dynamics_.IsAllocationValid();
    if (ff.enabled)
    {
        const float ff_vx = planner_vx_.update(vx, DT_SEC);
        const float ff_vy = planner_vy_.update(vy, DT_SEC);
        const float ff_wz = planner_wz_.update(wz, DT_SEC);
        const float ref_ax = planner_vx_.GetAcceleration();
        const float ref_ay = planner_vy_.GetAcceleration();
        const float ref_awz = planner_wz_.GetAcceleration();

        dynamics_.OmniInvDynamics(chassis_cfg::chassis.force_ff.mass_kg * ref_ax,
                                  chassis_cfg::chassis.force_ff.mass_kg * ref_ay,
                                  chassis_cfg::chassis.force_ff.yaw_inertia_kgm2 * ref_awz);
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            ff.inertial_nm[i] = dynamics_.GetMotorTorque(i);
        }

        const bool turning_translation_requested =
            (vx * vx + vy * vy) > 1.0e-12f && std::fabs(wz) > 1.0e-6f;
        dynamics_.OmniInvDynamics(
            turning_translation_requested ? -chassis_cfg::chassis.force_ff.mass_kg * ff_wz * ff_vy : 0.0f,
            turning_translation_requested ? chassis_cfg::chassis.force_ff.mass_kg * ff_wz * ff_vx : 0.0f,
            0.0f);

        ALG::Omni::WheelOutput angular_accel[WHEEL_COUNT]{};
        kinematics_.inverse_kinematics(ref_ax, ref_ay, ref_awz, angular_accel);
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            const float wheel_alpha =
                angular_accel[i].speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
            ff.inertial_nm[i] += chassis_cfg::chassis.force_ff.wheel_equivalent_inertia_kgm2 * wheel_alpha;
            const float wheel_speed = wheel_radps[i];
            float friction = chassis_cfg::chassis.force_ff.wheel_viscous_friction_nm_per_radps * wheel_speed;
            if constexpr (chassis_cfg::chassis.force_ff.wheel_coulomb_friction_nm != 0.0f)
            {
                friction += chassis_cfg::chassis.force_ff.wheel_coulomb_friction_nm *
                            std::tanh(wheel_speed / chassis_cfg::chassis.force_ff.wheel_friction_smooth_radps);
            }
            ff.speed_nm[i] = dynamics_.GetMotorTorque(i) + friction;
        }
    }
    else
    {
        planner_vx_.reset(vx);
        planner_vy_.reset(vy);
        planner_wz_.reset(wz);
    }
    return ff;
}

/**
 * @brief 计算各轮 PID 和前馈电流，执行功率限制后发布电机指令。
 * @param wheel_radps 四轮转速目标，单位为 rad/s。
 * @param ff 四轮力矩前馈计算结果。
 * @param feedback 四轮电机反馈，用于轮速闭环和功率估算。
 */
void ChassisCtrl::run_wheel_speed_control(const float (&wheel_radps)[WHEEL_COUNT],
                                         const FeedforwardTorques &ff,
                                         const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT])
{
    float requested_raw[WHEEL_COUNT]{};
    float requested_a[WHEEL_COUNT]{};
    float rotor_radps[WHEEL_COUNT]{};
    bool active[WHEEL_COUNT]{};
    const float amp_per_raw =
        chassis_cfg::chassis.wheel.motor.current_max_a / chassis_cfg::chassis.wheel.motor.command_max_raw;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        if (wheel_[i].offline || !std::isfinite(feedback[i].output_speed_radps) ||
            !std::isfinite(feedback[i].rotor_speed_radps))
        {
            reset_wheel_control(i);
            continue;
        }

        const float ref_radps = wheel_radps[i];
        const float sign = App::Chassis::WHEEL_SIGN[i];
        const float fb_radps = feedback[i].output_speed_radps * sign;
        float ff_raw = 0.0f;
        if (ff.enabled && wheel_[i].fb_online)
        {
            const float inertial = ALG::Omni::guard_chassis_inertial_ff(
                ff.inertial_nm[i], ref_radps - fb_radps,
                chassis_cfg::chassis.force_ff.inertial_fade_error_radps);

            ff_raw = torque_to_raw_unclamped(inertial + ff.speed_nm[i]) * sign;
        }
        const float pid_raw = wheel_[i].pid.update(ref_radps, fb_radps, DT_SEC);

        const float total_raw = ff_raw + pid_raw * sign;
        if (!std::isfinite(total_raw))
        {
            reset_wheel_control(i);
            continue;
        }
        requested_raw[i] = Sentry::Utils::clamp(total_raw, chassis_cfg::chassis.wheel.speed_pid.output_min,
                                                chassis_cfg::chassis.wheel.speed_pid.output_max);

        requested_a[i] = std::trunc(requested_raw[i]) * amp_per_raw;
        rotor_radps[i] = feedback[i].rotor_speed_radps;
        active[i] = true;
    }

    float fixed_power_w = chassis_cfg::chassis.power.fixed_w;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        if (!active[i])
        {
            const float idle = std::fmax(0.0f, chassis_cfg::chassis.power.model.coeffs[0]);
            fixed_power_w += idle;
        }
    }
    ALG::Omni::PowerControl<WHEEL_COUNT>::Result power_result{};
    if (chassis_cfg::chassis.power.enabled)
    {
        power_result = power_control_.DecayingCurrent(
            requested_a, rotor_radps, chassis_cfg::chassis.power.model.coeffs, nullptr, fixed_power_w,
            power_strategy_.GetInputLimit(), active);
    }
    const bool power_fault = !power_result.valid || !power_result.feasible;

    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        active[i] = active[i] && !power_fault;
        std::int16_t current = 0;
        if (active[i])
        {
            current =
                chassis_cfg::chassis.power.enabled
                    ? clamp_current(std::round(power_control_.getCurrent(i) / amp_per_raw))
                    : clamp_current(requested_raw[i]);
        }
        else
        {
            reset_wheel_control(i);
        }
        wheel_[i].cmd->publish(Sentry::MotorCommand{current, active[i]});
    }
}

/**
 * @brief 根据底盘半径、轮半径及各轮安装角创建逆动力学模型。
 * @return 用于分配各轮前馈力矩的逆动力学模型。
 */
ALG::Omni::Omni_ID ChassisCtrl::make_dynamics()
{
    const float wheel_azimuth[WHEEL_COUNT] = {
        chassis_cfg::chassis.geometry.azimuth.fl_rad,
        chassis_cfg::chassis.geometry.azimuth.fr_rad,
        chassis_cfg::chassis.geometry.azimuth.bl_rad,
        chassis_cfg::chassis.geometry.azimuth.br_rad,
    };
    const float wheel_direction[WHEEL_COUNT] = {
        chassis_cfg::chassis.geometry.direction.fl_rad,
        chassis_cfg::chassis.geometry.direction.fr_rad,
        chassis_cfg::chassis.geometry.direction.bl_rad,
        chassis_cfg::chassis.geometry.direction.br_rad,
    };

    return ALG::Omni::Omni_ID(chassis_cfg::chassis.geometry.chassis_radius_m,
                              chassis_cfg::chassis.geometry.wheel_radius_m,
                              static_cast<float>(WHEEL_COUNT), wheel_azimuth, wheel_direction);
}

/**
 * @brief 生成 X、Y 方向共用的平移速度规划参数。
 * @return 平移速度、加速度、减速度及加加速度限制。
 */
ALG::Omni::SCurveVelocityPlanning::Config ChassisCtrl::make_linear_planner_config()
{
    ALG::Omni::SCurveVelocityPlanning::Config config{};
    config.max_speed = chassis_cfg::chassis.limits.max_planar_speed_mps;
    config.max_accel = chassis_cfg::chassis.force_ff.linear.max_accel_mps2;
    config.max_decel = chassis_cfg::chassis.force_ff.linear.max_decel_mps2;
    config.max_jerk = chassis_cfg::chassis.force_ff.linear.max_jerk_mps3;
    return config;
}

/**
 * @brief 生成绕 Z 轴转动的速度规划参数。
 * @return 角速度、角加速度、角减速度及角加加速度限制。
 */
ALG::Omni::SCurveVelocityPlanning::Config ChassisCtrl::make_angular_planner_config()
{
    ALG::Omni::SCurveVelocityPlanning::Config config{};
    config.max_speed = chassis_cfg::chassis.limits.max_angular_speed_radps;
    config.max_accel = chassis_cfg::chassis.force_ff.angular.max_accel_radps2;
    config.max_decel = chassis_cfg::chassis.force_ff.angular.max_decel_radps2;
    config.max_jerk = chassis_cfg::chassis.force_ff.angular.max_jerk_radps3;
    return config;
}

/**
 * @brief 检查力矩前馈是否启用及其物理参数是否有效。
 * @return 前馈已启用且参数有效时返回 true。
 */
bool ChassisCtrl::force_ff_config_valid()
{
    if (!chassis_cfg::chassis.force_ff.enabled)
    {
        return false;
    }

    const float values[] = {
        chassis_cfg::chassis.force_ff.mass_kg,
        chassis_cfg::chassis.force_ff.yaw_inertia_kgm2,
        chassis_cfg::chassis.force_ff.wheel_equivalent_inertia_kgm2,
        chassis_cfg::chassis.force_ff.wheel_coulomb_friction_nm,
        chassis_cfg::chassis.force_ff.wheel_viscous_friction_nm_per_radps,
        chassis_cfg::chassis.force_ff.wheel_friction_smooth_radps,
        chassis_cfg::chassis.force_ff.inertial_fade_error_radps,
    };
    for (const float value : values)
    {
        if (!std::isfinite(value) || value < 0.0f)
        {
            return false;
        }
    }
    return chassis_cfg::chassis.force_ff.wheel_friction_smooth_radps > 0.0f &&
           chassis_cfg::chassis.force_ff.inertial_fade_error_radps > 0.0f &&
           (chassis_cfg::chassis.force_ff.mass_kg > 0.0f || chassis_cfg::chassis.force_ff.yaw_inertia_kgm2 > 0.0f ||
            chassis_cfg::chassis.force_ff.wheel_equivalent_inertia_kgm2 > 0.0f ||
            chassis_cfg::chassis.force_ff.wheel_coulomb_friction_nm > 0.0f ||
            chassis_cfg::chassis.force_ff.wheel_viscous_friction_nm_per_radps > 0.0f);
}

/**
 * @brief 校验功率模型、固定功耗及电流换算参数。
 * @return 功率限制所需参数有效时返回 true。
 */
bool ChassisCtrl::power_control_config_valid()
{
    for (const float coefficient : chassis_cfg::chassis.power.model.coeffs)
    {
        if (!std::isfinite(coefficient))
            return false;
    }
    return chassis_cfg::chassis.power.model.coeffs[4] >= 0.0f &&
           std::isfinite(chassis_cfg::chassis.power.fixed_w) && chassis_cfg::chassis.power.fixed_w >= 0.0f &&
           std::isfinite(chassis_cfg::chassis.wheel.motor.current_max_a) &&
           chassis_cfg::chassis.wheel.motor.current_max_a > 0.0f &&
           std::isfinite(chassis_cfg::chassis.wheel.motor.command_max_raw) &&
           chassis_cfg::chassis.wheel.motor.command_max_raw > 0.0f;
}

/**
 * @brief 更新裁判系统功率预算与底盘输出许可。
 * @param now 当前控制周期的毫秒时间戳。
 */
void ChassisCtrl::update_power_budget(std::uint32_t now)
{
    const Sentry::RefereeData referee = sentry_data.referee.read();
    power_strategy_.UpdateReferee(now, referee.online(chassis_cfg::chassis.power.referee_timeout_ms),
                                  referee.robot_status_count, referee.power_heat_count,
                                  static_cast<float>(referee.status.chassis_power_limit),
                                  static_cast<float>(referee.power.buffer_energy),
                                  referee.status.power_management_chassis_output != 0U);
    power_output_allowed_ =
        !chassis_cfg::chassis.power.enabled || power_strategy_.GetInputLimit() > 0.0f;
}

/**
 * @brief 将轮端目标力矩换算为未经限幅的电流指令。
 * @param torque_nm 目标力矩，单位为 N·m。
 * @return 电流指令原始值，力矩或换算参数无效时返回 0。
 */
float ChassisCtrl::torque_to_raw_unclamped(float torque_nm)
{
    const float k_tau = chassis_cfg::chassis.wheel.motor.torque_constant_nm_per_a;
    const float current_max = chassis_cfg::chassis.wheel.motor.current_max_a;
    const float command_max = chassis_cfg::chassis.wheel.motor.command_max_raw;
    if (!std::isfinite(torque_nm) || !std::isfinite(k_tau) || k_tau <= 0.0f ||
        !std::isfinite(current_max) || current_max <= 0.0f || !std::isfinite(command_max) ||
        command_max <= 0.0f)
    {
        return 0.0f;
    }

    return torque_nm / k_tau * command_max / current_max;
}

/**
 * @brief 检查遥控反馈有效期和禁用拨杆组合。
 * @param remote 遥控器数据快照。
 * @return 遥控器在线且允许底盘工作时返回 true。
 */
bool ChassisCtrl::remote_allows_enable(const Sentry::RemoteData &remote)
{
    const auto &switches = chassis_cfg::chassis.input.switches;
    return Sentry::Time::fresh(remote.timestamp, chassis_cfg::chassis.timeouts.remote_connection_ms) &&
           !(remote.s1 == switches.disable_s1 && remote.s2 == switches.disable_s2);
}

/**
 * @brief 根据遥控器在线状态和拨杆档位决定底盘控制模式。
 * @param remote 遥控器数据快照。
 * @return 停止、跟随、不跟随或自动模式。
 */
OnmiChassisMode ChassisCtrl::select_mode(const Sentry::RemoteData &remote)
{
    if (!remote_allows_enable(remote))
    {
        return OnmiChassisMode::STOP;
    }

    const std::uint8_t s1 = remote.s1;
    const std::uint8_t s2 = remote.s2;

    if (s1 == chassis_cfg::chassis.input.switches.auto_s1 && s2 == chassis_cfg::chassis.input.switches.auto_s2)
    {
        return OnmiChassisMode::AUTO;
    }
    if (s1 == chassis_cfg::chassis.input.switches.follow_s1 && s2 == chassis_cfg::chassis.input.switches.follow_s2)
    {
        return OnmiChassisMode::FOLLOW;
    }
    if (s1 >= 1U && s1 <= 3U && s2 >= 1U && s2 <= 3U)
    {
        return OnmiChassisMode::NO_FOLLOW;
    }
    return OnmiChassisMode::STOP;
}

/**
 * @brief 检查运动指令的来源是否与当前底盘模式匹配。
 * @param mode 当前底盘控制模式。
 * @param command 待执行的底盘运动指令。
 * @return 指令来源与模式匹配时返回 true。
 */
bool ChassisCtrl::command_matches_mode(OnmiChassisMode mode,
                                       const App::Chassis::ChassisMotionCommand &command)
{
    switch (mode)
    {
    case OnmiChassisMode::FOLLOW:
    case OnmiChassisMode::NO_FOLLOW:
        return command.source == App::Chassis::ChassisMotionSource::Manual;
    case OnmiChassisMode::AUTO:
        return command.source == App::Chassis::ChassisMotionSource::Navigation;
    case OnmiChassisMode::STOP:
    case OnmiChassisMode::COUNT:
        return false;
    }
    return false;
}

/**
 * @brief 检查遥控许可、指令有效期、来源及速度值是否满足控制输出条件。
 * @param remote 遥控器数据快照。
 * @param mode 当前底盘控制模式。
 * @param command 待执行的底盘运动指令。
 * @return 满足全部输出条件时返回 true。
 */
bool ChassisCtrl::system_safe_for_output(const Sentry::RemoteData &remote, OnmiChassisMode mode,
                                         const App::Chassis::ChassisMotionCommand &command)
{
    return remote_allows_enable(remote) &&
           command.is_online(chassis_cfg::chassis.timeouts.motion_command_ms) &&
           command.has_known_source() && std::isfinite(command.vx_mps) &&
           std::isfinite(command.vy_mps) && std::isfinite(command.wz_radps) &&
           command_matches_mode(mode, command);
}

/**
 * @brief 判断电机反馈是否达到硬离线条件。
 * @param feedback 当前轮电机反馈。
 * @param now 当前控制周期的毫秒时间戳。
 * @return 从未收到反馈，或反馈未通过软在线检查且达到硬超时时，返回 true。
 */
bool ChassisCtrl::feedback_hard_offline(const Sentry::MotorFeedback &feedback, std::uint32_t now)
{
    if (feedback.is_online(chassis_cfg::chassis.timeouts.motor_fb_soft_ms))
    {
        return false;
    }
    return feedback.timestamp == 0U ||
           now - feedback.timestamp >= chassis_cfg::chassis.timeouts.motor_fb_hard_ms;
}

/**
 * @brief 限制电流指令范围并转换为电机接口接受的整数。
 * @param value 需要限幅的电流指令原始值。
 * @return 限幅后的电流指令，输入为非有限值时返回 0。
 */
std::int16_t ChassisCtrl::clamp_current(float value)
{
    if (!std::isfinite(value))
    {
        return 0;
    }
    return static_cast<std::int16_t>(Sentry::Utils::clamp(value, chassis_cfg::chassis.wheel.speed_pid.output_min,
                                                          chassis_cfg::chassis.wheel.speed_pid.output_max));
}

/**
 * @brief 复位指定车轮的轮速 PID。
 * @param index 车轮索引，顺序为左前、右前、左后、右后。
 */
void ChassisCtrl::reset_wheel_control(std::uint8_t index)
{
    if (index >= WHEEL_COUNT)
    {
        return;
    }
    wheel_[index].pid.reset();
}

/**
 * @brief 复位功率控制、三个速度规划器和四轮轮速 PID。
 */
void ChassisCtrl::reset_control()
{
    power_control_.reset();
    planner_vx_.reset();
    planner_vy_.reset();
    planner_wz_.reset();
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        reset_wheel_control(i);
    }
}

/**
 * @brief 复位四轮轮速 PID，并向已绑定的电机指令通道发布零电流和禁用状态。
 */
void ChassisCtrl::stop_all()
{
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        reset_wheel_control(i);
        if (wheel_[i].cmd != nullptr)
        {
            wheel_[i].cmd->publish(Sentry::MotorCommand{0, false});
        }
    }
}
