#include "communication/robot_data.hpp"
#include "RtosTask/chassis_control.hpp"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cstddef>
#include <cmath>
#include "RtosTask/robot_config.hpp"

namespace
{

constexpr std::size_t DT7_CH_S1 = 4U;
constexpr std::size_t DT7_CH_S2 = 7U;

std::uint8_t dt7_switch(const Sentry::RemoteData &remote, std::size_t index)
{
    if (remote.timestamp == 0U)
        return 0U;
    if (index == DT7_CH_S1)
        return remote.s1;
    if (index == DT7_CH_S2)
        return remote.s2;
    return 0U;
}

bool dt7_connected(const Sentry::RemoteData &remote, std::uint32_t timeout_ms)
{
    return remote.timestamp != 0U && Sentry::Time::fresh(remote.timestamp, timeout_ms);
}

} // namespace

void ChassisCtrl::Update()
{
    // 读取输入快照，检查底盘模式和轮子在线状态后更新四轮输出。

    const auto in = readInputs();

    mode_fsm_.update(select_mode(in.remote), in.nowMs);
    if (mode_fsm_.changed())
    {

        reset_control();
    }

    if (mode_fsm_.state() == OnmiChassisMode::STOP || mode_fsm_.state() == OnmiChassisMode::COUNT)
    {
        if (last_output_allowed_)
        {
        }
        last_output_allowed_ = false;
        stop_all();
        return;
    }

    if (!power_output_allowed_ || !system_safe_for_output(in.remote, mode_fsm_.state(), in.command))
    {
        reset_control();

        last_output_allowed_ = false;
        stop_all();
        return;
    }

    run_motion_command(mode_fsm_.state(), in.command, in.feedback);
}

void ChassisCtrl::run_motion_command(OnmiChassisMode mode,
                                     const App::Chassis::ChassisMotionCommand &command,
                                     const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT])
{
    // 将机体速度目标转换为轮速，再执行速度环、功率限制和电流发布。

    float vx = Sentry::Utils::clamp(command.vx_mps, -chassis_cfg::max_planar_speed,
                                    chassis_cfg::max_planar_speed);
    float vy = Sentry::Utils::clamp(command.vy_mps, -chassis_cfg::max_planar_speed,
                                    chassis_cfg::max_planar_speed);
    float wz = Sentry::Utils::clamp(command.wz_radps, -chassis_cfg::max_vw, chassis_cfg::max_vw);
    Sentry::Utils::normalize_planar_speed(vx, vy, chassis_cfg::max_planar_speed);

    if ((mode == OnmiChassisMode::FOLLOW || mode == OnmiChassisMode::NO_FOLLOW) &&
        command.source == App::Chassis::ChassisMotionSource::Manual &&
        chassis_cfg::max_planar_speed > 0.0f)
    {
        const App::Chassis::GimbalYawJointFeedback yaw =
            true ? sentry_data.yaw_joint.read() : App::Chassis::GimbalYawJointFeedback{};
        float actual_vx = 0.0f;
        float actual_vy = 0.0f;
        float actual_wz = 0.0f;
        {
            const App::Chassis::ChassisOdometry odometry = sentry_data.odometry.read();
            if (odometry.is_online(chassis_cfg::manual_odometry_timeout_ms) &&
                std::isfinite(odometry.vx_body_mps) && std::isfinite(odometry.vy_body_mps) &&
                std::isfinite(odometry.wz_radps))
            {
                actual_vx = odometry.vx_body_mps;
                actual_vy = odometry.vy_body_mps;
                actual_wz = odometry.wz_radps;
            }
        }

        const bool yaw_valid = yaw.is_online(chassis_cfg::manual_gimbal_yaw_timeout_ms) &&
                               std::isfinite(yaw.yaw_angle_rad);
        const float yaw_angle =
            yaw_valid ? yaw.yaw_angle_rad : chassis_cfg::manual_gimbal_yaw_zero_rad;
        const float sign = chassis_cfg::manual_gimbal_yaw_sign;
        const float joystick_x =
            Sentry::Utils::clamp(-vy / chassis_cfg::max_planar_speed, -1.0f, 1.0f);
        const float joystick_y =
            Sentry::Utils::clamp(vx / chassis_cfg::max_planar_speed, -1.0f, 1.0f);
        Sentry::Utils::calculate_translation_xy(
            sign * yaw_angle, joystick_x, joystick_y,
            sign * chassis_cfg::manual_gimbal_yaw_zero_rad, chassis_cfg::max_planar_speed,
            chassis_cfg::manual_gimbal_angle_comp_rad, chassis_cfg::manual_gimbal_phase_comp_rad,
            actual_vx, actual_vy, actual_wz, &vx, &vy, chassis_cfg::manual_dynamic_k_base);
        Sentry::Utils::normalize_planar_speed(vx, vy, chassis_cfg::max_planar_speed);
    }

    ALG::Omni::WheelOutput output[WHEEL_COUNT]{};

    kinematics_.inverse_kinematics(vx, vy, wz, output);

    float logical_ref[WHEEL_COUNT]{};
    float max_abs_ref = 0.0f;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        const float wheel_radps =
            output[i].speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
        logical_ref[i] = wheel_[i].offline ? 0.0f : wheel_radps;
        max_abs_ref = std::fmax(max_abs_ref, std::fabs(logical_ref[i]));
    }
    const float max_wheel_speed_radps =
        chassis_cfg::max_wheel_speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
    if (max_wheel_speed_radps > 0.0f && max_abs_ref > max_wheel_speed_radps)
    {
        const float scale = max_wheel_speed_radps / max_abs_ref;
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            logical_ref[i] *= scale;
        }
        vx *= scale;
        vy *= scale;
        wz *= scale;
    }

    const bool use_force_ff = force_ff_config_valid() && dynamics_.IsAllocationValid();
    float inertial_torque_nm[WHEEL_COUNT]{};
    float speed_torque_nm[WHEEL_COUNT]{};
    if (use_force_ff)
    {
        const float ff_vx = planner_vx_.update(vx, DT_SEC);
        const float ff_vy = planner_vy_.update(vy, DT_SEC);
        const float ff_wz = planner_wz_.update(wz, DT_SEC);
        const float ref_ax = planner_vx_.GetAcceleration();
        const float ref_ay = planner_vy_.GetAcceleration();
        const float ref_awz = planner_wz_.GetAcceleration();

        dynamics_.OmniInvDynamics(chassis_cfg::chassis_mass_kg * ref_ax,
                                  chassis_cfg::chassis_mass_kg * ref_ay,
                                  chassis_cfg::chassis_yaw_inertia_kgm2 * ref_awz);
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            inertial_torque_nm[i] = dynamics_.GetMotorTorque(i);
        }

        const bool turning_translation_requested =
            (vx * vx + vy * vy) > 1.0e-12f && std::fabs(wz) > 1.0e-6f;
        dynamics_.OmniInvDynamics(
            turning_translation_requested ? -chassis_cfg::chassis_mass_kg * ff_wz * ff_vy : 0.0f,
            turning_translation_requested ? chassis_cfg::chassis_mass_kg * ff_wz * ff_vx : 0.0f,
            0.0f);

        ALG::Omni::WheelOutput angular_accel[WHEEL_COUNT]{};
        kinematics_.inverse_kinematics(ref_ax, ref_ay, ref_awz, angular_accel);
        for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
        {
            const float wheel_alpha =
                angular_accel[i].speed_rpm * Sentry::Utils::Constants::RPM_TO_RAD_PER_SEC;
            inertial_torque_nm[i] += chassis_cfg::wheel_equivalent_inertia_kgm2 * wheel_alpha;
            const float wheel_speed = logical_ref[i];
            float friction = chassis_cfg::wheel_viscous_friction_nm_per_radps * wheel_speed;
            if constexpr (chassis_cfg::wheel_coulomb_friction_nm != 0.0f)
            {
                friction += chassis_cfg::wheel_coulomb_friction_nm *
                            std::tanh(wheel_speed / chassis_cfg::wheel_friction_smooth_radps);
            }
            speed_torque_nm[i] = dynamics_.GetMotorTorque(i) + friction;
        }
    }
    else
    {
        planner_vx_.reset(vx);
        planner_vy_.reset(vy);
        planner_wz_.reset(wz);
    }

    float requested_raw[WHEEL_COUNT]{};
    float requested_a[WHEEL_COUNT]{};
    float rotor_radps[WHEEL_COUNT]{};
    bool active[WHEEL_COUNT]{};
    const float amp_per_raw = chassis_cfg::c620_current_max_a / chassis_cfg::c620_command_max_raw;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        if (wheel_[i].offline || !std::isfinite(feedback[i].output_speed_radps) ||
            !std::isfinite(feedback[i].rotor_speed_radps))
        {
            reset_wheel_control(i);
            continue;
        }

        ref_wheel_radps_[i] = logical_ref[i];
        const float fb_radps = feedback[i].output_speed_radps * wheel_sign_[i];
        float ff_raw = 0.0f;
        if (use_force_ff && wheel_[i].fb_online)
        {
            const float inertial = ALG::Omni::guard_chassis_inertial_ff(
                inertial_torque_nm[i], ref_wheel_radps_[i] - fb_radps,
                chassis_cfg::force_ff_inertial_fade_error_radps);

            ff_raw = torque_to_raw_unclamped(inertial + speed_torque_nm[i]) * wheel_sign_[i];
        }
        last_ff_current_raw_[i] = ff_raw;
        const float pid_raw = wheel_[i].pid.update(ref_wheel_radps_[i], fb_radps, DT_SEC);

        const float total_raw = ff_raw + pid_raw * wheel_sign_[i];
        if (!std::isfinite(total_raw))
        {
            reset_wheel_control(i);
            continue;
        }
        requested_raw[i] = Sentry::Utils::clamp(total_raw, chassis_cfg::wheel_pid_output_min,
                                                chassis_cfg::wheel_pid_output_max);

        requested_a[i] = std::trunc(requested_raw[i]) * amp_per_raw;
        rotor_radps[i] = feedback[i].rotor_speed_radps;
        active[i] = true;
    }

    float fixed_power_w = chassis_cfg::power_fixed_w;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        if (!active[i])
        {
            const float idle = std::fmax(0.0f, chassis_cfg::power_model_coeffs[0]);
            fixed_power_w += idle;
        }
    }
    ALG::Omni::PowerControl<WHEEL_COUNT>::Result power_result{};
    if (chassis_cfg::power_control_enabled)
    {

        power_result = power_control_.DecayingCurrent(
            requested_a, rotor_radps, chassis_cfg::power_model_coeffs, nullptr, fixed_power_w,
            power_strategy_.GetInputLimit(), active);
    }
    const bool power_fault = !power_result.valid || !power_result.feasible;
    if (power_fault != power_model_fault_)
    {
        if (power_fault)
        {
        }
        else
        {
        }
        power_model_fault_ = power_fault;
    }

    bool any_enabled = false;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        active[i] = active[i] && !power_fault;
        if (active[i])
        {

            last_wheel_current_[i] =
                chassis_cfg::power_control_enabled
                    ? clamp_current(std::round(power_control_.getCurrent(i) / amp_per_raw))
                    : clamp_current(requested_raw[i]);
            any_enabled = true;
        }
        else
        {
            reset_wheel_control(i);
        }
        wheel_[i].cmd->publish(Sentry::MotorCommand{last_wheel_current_[i], active[i]});
    }
    last_output_allowed_ = any_enabled;
}

void ChassisCtrl::Stop()
{
    mode_fsm_.force(OnmiChassisMode::STOP, Sentry::Time::now_ms());
    reset_control();
    last_output_allowed_ = false;
    stop_all();
}

ALG::Omni::OmniSolver::Config ChassisCtrl::make_kinematics_config()
{
    ALG::Omni::OmniSolver::Config config{};
    config.radius = chassis_cfg::chassis_radius;
    config.scaling = chassis_cfg::wheel_radius;
    config.wheel_azimuth[FL] = chassis_cfg::wheel_azimuth_fl;
    config.wheel_azimuth[FR] = chassis_cfg::wheel_azimuth_fr;
    config.wheel_azimuth[BL] = chassis_cfg::wheel_azimuth_bl;
    config.wheel_azimuth[BR] = chassis_cfg::wheel_azimuth_br;
    config.wheel_direction[FL] = chassis_cfg::wheel_direction_fl;
    config.wheel_direction[FR] = chassis_cfg::wheel_direction_fr;
    config.wheel_direction[BL] = chassis_cfg::wheel_direction_bl;
    config.wheel_direction[BR] = chassis_cfg::wheel_direction_br;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        config.chassis_direction[i] = 1.0f;
    }
    return config;
}

ALG::Omni::Omni_ID ChassisCtrl::make_dynamics()
{
    const float wheel_azimuth[WHEEL_COUNT] = {
        chassis_cfg::wheel_azimuth_fl,
        chassis_cfg::wheel_azimuth_fr,
        chassis_cfg::wheel_azimuth_bl,
        chassis_cfg::wheel_azimuth_br,
    };
    const float wheel_direction[WHEEL_COUNT] = {
        chassis_cfg::wheel_direction_fl,
        chassis_cfg::wheel_direction_fr,
        chassis_cfg::wheel_direction_bl,
        chassis_cfg::wheel_direction_br,
    };

    return ALG::Omni::Omni_ID(chassis_cfg::chassis_radius, chassis_cfg::wheel_radius,
                              static_cast<float>(WHEEL_COUNT), wheel_azimuth, wheel_direction);
}

ALG::Omni::SCurveVelocityPlanning::Config ChassisCtrl::make_linear_planner_config()
{
    ALG::Omni::SCurveVelocityPlanning::Config config{};
    config.max_speed = chassis_cfg::max_planar_speed;
    config.max_accel = chassis_cfg::force_ff_linear_max_accel_mps2;
    config.max_decel = chassis_cfg::force_ff_linear_max_decel_mps2;
    config.max_jerk = chassis_cfg::force_ff_linear_max_jerk_mps3;
    return config;
}

ALG::Omni::SCurveVelocityPlanning::Config ChassisCtrl::make_angular_planner_config()
{
    ALG::Omni::SCurveVelocityPlanning::Config config{};
    config.max_speed = chassis_cfg::max_vw;
    config.max_accel = chassis_cfg::force_ff_angular_max_accel_radps2;
    config.max_decel = chassis_cfg::force_ff_angular_max_decel_radps2;
    config.max_jerk = chassis_cfg::force_ff_angular_max_jerk_radps3;
    return config;
}

bool ChassisCtrl::force_ff_config_valid()
{
    if (!chassis_cfg::force_ff_enabled)
    {
        return false;
    }

    const float values[] = {
        chassis_cfg::chassis_mass_kg,
        chassis_cfg::chassis_yaw_inertia_kgm2,
        chassis_cfg::wheel_equivalent_inertia_kgm2,
        chassis_cfg::wheel_coulomb_friction_nm,
        chassis_cfg::wheel_viscous_friction_nm_per_radps,
        chassis_cfg::wheel_friction_smooth_radps,
        chassis_cfg::force_ff_inertial_fade_error_radps,
    };
    for (const float value : values)
    {
        if (!std::isfinite(value) || value < 0.0f)
        {
            return false;
        }
    }
    return chassis_cfg::wheel_friction_smooth_radps > 0.0f &&
           chassis_cfg::force_ff_inertial_fade_error_radps > 0.0f &&
           (chassis_cfg::chassis_mass_kg > 0.0f || chassis_cfg::chassis_yaw_inertia_kgm2 > 0.0f ||
            chassis_cfg::wheel_equivalent_inertia_kgm2 > 0.0f ||
            chassis_cfg::wheel_coulomb_friction_nm > 0.0f ||
            chassis_cfg::wheel_viscous_friction_nm_per_radps > 0.0f);
}

bool ChassisCtrl::power_control_config_valid()
{
    for (const float coefficient : chassis_cfg::power_model_coeffs)
    {
        if (!std::isfinite(coefficient))
            return false;
    }
    return chassis_cfg::power_model_coeffs[4] >= 0.0f &&
           std::isfinite(chassis_cfg::power_fixed_w) && chassis_cfg::power_fixed_w >= 0.0f &&
           std::isfinite(chassis_cfg::c620_current_max_a) &&
           chassis_cfg::c620_current_max_a > 0.0f &&
           std::isfinite(chassis_cfg::c620_command_max_raw) &&
           chassis_cfg::c620_command_max_raw > 0.0f;
}

void ChassisCtrl::update_power_budget(std::uint32_t now)
{
    const Sentry::RefereeData referee = sentry_data.referee.read();
    const auto previous = power_strategy_.GetRefereeSource();
    power_strategy_.UpdateReferee(now, referee.online(chassis_cfg::power_referee_timeout_ms),
                                  referee.robot_status_count, referee.power_heat_count,
                                  static_cast<float>(referee.status.chassis_power_limit),
                                  static_cast<float>(referee.power.buffer_energy),
                                  referee.status.power_management_chassis_output != 0U);
    const auto source = power_strategy_.GetRefereeSource();
    power_output_allowed_ =
        !chassis_cfg::power_control_enabled || power_strategy_.GetInputLimit() > 0.0f;
    if (source != previous)
    {
    }
}

float ChassisCtrl::torque_to_raw_unclamped(float torque_nm)
{
    const float k_tau = chassis_cfg::wheel_output_torque_constant_nm_per_a;
    const float current_max = chassis_cfg::c620_current_max_a;
    const float command_max = chassis_cfg::c620_command_max_raw;
    if (!std::isfinite(torque_nm) || !std::isfinite(k_tau) || k_tau <= 0.0f ||
        !std::isfinite(current_max) || current_max <= 0.0f || !std::isfinite(command_max) ||
        command_max <= 0.0f)
    {
        return 0.0f;
    }

    return torque_nm / k_tau * command_max / current_max;
}

bool ChassisCtrl::remote_connected(const Sentry::RemoteData &remote)
{
    return dt7_connected(remote, chassis_cfg::remote_connection_timeout_ms);
}

bool ChassisCtrl::remote_allows_enable(const Sentry::RemoteData &remote)
{
    return remote_connected(remote) && !(dt7_switch(remote, DT7_CH_S1) == chassis_cfg::disable_s1 &&
                                         dt7_switch(remote, DT7_CH_S2) == chassis_cfg::disable_s2);
}

OnmiChassisMode ChassisCtrl::select_mode(const Sentry::RemoteData &remote)
{
    if (!remote_allows_enable(remote))
    {
        return OnmiChassisMode::STOP;
    }

    const std::uint8_t s1 = dt7_switch(remote, DT7_CH_S1);
    const std::uint8_t s2 = dt7_switch(remote, DT7_CH_S2);

    if (s1 == chassis_cfg::auto_s1 && s2 == chassis_cfg::auto_s2)
    {
        return OnmiChassisMode::AUTO;
    }
    if (s1 == chassis_cfg::follow_s1_a && s2 == chassis_cfg::follow_s2_a)
    {
        return OnmiChassisMode::FOLLOW;
    }
    if (s1 >= 1U && s1 <= 3U && s2 >= 1U && s2 <= 3U)
    {
        return OnmiChassisMode::NO_FOLLOW;
    }
    return OnmiChassisMode::STOP;
}

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

bool ChassisCtrl::system_safe_for_output(const Sentry::RemoteData &remote, OnmiChassisMode mode,
                                         const App::Chassis::ChassisMotionCommand &command)
{
    return remote_allows_enable(remote) &&
           command.is_online(chassis_cfg::motion_command_timeout_ms) &&
           command.has_known_source() && std::isfinite(command.vx_mps) &&
           std::isfinite(command.vy_mps) && std::isfinite(command.wz_radps) &&
           command_matches_mode(mode, command);
}

const char *ChassisCtrl::mode_name(OnmiChassisMode mode)
{
    switch (mode)
    {
    case OnmiChassisMode::STOP:
        return "STOP";
    case OnmiChassisMode::FOLLOW:
        return "FOLLOW";
    case OnmiChassisMode::NO_FOLLOW:
        return "NO_FOLLOW";
    case OnmiChassisMode::AUTO:
        return "AUTO";
    case OnmiChassisMode::COUNT:
        return "COUNT";
    }
    return "?";
}

bool ChassisCtrl::update_fb_offline(const Sentry::MotorFeedback &feedback, std::uint32_t now,
                                    std::uint32_t &offline_since_ms)
{
    if (feedback.is_online(chassis_cfg::motor_fb_soft_timeout_ms))
    {
        offline_since_ms = 0U;
        return false;
    }
    if (feedback.timestamp == 0U)
    {
        offline_since_ms = now;
        return true;
    }
    if (offline_since_ms == 0U)
    {
        offline_since_ms = now;
    }
    return fb_age_ms(now, feedback) >= chassis_cfg::motor_fb_hard_timeout_ms;
}

std::uint32_t ChassisCtrl::fb_age_ms(std::uint32_t now, const Sentry::MotorFeedback &feedback)
{
    return feedback.timestamp == 0U ? 0xFFFFFFFFUL : now - feedback.timestamp;
}

std::int16_t ChassisCtrl::clamp_current(float value)
{
    if (!std::isfinite(value))
    {
        return 0;
    }
    return static_cast<std::int16_t>(Sentry::Utils::clamp(value, chassis_cfg::wheel_pid_output_min,
                                                          chassis_cfg::wheel_pid_output_max));
}

void ChassisCtrl::reset_wheel_control(std::uint8_t index)
{
    if (index >= WHEEL_COUNT)
    {
        return;
    }
    wheel_[index].pid.reset();
    ref_wheel_radps_[index] = 0.0f;
    last_ff_current_raw_[index] = 0.0f;
    last_wheel_current_[index] = 0;
}

const char *ChassisCtrl::wheel_name(std::uint8_t index)
{
    switch (index)
    {
    case FL:
        return "FL";
    case FR:
        return "FR";
    case BL:
        return "BL";
    case BR:
        return "BR";
    default:
        return "?";
    }
}

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

ChassisCtrl::Inputs ChassisCtrl::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.command = sentry_data.motion_command.read();

    update_power_budget(in.nowMs);

    in.anyOffline = false;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        in.feedback[i] = wheel_[i].fb->read();
        const bool was_fb_online = wheel_[i].fb_online;
        wheel_[i].fb_online = in.feedback[i].timestamp != 0U &&
                              in.feedback[i].is_online(chassis_cfg::motor_fb_soft_timeout_ms);

        const bool was_offline = wheel_[i].offline;
        wheel_[i].offline = update_fb_offline(in.feedback[i], in.nowMs, wheel_offline_since_ms_[i]);
        if (wheel_[i].offline != was_offline)
        {
            reset_wheel_control(i);
        }
        in.anyOffline = in.anyOffline || wheel_[i].offline;
    }

    return in;
}

ChassisCtrl::ChassisCtrl() : kinematics_(make_kinematics_config()), dynamics_(make_dynamics()) {}

bool ChassisCtrl::Init()
{

    if (chassis_cfg::power_control_enabled && !power_control_config_valid())
    {

        return false;
    }
    ALG::Omni::PowerControlStrategy::RefereeConfig power_config{};
    power_config.fallback_w = chassis_cfg::power_fallback_w;
    power_config.margin_w = chassis_cfg::power_margin_w;
    power_config.timeout_ms = chassis_cfg::power_referee_timeout_ms;
    power_config.buffer_low_j = chassis_cfg::power_buffer_low_j;
    power_config.buffer_recover_j = chassis_cfg::power_buffer_recover_j;
    power_config.low_buffer_scale = chassis_cfg::power_low_buffer_scale;
    if (!power_strategy_.ConfigureReferee(power_config) && chassis_cfg::power_control_enabled)
    {

        return false;
    }
    power_control_.configure({chassis_cfg::power_model_absolute_speed_loss, false,
                              chassis_cfg::c620_current_max_a / chassis_cfg::c620_command_max_raw});

    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        wheel_[i].name = wheel_name(i);
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

namespace
{

constexpr std::size_t DT7_CH_LEFT_Y = 0U;
constexpr std::size_t DT7_CH_LEFT_X = 1U;

constexpr std::size_t DT7_CH_LEFT_SCROLL = 5U;

float remote_channel_normalized(const Sentry::RemoteData &remote, std::size_t index)
{
    switch (index)
    {
    case DT7_CH_LEFT_Y:
        return remote.left_y;
    case DT7_CH_LEFT_X:
        return remote.left_x;
    case DT7_CH_LEFT_SCROLL:
        return remote.scroll_norm;
    default:
        return 0.0f;
    }
}

float apply_remote_deadband(float value)
{
    const float deadband = chassis_cfg::sbus_analog_deadband;
    const float magnitude = std::fabs(value);
    if (deadband <= 0.0f || magnitude <= deadband)
    {
        return deadband > 0.0f ? 0.0f : value;
    }
    if (deadband >= 1.0f)
    {
        return 0.0f;
    }

    const float scaled = (magnitude - deadband) / (1.0f - deadband);
    return value < 0.0f ? -scaled : scaled;
}

} // namespace

void ChassisCmd::Update()
{
    // 仲裁本地遥控器与云台请求，校验新鲜度后发布唯一的底盘指令。

    const auto in = readInputs();

    const bool host_request_fresh = in.host.is_online(chassis_cfg::host_request_timeout_ms);

    const bool gimbal_gate_fresh = in.gate.is_online(chassis_cfg::gimbal_motion_gate_timeout_ms);
    const bool gimbal_switch_lock = (gimbal_gate_fresh && in.gate.locks_chassis()) ||
                                    (in.gate.timestamp != 0U && !gimbal_gate_fresh &&
                                     in.gate.state != App::Chassis::GimbalMotionGateState::NORMAL);

    std::uint8_t flags = 0U;
    if (remote_connected(in.remote))
    {
        flags |= App::Chassis::CHASSIS_CMD_REMOTE_FRESH;
    }
    if (host_request_fresh)
    {
        flags |= App::Chassis::CHASSIS_CMD_HOST_REQUEST_FRESH;
    }

    App::Chassis::ChassisMotionCommand command = make_stop(in.nowMs, flags);
    const Selection selection = select(in.remote);
    bool limited = false;

    switch (selection)
    {
    case Selection::Auto:
        command.flags |= App::Chassis::CHASSIS_CMD_NAVIGATION_SELECTED;

        command.source = App::Chassis::ChassisMotionSource::Navigation;
        follow_pid_.reset();
        if (host_request_fresh && is_finite(in.host.vx_mps) && is_finite(in.host.vy_mps) &&
            is_finite(in.host.wz_radps))
        {
            command.vx_mps = in.host.vx_mps;
            command.vy_mps = in.host.vy_mps;
            command.wz_radps = in.host.wz_radps;
        }
        break;

    case Selection::Manual:
    {
        const float left_y =
            apply_remote_deadband(remote_channel_normalized(in.remote, DT7_CH_LEFT_Y));
        const float left_x =
            apply_remote_deadband(remote_channel_normalized(in.remote, DT7_CH_LEFT_X));
        const float scroll =
            apply_remote_deadband(remote_channel_normalized(in.remote, DT7_CH_LEFT_SCROLL));

        const bool shooter_scroll_owned = shooter_owns_scroll(in.remote);
        if (!is_finite(left_y) || !is_finite(left_x) || !is_finite(scroll))
        {
            follow_pid_.reset();
            break;
        }

        command.vx_mps = Sentry::Utils::clamp(left_y, -1.0f, 1.0f) * chassis_cfg::max_planar_speed;
        command.vy_mps = -Sentry::Utils::clamp(left_x, -1.0f, 1.0f) * chassis_cfg::max_planar_speed;

        if (follow_selected(in.remote))
        {
            const App::Chassis::GimbalYawJointFeedback yaw =
                true ? sentry_data.yaw_joint.read() : App::Chassis::GimbalYawJointFeedback{};
            if (yaw.is_online(chassis_cfg::manual_gimbal_yaw_timeout_ms) &&
                is_finite(yaw.yaw_angle_rad))
            {
                const float sign = chassis_cfg::manual_gimbal_yaw_sign;
                command.wz_radps = Sentry::Utils::calculate_follow_wz(
                    sign * chassis_cfg::manual_gimbal_yaw_zero_rad, sign * yaw.yaw_angle_rad,
                    follow_pid_, DT_SEC, chassis_cfg::follow_yaw_deadband_rad,
                    chassis_cfg::follow_yaw_half_range_rad);
            }
            else
            {
                follow_pid_.reset();
                command.wz_radps = 0.0f;
            }
        }
        else
        {
            follow_pid_.reset();
            command.wz_radps = shooter_scroll_owned ? 0.0f
                                                    : Sentry::Utils::clamp(scroll, -1.0f, 1.0f) *
                                                          chassis_cfg::max_vw;
        }

        command.source = App::Chassis::ChassisMotionSource::Manual;
        break;
    }
    case Selection::Stop:
        follow_pid_.reset();
        break;
    }

    limited = !std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
              !std::isfinite(command.wz_radps) ||
              std::fabs(command.vx_mps) > chassis_cfg::max_planar_speed ||
              std::fabs(command.vy_mps) > chassis_cfg::max_planar_speed ||
              std::hypot(command.vx_mps, command.vy_mps) > chassis_cfg::max_planar_speed ||
              std::fabs(command.wz_radps) > chassis_cfg::max_vw;

    if (gimbal_switch_lock)
    {

        if (command.source == App::Chassis::ChassisMotionSource::Stop &&
            remote_allows_enable(in.remote))
        {
            command.source = selection == Selection::Auto
                                 ? App::Chassis::ChassisMotionSource::Navigation
                                 : App::Chassis::ChassisMotionSource::Manual;
        }
        command.vx_mps = 0.0f;
        command.vy_mps = 0.0f;
        command.wz_radps = 0.0f;
        command.flags |= App::Chassis::CHASSIS_CMD_GIMBAL_TRANSITION_LOCK;
        follow_pid_.reset();
    }

    if (limited)
    {
        command.flags |= App::Chassis::CHASSIS_CMD_VALUE_LIMITED;
    }

    publish(command);
}

void ChassisCmd::Stop()
{
    follow_pid_.reset();
    publish(make_stop(Sentry::Time::now_ms(), 0U));
}

bool ChassisCmd::is_finite(float value)
{
    return std::isfinite(value);
}

bool ChassisCmd::remote_connected(const Sentry::RemoteData &remote)
{
    return dt7_connected(remote, chassis_cfg::remote_connection_timeout_ms);
}

bool ChassisCmd::remote_allows_enable(const Sentry::RemoteData &remote)
{
    return remote_connected(remote) && !(dt7_switch(remote, DT7_CH_S1) == chassis_cfg::disable_s1 &&
                                         dt7_switch(remote, DT7_CH_S2) == chassis_cfg::disable_s2);
}

bool ChassisCmd::auto_selected(const Sentry::RemoteData &remote)
{
    return remote_allows_enable(remote) && dt7_switch(remote, DT7_CH_S1) == chassis_cfg::auto_s1 &&
           dt7_switch(remote, DT7_CH_S2) == chassis_cfg::auto_s2;
}

bool ChassisCmd::follow_selected(const Sentry::RemoteData &remote)
{
    if (!remote_allows_enable(remote))
    {
        return false;
    }
    const std::uint8_t s1 = dt7_switch(remote, DT7_CH_S1);
    const std::uint8_t s2 = dt7_switch(remote, DT7_CH_S2);
    return (s1 == chassis_cfg::follow_s1_a && s2 == chassis_cfg::follow_s2_a);
}

bool ChassisCmd::shooter_owns_scroll(const Sentry::RemoteData &remote)
{

    return remote_allows_enable(remote) &&
           dt7_switch(remote, DT7_CH_S1) == chassis_cfg::shooter_scroll_s1;
}

ChassisCmd::Selection ChassisCmd::select(const Sentry::RemoteData &remote)
{
    if (!remote_allows_enable(remote))
    {
        return Selection::Stop;
    }
    if (auto_selected(remote))
    {
        return Selection::Auto;
    }
    return Selection::Manual;
}

App::Chassis::ChassisMotionCommand ChassisCmd::make_stop(std::uint32_t now_ms, std::uint8_t flags)
{
    App::Chassis::ChassisMotionCommand command{};
    command.timestamp = now_ms;
    command.source = App::Chassis::ChassisMotionSource::Stop;
    command.flags = flags;
    return command;
}

void ChassisCmd::publish(App::Chassis::ChassisMotionCommand command)
{
    {
        sentry_data.motion_command.publish(command);
    }
}

ChassisCmd::Inputs ChassisCmd::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.host = true ? sentry_data.host.read() : App::Chassis::HostMotionRequest{};
    in.gate = true ? sentry_data.motion_gate.read() : App::Chassis::GimbalMotionGate{};
    in.heading = true ? sentry_data.heading.read() : App::Chassis::HeadingFeedback{};
    in.yawJoint = true ? sentry_data.yaw_joint.read() : App::Chassis::GimbalYawJointFeedback{};
    return in;
}

ChassisCmd::ChassisCmd() {}

bool ChassisCmd::Init()
{

    follow_pid_.set_config(ALG::Omni::TimePid::Config{
        chassis_cfg::follow_yaw_pid_kp,
        chassis_cfg::follow_yaw_pid_ki,
        chassis_cfg::follow_yaw_pid_kd,
        chassis_cfg::follow_yaw_pid_output_max,
        chassis_cfg::follow_yaw_pid_output_min,
        chassis_cfg::follow_yaw_pid_integral_max,
    });
    follow_pid_.reset();
    sentry_data.motion_command.publish(make_stop(Sentry::Time::now_ms(), 0U));
    return true;
}

namespace
{
constexpr float PI = 3.14159265358979323846f;
}

void ChassisOdom::Update()
{
    // 合并有效轮速和航向反馈，更新并发布底盘里程计。

    const auto in = readInputs();

    const bool wheels_fresh = in.validWheels == WHEEL_COUNT;
    ALG::Omni::ChassisOdometry velocity{};
    bool kinematics_valid = false;
    bool stationary_gated = false;
    if (wheels_fresh)
    {
        kinematics_.forward_kinematics(in.wheelRpm, velocity);
        kinematics_valid = finite(velocity.vx) && finite(velocity.vy) && finite(velocity.vw) &&
                           std::hypot(velocity.vx, velocity.vy) <= odom_cfg::max_linear_speed_mps &&
                           std::fabs(velocity.vw) <= odom_cfg::max_angular_speed_radps;
    }
    if (kinematics_valid)
    {
        if (std::fabs(velocity.vx) < odom_cfg::linear_deadband_mps)
        {
            velocity.vx = 0.0f;
        }
        if (std::fabs(velocity.vy) < odom_cfg::linear_deadband_mps)
        {
            velocity.vy = 0.0f;
        }
        if (std::fabs(velocity.vw) < odom_cfg::angular_deadband_radps)
        {
            velocity.vw = 0.0f;
        }
        stationary_gated = apply_stationary_gate(velocity);
        state_.vx_body_mps = velocity.vx;
        state_.vy_body_mps = velocity.vy;
        state_.wz_radps = velocity.vw;
    }
    else
    {
        stationary_lock_ = true;
        state_.vx_body_mps = 0.0f;
        state_.vy_body_mps = 0.0f;
        state_.wz_radps = 0.0f;
    }

    const App::Chassis::HeadingFeedback heading = sentry_data.heading.read();
    const bool heading_fresh =
        heading.is_online(odom_cfg::heading_timeout_ms) && finite(heading.yaw_rad);
    bool heading_jump = false;
    float integration_yaw = state_.yaw_rad;
    const bool was_ready = ready_;
    if (heading_fresh)
    {
        const float yaw = wrap_angle(heading.yaw_rad);
        state_.heading_timestamp = heading.timestamp;
        if (!ready_)
        {
            state_.yaw_rad = yaw;
            ready_ = true;
        }
        else
        {
            const float delta = wrap_angle(yaw - state_.yaw_rad);
            if (std::fabs(delta) > odom_cfg::max_heading_jump_rad)
            {
                state_.yaw_rad = yaw;
                heading_jump = true;
            }
            else
            {
                integration_yaw = wrap_angle(state_.yaw_rad + 0.5f * delta);
                state_.yaw_rad = yaw;
            }
        }
    }

    const bool dt_valid =
        was_ready && in.dtMs >= odom_cfg::min_update_dt_ms && in.dtMs <= odom_cfg::max_update_dt_ms;
    std::uint8_t flags = 0U;
    if (ready_)
        flags |= App::Chassis::ODOM_INITIALIZED;
    if (heading_fresh)
        flags |= App::Chassis::ODOM_HEADING_FRESH;
    if (wheels_fresh)
        flags |= App::Chassis::ODOM_WHEELS_FRESH;
    if (kinematics_valid)
        flags |= App::Chassis::ODOM_KINEMATICS_VALID;
    if (dt_valid)
        flags |= App::Chassis::ODOM_DT_VALID;
    if (heading_jump)
        flags |= App::Chassis::ODOM_HEADING_JUMP;
    if (stationary_gated)
        flags |= App::Chassis::ODOM_STATIONARY_GATED;

    if (heading_fresh && kinematics_valid && dt_valid && !heading_jump && !stationary_gated)
    {
        const float dt_sec = static_cast<float>(in.dtMs) * 0.001f;
        const float cos_yaw = std::cos(integration_yaw);
        const float sin_yaw = std::sin(integration_yaw);
        state_.x_m += (velocity.vx * cos_yaw - velocity.vy * sin_yaw) * dt_sec;
        state_.y_m += (velocity.vx * sin_yaw + velocity.vy * cos_yaw) * dt_sec;
        flags |= App::Chassis::ODOM_POSITION_INTEGRATED;
    }
    publish(in.nowMs, flags, in.validWheels);
}

void ChassisOdom::Stop()
{
    state_.vx_body_mps = 0.0f;
    state_.vy_body_mps = 0.0f;
    state_.wz_radps = 0.0f;
    publish(Sentry::Time::now_ms(), 0U, 0U);
}

ALG::Omni::OmniSolver::Config ChassisOdom::make_kinematics_config()
{
    ALG::Omni::OmniSolver::Config config{};
    config.radius = chassis_cfg::chassis_radius;
    config.scaling = chassis_cfg::wheel_radius;
    config.wheel_azimuth[FL] = chassis_cfg::wheel_azimuth_fl;
    config.wheel_azimuth[FR] = chassis_cfg::wheel_azimuth_fr;
    config.wheel_azimuth[BL] = chassis_cfg::wheel_azimuth_bl;
    config.wheel_azimuth[BR] = chassis_cfg::wheel_azimuth_br;
    config.wheel_direction[FL] = chassis_cfg::wheel_direction_fl;
    config.wheel_direction[FR] = chassis_cfg::wheel_direction_fr;
    config.wheel_direction[BL] = chassis_cfg::wheel_direction_bl;
    config.wheel_direction[BR] = chassis_cfg::wheel_direction_br;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        config.chassis_direction[i] = 1.0f;
    }
    return config;
}

bool ChassisOdom::finite(float value)
{
    return std::isfinite(value);
}

float ChassisOdom::wrap_angle(float value)
{
    float wrapped = std::fmod(value, 2.0f * PI);
    if (wrapped > PI)
    {
        wrapped -= 2.0f * PI;
    }
    else if (wrapped < -PI)
    {
        wrapped += 2.0f * PI;
    }
    return wrapped;
}

bool ChassisOdom::apply_stationary_gate(ALG::Omni::ChassisOdometry &velocity)
{
    const float planar = std::hypot(velocity.vx, velocity.vy);
    const float angular = std::fabs(velocity.vw);
    if (stationary_lock_)
    {
        if (planar > odom_cfg::stationary_exit_linear_mps ||
            angular > odom_cfg::stationary_exit_angular_radps)
        {
            stationary_lock_ = false;
        }
    }
    else if (planar < odom_cfg::stationary_enter_linear_mps &&
             angular < odom_cfg::stationary_enter_angular_radps)
    {
        stationary_lock_ = true;
    }
    if (!stationary_lock_)
    {
        return false;
    }
    velocity.vx = 0.0f;
    velocity.vy = 0.0f;
    velocity.vw = 0.0f;
    return true;
}

void ChassisOdom::publish(std::uint32_t now, std::uint8_t flags, std::uint8_t valid_wheel_count)
{
    if (false)
    {
        return;
    }
    state_.timestamp = now;
    state_.flags = flags;
    state_.valid_wheel_count = valid_wheel_count;
    state_.gimbal_motion_lock = gimbal_motion_lock_ ? 1U : 0U;
    sentry_data.odometry.publish(state_);
}

ChassisOdom::Inputs ChassisOdom::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    const App::Chassis::GimbalMotionGate gimbal_gate =
        true ? sentry_data.motion_gate.read() : App::Chassis::GimbalMotionGate{};
    const bool gate_fresh = gimbal_gate.is_online(chassis_cfg::gimbal_motion_gate_timeout_ms);
    gimbal_motion_lock_ = (gate_fresh && gimbal_gate.locks_chassis()) ||
                          (gimbal_gate.timestamp != 0U && !gate_fresh &&
                           gimbal_gate.state != App::Chassis::GimbalMotionGateState::NORMAL);
    in.dtMs = in.nowMs - last_update_ms_;
    last_update_ms_ = in.nowMs;

    constexpr float wheel_sign[WHEEL_COUNT] = {
        chassis_cfg::wheel_sign_fl,
        chassis_cfg::wheel_sign_fr,
        chassis_cfg::wheel_sign_bl,
        chassis_cfg::wheel_sign_br,
    };
    in.validWheels = 0U;
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        const Sentry::MotorFeedback feedback = wheel_sub_[i]->read();
        if (feedback.is_online(odom_cfg::motor_fb_timeout_ms) && finite(feedback.output_speed_rpm))
        {
            in.wheelRpm[i] = feedback.output_speed_rpm * wheel_sign[i];
            ++in.validWheels;
        }
    }
    return in;
}

ChassisOdom::ChassisOdom() : kinematics_(make_kinematics_config()) {}

bool ChassisOdom::Init()
{
    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        wheel_sub_[i] = &sentry_data.wheel_fb[i];
    }

    state_ = App::Chassis::ChassisOdometry{};
    last_update_ms_ = Sentry::Time::now_ms();
    ready_ = false;
    stationary_lock_ = true;

    return true;
}
