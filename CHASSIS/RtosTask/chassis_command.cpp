#include "chassis_command.hpp"
#include "RtosTask/robot_config.hpp"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cmath>

namespace
{
/**
 * @brief 按配置对遥控器模拟通道进行死区处理并重新缩放。
 * @param value 归一化的遥控通道值。
 * @return 去除死区并保留方向的归一化通道值。
 */
float apply_remote_deadband(float value)
{
    const float deadband = chassis_cfg::chassis.input.analog_deadband;
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

/**
 * @brief 构造底盘运动指令生成器。
 */
ChassisCmd::ChassisCmd() = default;

/**
 * @brief 配置云台跟随 PID 并发布初始停止指令。
 * @return 初始化完成后返回 true。
 */
bool ChassisCmd::Init()
{
    follow_pid_.set_config(ALG::Omni::TimePid::Config{
        chassis_cfg::chassis.follow_yaw.angle_pid.kp,
        chassis_cfg::chassis.follow_yaw.angle_pid.ki,
        chassis_cfg::chassis.follow_yaw.angle_pid.kd,
        chassis_cfg::chassis.follow_yaw.angle_pid.output_max_radps,
        chassis_cfg::chassis.follow_yaw.angle_pid.output_min_radps,
        chassis_cfg::chassis.follow_yaw.angle_pid.integral_max_rad_s,
    });
    follow_pid_.reset();
    sentry_data.motion_command.publish(make_stop(Sentry::Time::now_ms(), 0U));
    return true;
}

/**
 * @brief 仲裁遥控和主机导航请求，处理云台运动锁并发布本周期底盘指令。
 */
void ChassisCmd::Update()
{
    const auto in = readInputs();

    const bool host_request_fresh = in.host.is_online(chassis_cfg::chassis.timeouts.host_request_ms);

    const bool gimbal_gate_fresh = in.gate.is_online(chassis_cfg::chassis.timeouts.gimbal_motion_gate_ms);
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

    switch (selection)
    {
    case Selection::Auto:
        command.flags |= App::Chassis::CHASSIS_CMD_NAVIGATION_SELECTED;
        command.source = App::Chassis::ChassisMotionSource::Navigation;
        follow_pid_.reset();
        if (host_request_fresh && std::isfinite(in.host.vx_mps) && std::isfinite(in.host.vy_mps) &&
            std::isfinite(in.host.wz_radps))
        {
            command.vx_mps = in.host.vx_mps;
            command.vy_mps = in.host.vy_mps;
            command.wz_radps = in.host.wz_radps;
        }
        break;

    case Selection::Manual:
    {
        const float left_y = apply_remote_deadband(in.remote.left_y);
        const float left_x = apply_remote_deadband(in.remote.left_x);
        const float scroll = apply_remote_deadband(in.remote.scroll_norm);

        const bool shooter_scroll_owned = shooter_owns_scroll(in.remote);
        if (!std::isfinite(left_y) || !std::isfinite(left_x) || !std::isfinite(scroll))
        {
            follow_pid_.reset();
            break;
        }

        command.vx_mps = Sentry::Utils::clamp(left_y, -1.0f, 1.0f) * chassis_cfg::chassis.limits.max_planar_speed_mps;
        command.vy_mps = -Sentry::Utils::clamp(left_x, -1.0f, 1.0f) * chassis_cfg::chassis.limits.max_planar_speed_mps;

        if (follow_selected(in.remote))
        {
            const auto yaw = sentry_data.yaw_joint.read();
            if (yaw.is_online(chassis_cfg::chassis.manual.gimbal_yaw_timeout_ms) &&
                std::isfinite(yaw.yaw_angle_rad))
            {
                const float sign = chassis_cfg::chassis.manual.gimbal_yaw_sign;
                command.wz_radps = Sentry::Utils::calculate_follow_wz(
                    sign * chassis_cfg::chassis.manual.gimbal_yaw_zero_rad, sign * yaw.yaw_angle_rad,
                    follow_pid_, DT_SEC, chassis_cfg::chassis.follow_yaw.deadband_rad,
                    chassis_cfg::chassis.follow_yaw.half_range_rad);
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
                                                          chassis_cfg::chassis.limits.max_angular_speed_radps;
        }

        command.source = App::Chassis::ChassisMotionSource::Manual;
        break;
    }
    case Selection::Stop:
        follow_pid_.reset();
        break;
    }

    const bool limited =
        !std::isfinite(command.vx_mps) || !std::isfinite(command.vy_mps) ||
        !std::isfinite(command.wz_radps) ||
        std::fabs(command.vx_mps) > chassis_cfg::chassis.limits.max_planar_speed_mps ||
        std::fabs(command.vy_mps) > chassis_cfg::chassis.limits.max_planar_speed_mps ||
        std::hypot(command.vx_mps, command.vy_mps) > chassis_cfg::chassis.limits.max_planar_speed_mps ||
        std::fabs(command.wz_radps) > chassis_cfg::chassis.limits.max_angular_speed_radps;

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

    sentry_data.motion_command.publish(command);
}

/**
 * @brief 复位云台跟随 PID 并发布停止指令。
 */
void ChassisCmd::Stop()
{
    follow_pid_.reset();
    sentry_data.motion_command.publish(make_stop(Sentry::Time::now_ms(), 0U));
}

/**
 * @brief 读取生成底盘指令所需的时间、遥控器、主机请求及云台运动许可。
 * @return 本周期的输入快照。
 */
ChassisCmd::Inputs ChassisCmd::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.host = sentry_data.host.read();
    in.gate = sentry_data.motion_gate.read();
    return in;
}

/**
 * @brief 按底盘遥控超时配置判断遥控器是否在线。
 * @param remote 遥控器数据快照。
 * @return 收到过反馈且反馈未超时时返回 true。
 */
bool ChassisCmd::remote_connected(const Sentry::RemoteData &remote)
{
    return Sentry::Time::fresh(remote.timestamp, chassis_cfg::chassis.timeouts.remote_connection_ms);
}

/**
 * @brief 判断遥控器是否允许生成底盘运动指令。
 * @param remote 遥控器数据快照。
 * @return 遥控器在线且拨杆未处于禁用组合时返回 true。
 */
bool ChassisCmd::remote_allows_enable(const Sentry::RemoteData &remote)
{
    const auto &switches = chassis_cfg::chassis.input.switches;
    return remote_connected(remote) &&
           !(remote.s1 == switches.disable_s1 && remote.s2 == switches.disable_s2);
}

/**
 * @brief 判断遥控器拨杆是否选择自动导航。
 * @param remote 遥控器数据快照。
 * @return 遥控器允许工作且选择自动导航时返回 true。
 */
bool ChassisCmd::auto_selected(const Sentry::RemoteData &remote)
{
    return remote_allows_enable(remote) &&
           remote.s1 == chassis_cfg::chassis.input.switches.auto_s1 &&
           remote.s2 == chassis_cfg::chassis.input.switches.auto_s2;
}

/**
 * @brief 判断遥控器拨杆是否选择底盘跟随云台。
 * @param remote 遥控器数据快照。
 * @return 遥控器允许工作且选择跟随时返回 true。
 */
bool ChassisCmd::follow_selected(const Sentry::RemoteData &remote)
{
    if (!remote_allows_enable(remote))
    {
        return false;
    }
    const std::uint8_t s1 = remote.s1;
    const std::uint8_t s2 = remote.s2;
    return (s1 == chassis_cfg::chassis.input.switches.follow_s1 && s2 == chassis_cfg::chassis.input.switches.follow_s2);
}

/**
 * @brief 判断当前拨杆档位是否将遥控滚轮分配给发射控制。
 * @param remote 遥控器数据快照。
 * @return 滚轮分配给发射控制时返回 true。
 */
bool ChassisCmd::shooter_owns_scroll(const Sentry::RemoteData &remote)
{
    return remote_allows_enable(remote) &&
           remote.s1 == chassis_cfg::chassis.input.switches.shooter_scroll_s1;
}

/**
 * @brief 根据遥控状态选择底盘运动指令来源。
 * @param remote 遥控器数据快照。
 * @return 停止、手动或自动来源。
 */
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

/**
 * @brief 构造带时间戳和状态标志的零速度指令。
 * @param now_ms 指令发布的毫秒时间戳。
 * @param flags 需随指令保留的状态标志位。
 * @return 底盘停止指令。
 */
App::Chassis::ChassisMotionCommand ChassisCmd::make_stop(std::uint32_t now_ms, std::uint8_t flags)
{
    App::Chassis::ChassisMotionCommand command{};
    command.timestamp = now_ms;
    command.source = App::Chassis::ChassisMotionSource::Stop;
    command.flags = flags;
    return command;
}
