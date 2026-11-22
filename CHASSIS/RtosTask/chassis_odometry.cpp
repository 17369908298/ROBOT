#include "chassis_odometry.hpp"
#include "chassis_kinematics_config.hpp"
#include <cmath>

namespace
{
constexpr float PI = 3.14159265358979323846f;
} // namespace

/**
 * @brief 构造用于估算机体速度的全向轮运动学求解器。
 */
ChassisOdom::ChassisOdom() : kinematics_(App::Chassis::make_kinematics_config())
{
}

/**
 * @brief 绑定四轮反馈通道，初始化位姿、积分时间和静止锁状态。
 * @return 初始化完成后返回 true。
 */
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

/**
 * @brief 由四轮反馈解算机体速度，结合有效航向积分平面位移并发布里程计。
 */
void ChassisOdom::Update()
{
    const auto in = readInputs();

    const bool wheels_fresh = in.validWheels == WHEEL_COUNT;
    ALG::Omni::ChassisOdometry velocity{};
    bool kinematics_valid = false;
    bool stationary_gated = false;
    if (wheels_fresh)
    {
        kinematics_.forward_kinematics(in.wheelRpm, velocity);
        kinematics_valid = std::isfinite(velocity.vx) && std::isfinite(velocity.vy) &&
                           std::isfinite(velocity.vw) &&
                           std::hypot(velocity.vx, velocity.vy) <= odom_cfg::odometry.limits.max_linear_speed_mps &&
                           std::fabs(velocity.vw) <= odom_cfg::odometry.limits.max_angular_speed_radps;
    }
    if (kinematics_valid)
    {
        if (std::fabs(velocity.vx) < odom_cfg::odometry.deadband.linear_mps)
        {
            velocity.vx = 0.0f;
        }
        if (std::fabs(velocity.vy) < odom_cfg::odometry.deadband.linear_mps)
        {
            velocity.vy = 0.0f;
        }
        if (std::fabs(velocity.vw) < odom_cfg::odometry.deadband.angular_radps)
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
        heading.is_online(odom_cfg::odometry.timeouts.heading_ms) && std::isfinite(heading.yaw_rad);
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
            if (std::fabs(delta) > odom_cfg::odometry.limits.max_heading_jump_rad)
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
        was_ready && in.dtMs >= odom_cfg::odometry.update.min_dt_ms && in.dtMs <= odom_cfg::odometry.update.max_dt_ms;
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

/**
 * @brief 清零机体速度并发布停止状态，保留累计位姿。
 */
void ChassisOdom::Stop()
{
    state_.vx_body_mps = 0.0f;
    state_.vy_body_mps = 0.0f;
    state_.wz_radps = 0.0f;
    publish(Sentry::Time::now_ms(), 0U, 0U);
}

/**
 * @brief 采集有效轮速和积分间隔，并刷新云台运动锁状态。
 * @return 当前时间、积分间隔、有效轮数及方向校正后的轮速。
 */
ChassisOdom::Inputs ChassisOdom::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    const auto gimbal_gate = sentry_data.motion_gate.read();
    const bool gate_fresh = gimbal_gate.is_online(chassis_cfg::chassis.timeouts.gimbal_motion_gate_ms);
    gimbal_motion_lock_ = (gate_fresh && gimbal_gate.locks_chassis()) ||
                          (gimbal_gate.timestamp != 0U && !gate_fresh &&
                           gimbal_gate.state != App::Chassis::GimbalMotionGateState::NORMAL);
    in.dtMs = in.nowMs - last_update_ms_;
    last_update_ms_ = in.nowMs;

    for (std::uint8_t i = 0U; i < WHEEL_COUNT; ++i)
    {
        const Sentry::MotorFeedback feedback = wheel_sub_[i]->read();
        if (feedback.is_online(odom_cfg::odometry.timeouts.motor_fb_ms) && std::isfinite(feedback.output_speed_rpm))
        {
            in.wheelRpm[i] = feedback.output_speed_rpm * App::Chassis::WHEEL_SIGN[i];
            ++in.validWheels;
        }
    }
    return in;
}

/**
 * @brief 将航向角折叠到一个圆周内。
 * @param value 待折叠的航向角，单位为 rad。
 * @return 范围为 [-π, π] 的航向角，单位为 rad。
 */
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

/**
 * @brief 根据进入和退出阈值更新静止锁，抑制静止时的速度噪声。
 * @param[in,out] velocity 运动学解算得到的机体速度，静止锁生效时将各分量清零。
 * @return 静止锁生效并将速度清零时返回 true。
 */
bool ChassisOdom::apply_stationary_gate(ALG::Omni::ChassisOdometry &velocity)
{
    const float planar = std::hypot(velocity.vx, velocity.vy);
    const float angular = std::fabs(velocity.vw);
    if (stationary_lock_)
    {
        if (planar > odom_cfg::odometry.stationary.exit_linear_mps ||
            angular > odom_cfg::odometry.stationary.exit_angular_radps)
        {
            stationary_lock_ = false;
        }
    }
    else if (planar < odom_cfg::odometry.stationary.enter_linear_mps &&
             angular < odom_cfg::odometry.stationary.enter_angular_radps)
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

/**
 * @brief 写入里程计时间戳、有效性标志和云台运动锁，并发布结果。
 * @param now 当前里程计更新的毫秒时间戳。
 * @param flags 里程计状态标志位。
 * @param valid_wheel_count 本周期参与有效性统计的车轮数量。
 */
void ChassisOdom::publish(std::uint32_t now, std::uint8_t flags, std::uint8_t valid_wheel_count)
{
    state_.timestamp = now;
    state_.flags = flags;
    state_.valid_wheel_count = valid_wheel_count;
    state_.gimbal_motion_lock = gimbal_motion_lock_ ? 1U : 0U;
    sentry_data.odometry.publish(state_);
}
