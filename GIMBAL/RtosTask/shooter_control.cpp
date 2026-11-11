#include "communication/robot_data.hpp"

#include "RtosTask/shooter_control.hpp"

#include "RtosTask/gimbal_control.hpp"

#include <cmath>
#include <algorithm>

namespace
{
constexpr std::uint8_t kDt7ChRightScroll = 6U;
constexpr std::uint8_t kDt7ChS1 = 4U;
constexpr std::uint8_t kDt7ChS2 = 7U;

float dial_output_angle_deg(const Sentry::MotorFeedback &feedback)
{
    return feedback.total_angle_deg / AppConfig::OnmiShooter::dial_external_reduction;
}

float dial_output_speed_rpm(const Sentry::MotorFeedback &feedback)
{
    return feedback.speed_rpm();
}
} // namespace

void ShooterCtrl::Update()
{
    // 汇总遥控器、视觉、裁判和电机反馈后，更新模式及拨盘/摩擦轮命令。

    const auto in = readInputs();

    const bool left_online = fb_online(in.left);
    const bool right_online = fb_online(in.right);
    const bool third_online = fb_online(in.third);
    const bool dial_online = fb_online(in.dial) && is_finite(dial_output_angle_deg(in.dial));
    if (dial_online && !last_dial_fb_online_)
    {

        sync_dial_ref(dial_output_angle_deg(in.dial));
        scroll_active_since_ms_ = 0U;
    }
    last_dial_fb_online_ = dial_online;
    const bool friction_brake_needed =
        remote_connected(in.remote) && ((left_online && !friction_wheel_stopped(in.left)) ||
                                        (right_online && !friction_wheel_stopped(in.right)) ||
                                        (third_online && !friction_wheel_stopped(in.third)));
    const bool output_gate = remote_allows_output(in.remote) || friction_brake_needed;

    const OnmiShooterMode next_mode =
        remote_allows_output(in.remote) ? select_mode(in.remote) : OnmiShooterMode::STOP;
    const float dial_angle_deg = dial_online ? dial_output_angle_deg(in.dial) : dial_ref_deg_;
    const bool friction_is_ready =
        remote_allows_output(in.remote) && friction_ready(in.left, in.right, in.third);
    if (next_mode != mode_)
    {
        mode_ = next_mode;
        sync_dial_ref(dial_angle_deg);
        scroll_active_since_ms_ = 0U;
        reset_vis_state();
        reset_pids();
    }

    if (!remote_allows_output(in.remote) || mode_ == OnmiShooterMode::STOP)
    {
        update_dial_recovery(in.dial, 0.0f, false, in.nowMs);
        reset_vis_state();
        heat_result_ = update_heat_control(false, 0.0f, in.left, in.right, in.referee, in.nowMs);
        if (friction_brake_needed)
        {
            publish_stop(in.left, in.right, in.third);
        }
        else
        {
            publish_zero_control();
        }

        return;
    }

    bool run_friction = true;
    bool allow_dial_fire = friction_is_ready && dial_online;
    if (mode_ == OnmiShooterMode::VISION)
    {

        run_friction = vis_control_active(in.vision);
        const auto state = update_vis_state(in.vision, friction_is_ready, in.nowMs);
        allow_dial_fire = allow_dial_fire && vis_dial_should_fire(state);
    }
    else
    {
        reset_vis_state();
    }

    allow_dial_fire = allow_dial_fire && update_base_fire_gate(in.nowMs);

    if (!run_friction)
    {
        update_dial_recovery(in.dial, 0.0f, false, in.nowMs);
        heat_result_ = update_heat_control(false, 0.0f, in.left, in.right, in.referee, in.nowMs);
        if (friction_brake_needed)
        {
            publish_stop(in.left, in.right, in.third);
        }
        else
        {
            publish_zero_control();
        }

        return;
    }

    const float requested_rate_hz = requested_fire_rate_hz(mode_);
    heat_result_ =
        update_heat_control(true, requested_rate_hz, in.left, in.right, in.referee, in.nowMs);
    float allowed_rate_hz = requested_rate_hz;
    if (ShooterRuntime::heat_limit_enable)
    {
        allow_dial_fire = allow_dial_fire && heat_result_.allow_shoot;
        allowed_rate_hz = heat_result_.allowed_fire_rate;
    }

    update_dial_ref(mode_, in.remote, dial_angle_deg, allow_dial_fire, allowed_rate_hz, in.nowMs);

    update_dial_recovery(in.dial, 0.0f, allow_dial_fire, in.nowMs);

    publish_control(true, dial_online, in.left, in.right, in.third, in.dial);
}

void ShooterCtrl::update_dial_ref(OnmiShooterMode mode, const Sentry::RemoteData &remote,
                                  float dial_angle_deg, bool allow_fire, float fire_rate_hz,
                                  std::uint32_t now_ms)
{
    if (!ref_synced_)
        sync_dial_ref(dial_angle_deg);
    const bool fire_active = right_scroll_active(remote);
    if (fire_active && !last_scroll_active_)
        scroll_active_since_ms_ = now_ms;
    else if (!fire_active)
        scroll_active_since_ms_ = 0U;

    switch (mode)
    {
    case OnmiShooterMode::CONTINUOUS:
        if (allow_fire && fire_active && scroll_active_since_ms_ != 0U &&
            (now_ms - scroll_active_since_ms_) >= ShooterRuntime::continuous_hold_threshold_ms)
        {
            dial_ref_deg_ -= ShooterRuntime::dial_direction * angle_per_tick_deg(fire_rate_hz);
        }
        else
        {
            sync_dial_ref(dial_angle_deg);
        }
        break;
    case OnmiShooterMode::VISION:

        if (allow_fire)
        {
            dial_ref_deg_ -= ShooterRuntime::dial_direction * angle_per_tick_deg(fire_rate_hz);
        }
        break;
    case OnmiShooterMode::STOP:
    case OnmiShooterMode::COUNT:
        sync_dial_ref(dial_angle_deg);
        break;
    }
    last_scroll_active_ = fire_active;
}

void ShooterCtrl::publish_control(bool enable, bool enable_dial,
                                  const Sentry::MotorFeedback &fric_left,
                                  const Sentry::MotorFeedback &fric_right,
                                  const Sentry::MotorFeedback &fric_third,
                                  const Sentry::MotorFeedback &dial)
{

    const bool left_enable = enable && fb_online(fric_left);
    const bool right_enable = enable && fb_online(fric_right);
    const bool third_enable = enable && fb_online(fric_third);
    const float left_current =
        left_enable ? fric_left_pid_.update(ShooterRuntime::fric_left_ref_rpm,
                                            fric_left.speed_rpm(), ShooterRuntime::dt_sec)
                    : 0.0f;
    const float right_current =
        right_enable ? fric_right_pid_.update(ShooterRuntime::fric_right_ref_rpm,
                                              fric_right.speed_rpm(), ShooterRuntime::dt_sec)
                     : 0.0f;
    const float third_current =
        third_enable ? fric_third_pid_.update(ShooterRuntime::fric_third_ref_rpm,
                                              fric_third.speed_rpm(), ShooterRuntime::dt_sec)
                     : 0.0f;
    if (!left_enable)
        fric_left_pid_.reset();
    if (!right_enable)
        fric_right_pid_.reset();
    if (!third_enable)
        fric_third_pid_.reset();

    float dial_current = 0.0f;
    float dial_ref_speed = 0.0f;
    if (enable_dial && dial_recovery_state_ == OnmiDialRecoveryState::NORMAL)
    {
        dial_ref_speed = dial_angle_pid_.update(dial_ref_deg_, dial_output_angle_deg(dial),
                                                ShooterRuntime::dt_sec);
        dial_current = dial_speed_pid_.update(dial_ref_speed, dial_output_speed_rpm(dial),
                                              ShooterRuntime::dt_sec);
    }
    else
    {
        dial_angle_pid_.reset();
        dial_speed_pid_.reset();
    }

    const auto left_output = clamp_fric_current(left_current);
    const auto right_output = clamp_fric_current(right_current);
    const auto third_output = clamp_fric_current(third_current);

    sentry_data.fric_left_cmd.publish(Sentry::MotorCommand{left_output, left_enable});
    sentry_data.fric_right_cmd.publish(Sentry::MotorCommand{right_output, right_enable});
    sentry_data.fric_third_cmd.publish(Sentry::MotorCommand{third_output, third_enable});

    if (dial_recovery_state_ == OnmiDialRecoveryState::REVERSE)
    {
        publish_dial_override(static_cast<std::int16_t>(dial_normal_direction_ *
                                                        ShooterRuntime::dial_reverse_current),
                              true);
    }
    else if (dial_recovery_state_ == OnmiDialRecoveryState::REINIT)
    {
        publish_dial_override(0, false, true);
        dial_recovery_state_ = OnmiDialRecoveryState::WAIT_FEEDBACK;
        dial_recovery_state_since_ms_ = Sentry::Time::now_ms();
        dial_last_fb_ms_ = dial.timestamp;
    }
    else if (dial_recovery_state_ == OnmiDialRecoveryState::SETTLE ||
             dial_recovery_state_ == OnmiDialRecoveryState::WAIT_FEEDBACK ||
             dial_recovery_state_ == OnmiDialRecoveryState::BACKOFF)
    {
        publish_dial_override(0, false);
    }
    else
    {
        const auto dial_output =
            enable_dial ? clamp_dial_current(dial_current) : static_cast<std::int16_t>(0);
        const bool dial_enable = enable && enable_dial;

        sentry_data.dial_cmd.publish(Sentry::MotorCommand{dial_output, dial_enable});
    }
}

void ShooterCtrl::Stop()
{
    mode_ = OnmiShooterMode::STOP;
    ref_synced_ = false;
    last_dial_fb_online_ = false;
    scroll_active_since_ms_ = 0U;
    reset_vis_state();
    base_fire_gate_open_ = false;
    base_fire_gate_clear_since_ms_ = 0U;
    dial_recovery_state_ = OnmiDialRecoveryState::NORMAL;
    dial_stall_since_ms_ = 0U;
    dial_stall_start_angle_deg_ = 0.0f;
    dial_recovery_retry_count_ = 0U;
    publish_zero_control();
}

bool ShooterCtrl::is_finite(float value)
{
    return std::isfinite(value);
}

std::uint8_t ShooterCtrl::dt7_switch(const Sentry::RemoteData &remote, std::uint8_t index)
{
    if (remote.timestamp == 0U)
        return 0U;
    if (index == kDt7ChS1)
        return remote.s1;
    if (index == kDt7ChS2)
        return remote.s2;
    return 0U;
}

bool ShooterCtrl::remote_connected(const Sentry::RemoteData &remote)
{
    return remote.timestamp != 0U &&
           Sentry::Time::fresh(remote.timestamp, ShooterRuntime::remote_connection_timeout_ms);
}

bool ShooterCtrl::remote_allows_output(const Sentry::RemoteData &remote)
{
    return remote_connected(remote) && !(dt7_switch(remote, kDt7ChS1) == ShooterRuntime::stop_s1 &&
                                         dt7_switch(remote, kDt7ChS2) == ShooterRuntime::stop_s2);
}

OnmiShooterMode ShooterCtrl::select_mode(const Sentry::RemoteData &remote)
{
    const std::uint8_t s1 = dt7_switch(remote, kDt7ChS1);
    const std::uint8_t s2 = dt7_switch(remote, kDt7ChS2);
    if (s1 == ShooterRuntime::continuous_s1 && s2 == ShooterRuntime::continuous_s2)
    {
        return OnmiShooterMode::CONTINUOUS;
    }
    if (s1 == ShooterRuntime::vis_s1 && s2 == ShooterRuntime::vis_s2)
    {
        return OnmiShooterMode::VISION;
    }
    return OnmiShooterMode::STOP;
}

bool ShooterCtrl::right_scroll_active(const Sentry::RemoteData &remote) const
{
    const float threshold =
        ShooterRuntime::scroll_deadband *
        (last_scroll_active_ ? AppConfig::OnmiShooter::scroll_release_threshold_scale : 1.0f);
    return std::fabs(remote.scroll_norm) > threshold;
}

float ShooterCtrl::angle_per_tick_deg(float fire_rate_hz)
{
    return fire_rate_hz * (360.0f / ShooterRuntime::dial_gear_count) * ShooterRuntime::dt_sec *
           ShooterRuntime::dial_reduction_ratio;
}

bool ShooterCtrl::fb_online(const Sentry::MotorFeedback &feedback)
{
    return feedback.is_online(ShooterRuntime::motor_fb_timeout_ms);
}

bool ShooterCtrl::friction_wheel_stopped(const Sentry::MotorFeedback &feedback)
{
    return std::fabs(feedback.speed_rpm()) <= ShooterRuntime::fric_stop_error_rpm;
}

bool ShooterCtrl::friction_ready(const Sentry::MotorFeedback &left,
                                 const Sentry::MotorFeedback &right,
                                 const Sentry::MotorFeedback &third)
{
    const auto speed_ready = [](float ref_rpm, float fb_rpm)
    {
        const float speed_in_ref_direction = ref_rpm < 0.0f ? -fb_rpm : fb_rpm;
        return std::isfinite(fb_rpm) &&
               speed_in_ref_direction >= std::fabs(ref_rpm) - ShooterRuntime::fric_ready_error_rpm;
    };
    return fb_online(left) && fb_online(right) && fb_online(third) &&
           speed_ready(ShooterRuntime::fric_left_ref_rpm, left.speed_rpm()) &&
           speed_ready(ShooterRuntime::fric_right_ref_rpm, right.speed_rpm()) &&
           speed_ready(ShooterRuntime::fric_third_ref_rpm, third.speed_rpm());
}

std::int16_t ShooterCtrl::clamp_fric_current(float value)
{
    return static_cast<std::int16_t>(Sentry::Utils::clamp(value, -10000.0f, 10000.0f));
}

std::int16_t ShooterCtrl::clamp_dial_current(float value)
{
    return static_cast<std::int16_t>(Sentry::Utils::clamp(value, -2048.0f, 2048.0f));
}

float ShooterCtrl::requested_fire_rate_hz(OnmiShooterMode mode)
{
    switch (mode)
    {
    case OnmiShooterMode::CONTINUOUS:
        return ShooterRuntime::manual_fire_rate_hz;
    case OnmiShooterMode::VISION:
        return ShooterRuntime::auto_fire_rate_hz;
    case OnmiShooterMode::STOP:
    case OnmiShooterMode::COUNT:
        break;
    }
    return 0.0f;
}

float ShooterCtrl::commanded_friction_speed_rpm(bool run_friction)
{
    return run_friction ? 0.5f * (std::fabs(ShooterRuntime::fric_left_ref_rpm) +
                                  std::fabs(ShooterRuntime::fric_right_ref_rpm))
                        : 0.0f;
}

float ShooterCtrl::referee_shooter_heat(const Sentry::RefereeData &referee)
{
    return static_cast<float>(referee.power.shooter_17mm_barrel_heat);
}

bool ShooterCtrl::vis_control_active(const App::Vision::VisionControlRequest &vision)
{
    return vision.is_online(ShooterRuntime::vis_request_timeout_ms) && vision.has_ref() &&
           is_finite(vision.yaw_angle_rad) && is_finite(vision.yaw_speed_radps) &&
           is_finite(vision.yaw_accel_radps2) && is_finite(vision.pitch_angle_rad) &&
           is_finite(vision.pitch_speed_radps) && is_finite(vision.pitch_accel_radps2);
}

bool ShooterCtrl::vis_lock_detected(const App::Vision::VisionControlRequest &vision)
{
    return ShooterRuntime::vis_request_timeout_ms != 0U && ShooterRuntime::enable_vis_fire &&
           vis_control_active(vision);
}

bool ShooterCtrl::vis_fire_requested(const App::Vision::VisionControlRequest &vision)
{
    return vis_lock_detected(vision) && vision.should_fire();
}

bool ShooterCtrl::vis_dial_should_fire(OnmiVisionShooterState state)
{
    return state == OnmiVisionShooterState::FIRE;
}

bool ShooterCtrl::update_base_fire_gate(std::uint32_t now_ms)
{
    const auto gimbal = App::Onmi::gimbalMotionStatus.read();
    const bool status_fresh =
        gimbal.timestamp != 0U &&
        Sentry::Time::fresh(gimbal.timestamp, ShooterRuntime::base_fire_status_timeout_ms) &&
        std::isfinite(gimbal.bigYawSpeed) && std::isfinite(gimbal.bigYawTargetSpeed);
    if (!status_fresh)
    {

        base_fire_gate_open_ = false;
        base_fire_gate_clear_since_ms_ = 0U;
        return false;
    }

    const float motion_speed_radps =
        std::fmax(std::fabs(gimbal.bigYawSpeed), std::fabs(gimbal.bigYawTargetSpeed));
    if (motion_speed_radps >= ShooterRuntime::base_fire_block_speed_radps)
    {
        base_fire_gate_open_ = false;
        base_fire_gate_clear_since_ms_ = 0U;
        return false;
    }

    if (motion_speed_radps > ShooterRuntime::base_fire_release_speed_radps)
    {

        base_fire_gate_clear_since_ms_ = 0U;
        return base_fire_gate_open_;
    }

    if (base_fire_gate_open_)
    {
        return true;
    }

    if (base_fire_gate_clear_since_ms_ == 0U)
    {
        base_fire_gate_clear_since_ms_ = now_ms;
    }
    if (now_ms - base_fire_gate_clear_since_ms_ >= ShooterRuntime::base_fire_release_hold_ms)
    {
        base_fire_gate_open_ = true;
    }
    return base_fire_gate_open_;
}

OnmiVisionShooterState ShooterCtrl::update_vis_state(
    const App::Vision::VisionControlRequest &vision, bool friction_is_ready, std::uint32_t now_ms)
{
    if (vis_lock_detected(vision))
        last_vis_lock_ms_ = now_ms;
    const bool lock_recent =
        last_vis_lock_ms_ != 0U && (now_ms - last_vis_lock_ms_) <= ShooterRuntime::vis_lock_hold_ms;
    OnmiVisionShooterState next = OnmiVisionShooterState::IDLE;
    if (lock_recent)
    {
        if (!friction_is_ready)
            next = OnmiVisionShooterState::WAIT_READY;
        else if (vis_fire_requested(vision))
            next = OnmiVisionShooterState::FIRE;
        else
            next = OnmiVisionShooterState::HOLD;
    }
    if (next == OnmiVisionShooterState::IDLE)
        last_vis_lock_ms_ = 0U;
    vis_state_ = next;
    return vis_state_;
}

void ShooterCtrl::reset_vis_state()
{
    vis_state_ = OnmiVisionShooterState::IDLE;
    last_vis_lock_ms_ = 0U;
}

ALG::Omni::ShooterHeatLimiter::Result ShooterCtrl::update_heat_control(
    bool run_friction, float requested_fire_rate_hz, const Sentry::MotorFeedback &fric_left,
    const Sentry::MotorFeedback &fric_right, const Sentry::RefereeData &referee,
    std::uint32_t now_ms)
{
    if (!ShooterRuntime::heat_limit_enable)
    {
        ALG::Omni::ShooterHeatLimiter::Result result{};
        result.allow_shoot = true;
        result.heat_limit = ShooterRuntime::heat_limit;
        result.heat_margin = ShooterRuntime::heat_limit;
        result.allowed_fire_rate = requested_fire_rate_hz;
        return result;
    }
    ALG::Omni::ShooterHeatLimiter::Input input{};
    input.now_ms = now_ms;
    input.dt_s = ShooterRuntime::dt_sec;
    input.ref_speed = commanded_friction_speed_rpm(run_friction);
    input.feedback.left_current = fric_left.current_A();
    input.feedback.right_current = fric_right.current_A();
    input.feedback.left_speed = fric_left.speed_rpm();
    input.feedback.right_speed = fric_right.speed_rpm();
    input.heat_limit = ShooterRuntime::heat_limit;
    input.cooling_rate = ShooterRuntime::heat_cooling_rate;
    input.ref_fire_rate = requested_fire_rate_hz;
    const bool referee_valid = ShooterRuntime::heat_use_referee &&
                               referee.online(ShooterRuntime::heat_referee_timeout_ms) &&
                               referee.robot_status_count != 0U && referee.power_heat_count != 0U &&
                               referee.status.shooter_barrel_heat_limit > 0U;
    if (referee_valid)
    {
        input.use_external_heat = true;
        input.external_heat = referee_shooter_heat(referee);
        input.heat_limit = static_cast<float>(referee.status.shooter_barrel_heat_limit);
        input.cooling_rate = static_cast<float>(referee.status.shooter_barrel_cooling_value);
        heat_limiter_.set_estimated_heat(input.external_heat);
    }
    return heat_limiter_.update(input);
}

void ShooterCtrl::sync_dial_ref(float dial_angle_deg)
{
    if (!is_finite(dial_angle_deg))
        return;
    dial_ref_deg_ = dial_angle_deg;
    ref_synced_ = true;
    last_scroll_active_ = false;
}

void ShooterCtrl::publish_dial_override(std::int16_t current, bool enable, bool reinitialize)
{

    sentry_data.dial_cmd.publish(Sentry::MotorCommand{current, enable, reinitialize});
}

bool ShooterCtrl::update_dial_recovery(const Sentry::MotorFeedback &dial, float,
                                       bool motion_requested, std::uint32_t now_ms)
{
    if (!AppConfig::OnmiShooter::dial_stall_recovery_enable)
    {

        dial_stall_since_ms_ = 0U;
        dial_stall_start_angle_deg_ = dial_output_angle_deg(dial);
        dial_recovery_state_ = OnmiDialRecoveryState::NORMAL;
        return false;
    }

    const float dial_angle_deg = dial_output_angle_deg(dial);
    const bool online = fb_online(dial) && is_finite(dial_angle_deg);
    const float error = dial_ref_deg_ - dial_angle_deg;
    switch (dial_recovery_state_)
    {
    case OnmiDialRecoveryState::NORMAL:
        if (!motion_requested || !online)
        {
            dial_stall_since_ms_ = 0U;
            return false;
        }
        if (std::fabs(error) > ShooterRuntime::dial_stall_error_deg &&
            std::fabs(dial_output_speed_rpm(dial)) < ShooterRuntime::dial_stall_speed_rpm &&
            std::fabs(dial.current_A()) >= ShooterRuntime::dial_stall_current_a)
        {
            if (dial_stall_since_ms_ == 0U)
            {
                dial_stall_since_ms_ = now_ms;
                dial_stall_start_angle_deg_ = dial_angle_deg;
            }

            const float commanded_direction = error < 0.0f ? -1.0f : 1.0f;
            const float forward_progress =
                commanded_direction * (dial_angle_deg - dial_stall_start_angle_deg_);
            if (forward_progress >= AppConfig::OnmiShooter::dial_stall_progress_deg)
            {
                dial_stall_since_ms_ = now_ms;
                dial_stall_start_angle_deg_ = dial_angle_deg;
            }
            if ((now_ms - dial_stall_since_ms_) >= ShooterRuntime::dial_stall_hold_ms)
            {
                dial_normal_direction_ = (ShooterRuntime::dial_direction >= 0.0f) ? 1 : -1;
                dial_recovery_state_ = OnmiDialRecoveryState::REVERSE;
                dial_recovery_state_since_ms_ = now_ms;
                dial_recovery_retry_count_ = 0U;
                dial_stall_since_ms_ = 0U;
                dial_stall_start_angle_deg_ = dial_angle_deg;
                dial_angle_pid_.reset();
                dial_speed_pid_.reset();
            }
        }
        else
        {
            dial_stall_since_ms_ = 0U;
            dial_stall_start_angle_deg_ = dial_angle_deg;
        }
        break;
    case OnmiDialRecoveryState::REVERSE:
        if ((now_ms - dial_recovery_state_since_ms_) >= ShooterRuntime::dial_reverse_time_ms)
        {
            dial_recovery_state_ = OnmiDialRecoveryState::SETTLE;
            dial_recovery_state_since_ms_ = now_ms;
        }
        break;
    case OnmiDialRecoveryState::SETTLE:
        if ((now_ms - dial_recovery_state_since_ms_) >= ShooterRuntime::dial_settle_time_ms)
        {
            dial_recovery_state_ = OnmiDialRecoveryState::REINIT;
            dial_recovery_state_since_ms_ = now_ms;
        }
        break;
    case OnmiDialRecoveryState::REINIT:
        break;
    case OnmiDialRecoveryState::WAIT_FEEDBACK:
        if (online && dial.timestamp != dial_last_fb_ms_)
        {
            sync_dial_ref(dial_angle_deg);
            dial_recovery_state_ = OnmiDialRecoveryState::NORMAL;
            dial_recovery_state_since_ms_ = now_ms;
            dial_stall_since_ms_ = 0U;
            dial_stall_start_angle_deg_ = dial_angle_deg;
            dial_recovery_retry_count_ = 0U;
        }
        else if ((now_ms - dial_recovery_state_since_ms_) >= ShooterRuntime::dial_reinit_timeout_ms)
        {
            ++dial_recovery_retry_count_;
            dial_recovery_state_ = OnmiDialRecoveryState::BACKOFF;
            dial_recovery_state_since_ms_ = now_ms;
        }
        break;
    case OnmiDialRecoveryState::BACKOFF:
        if ((now_ms - dial_recovery_state_since_ms_) >= ShooterRuntime::dial_retry_interval_ms)
        {
            dial_recovery_state_ = OnmiDialRecoveryState::REVERSE;
            dial_recovery_state_since_ms_ = now_ms;
            if (dial_recovery_retry_count_ >= ShooterRuntime::dial_recovery_max_retry)
            {
                dial_recovery_retry_count_ = 0U;
            }
        }
        break;
    }
    return dial_recovery_state_ != OnmiDialRecoveryState::NORMAL;
}

void ShooterCtrl::publish_stop(const Sentry::MotorFeedback &fric_left,
                               const Sentry::MotorFeedback &fric_right,
                               const Sentry::MotorFeedback &fric_third)
{
    const bool left_brake = fb_online(fric_left) && !friction_wheel_stopped(fric_left);
    const bool right_brake = fb_online(fric_right) && !friction_wheel_stopped(fric_right);
    const bool third_brake = fb_online(fric_third) && !friction_wheel_stopped(fric_third);
    const float left_current =
        left_brake ? fric_left_pid_.update(0.0f, fric_left.speed_rpm(), ShooterRuntime::dt_sec)
                   : 0.0f;
    const float right_current =
        right_brake ? fric_right_pid_.update(0.0f, fric_right.speed_rpm(), ShooterRuntime::dt_sec)
                    : 0.0f;
    const float third_current =
        third_brake ? fric_third_pid_.update(0.0f, fric_third.speed_rpm(), ShooterRuntime::dt_sec)
                    : 0.0f;
    if (!left_brake)
        fric_left_pid_.reset();
    if (!right_brake)
        fric_right_pid_.reset();
    if (!third_brake)
        fric_third_pid_.reset();
    dial_angle_pid_.reset();
    dial_speed_pid_.reset();

    const auto left_output = clamp_fric_current(left_current);
    const auto right_output = clamp_fric_current(right_current);
    const auto third_output = clamp_fric_current(third_current);

    sentry_data.fric_left_cmd.publish(Sentry::MotorCommand{left_output, left_brake});
    sentry_data.fric_right_cmd.publish(Sentry::MotorCommand{right_output, right_brake});
    sentry_data.fric_third_cmd.publish(Sentry::MotorCommand{third_output, third_brake});
    if (dial_recovery_state_ == OnmiDialRecoveryState::REVERSE)
    {
        publish_dial_override(static_cast<std::int16_t>(dial_normal_direction_ *
                                                        ShooterRuntime::dial_reverse_current),
                              true);
    }
    else if (dial_recovery_state_ == OnmiDialRecoveryState::REINIT)
    {
        publish_dial_override(0, false, true);
        dial_recovery_state_ = OnmiDialRecoveryState::WAIT_FEEDBACK;
        dial_recovery_state_since_ms_ = Sentry::Time::now_ms();
    }
    else
    {
        publish_dial_override(0, false);
    }
}

void ShooterCtrl::publish_zero_control()
{
    reset_pids();

    sentry_data.fric_left_cmd.publish(Sentry::MotorCommand{0, false});
    sentry_data.fric_right_cmd.publish(Sentry::MotorCommand{0, false});
    sentry_data.fric_third_cmd.publish(Sentry::MotorCommand{0, false});
    if (dial_recovery_state_ == OnmiDialRecoveryState::REVERSE)
    {
        publish_dial_override(static_cast<std::int16_t>(dial_normal_direction_ *
                                                        ShooterRuntime::dial_reverse_current),
                              true);
    }
    else if (dial_recovery_state_ == OnmiDialRecoveryState::REINIT)
    {
        publish_dial_override(0, false, true);
        dial_recovery_state_ = OnmiDialRecoveryState::WAIT_FEEDBACK;
        dial_recovery_state_since_ms_ = Sentry::Time::now_ms();
    }
    else
    {
        publish_dial_override(0, false);
    }
}

void ShooterCtrl::reset_pids()
{
    fric_left_pid_.reset();
    fric_right_pid_.reset();
    fric_third_pid_.reset();
    dial_angle_pid_.reset();
    dial_speed_pid_.reset();
}

ShooterCtrl::Inputs ShooterCtrl::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.vision = sentry_data.vision.read();
    in.referee = sentry_data.referee.read();
    in.left = sentry_data.fric_left_fb.read();
    in.right = sentry_data.fric_right_fb.read();
    in.third = sentry_data.fric_third_fb.read();
    in.dial = sentry_data.dial_fb.read();

    return in;
}

ShooterCtrl::ShooterCtrl() {}

bool ShooterCtrl::Init()
{

    fric_left_pid_.set_config(ALG::Omni::TimePid::Config{
        ShooterRuntime::fric_pid_kp,
        ShooterRuntime::fric_pid_ki,
        ShooterRuntime::fric_pid_kd,
        ShooterRuntime::fric_pid_output_max,
        ShooterRuntime::fric_pid_output_min,
        ShooterRuntime::fric_pid_integral_max,
    });
    fric_right_pid_.set_config(fric_left_pid_.get_config());
    fric_third_pid_.set_config(fric_left_pid_.get_config());
    dial_angle_pid_.set_config(ALG::Omni::TimePid::Config{
        ShooterRuntime::dial_angle_pid_kp,
        ShooterRuntime::dial_angle_pid_ki,
        ShooterRuntime::dial_angle_pid_kd,
        ShooterRuntime::dial_angle_pid_output_max,
        ShooterRuntime::dial_angle_pid_output_min,
        ShooterRuntime::dial_angle_pid_integral_max,
    });
    dial_speed_pid_.set_config(ALG::Omni::TimePid::Config{
        ShooterRuntime::dial_speed_pid_kp,
        ShooterRuntime::dial_speed_pid_ki,
        ShooterRuntime::dial_speed_pid_kd,
        ShooterRuntime::dial_speed_pid_output_max,
        ShooterRuntime::dial_speed_pid_output_min,
        ShooterRuntime::dial_speed_pid_integral_max,
    });
    reset_pids();
    ALG::Omni::ShooterHeatLimiter::Config heat_config{};
    heat_config.window_size = ShooterRuntime::heat_window_size;
    heat_config.shot_cooldown_ms = ShooterRuntime::heat_shot_cooldown_ms;
    heat_config.startup_ignore_ms = ShooterRuntime::heat_startup_ignore_ms;
    heat_config.integral_threshold = ShooterRuntime::heat_integral_threshold;
    heat_config.noise_threshold = ShooterRuntime::heat_noise_threshold;
    heat_config.ref_start_threshold = ShooterRuntime::heat_ref_start_threshold;
    heat_config.wheel_ready_speed = ShooterRuntime::heat_wheel_ready_speed;
    heat_config.heat_limit = ShooterRuntime::heat_limit;
    heat_config.cooling_rate = ShooterRuntime::heat_cooling_rate;
    heat_config.heat_per_shot = ShooterRuntime::heat_per_shot;
    heat_config.ref_fire_rate = ShooterRuntime::manual_fire_rate_hz;
    heat_config.free_margin_ratio = ShooterRuntime::heat_free_margin_ratio;
    heat_config.stop_margin_ratio = ShooterRuntime::heat_stop_margin_ratio;
    heat_limiter_.set_config(heat_config);
    heat_limiter_.reset();
    return true;
}

unsigned ShooterCtrl::GetVofaData(float *out, bool friction) const
{
    if (friction)
    {
        const auto l = sentry_data.fric_left_fb.read(), r = sentry_data.fric_right_fb.read(),
                   t = sentry_data.fric_third_fb.read();
        const float v[] = {ShooterRuntime::fric_left_ref_rpm,
                           l.speed_rpm(),
                           ShooterRuntime::fric_right_ref_rpm,
                           r.speed_rpm(),
                           ShooterRuntime::fric_third_ref_rpm,
                           t.speed_rpm(),
                           float(sentry_data.fric_left_cmd.read().current),
                           float(sentry_data.fric_right_cmd.read().current),
                           float(sentry_data.fric_third_cmd.read().current)};
        std::copy(v, v + 9, out);
        return 9;
    }
    const auto dial = sentry_data.dial_fb.read();
    const float v[] = {dial_ref_deg_,
                       dial_output_angle_deg(dial),
                       dial_speed_pid_.get_ref(),
                       dial.speed_rpm(),
                       float(sentry_data.dial_cmd.read().current),
                       float(fb_online(dial)),
                       float(mode_),
                       float(heat_result_.allow_shoot)};
    std::copy(v, v + 8, out);
    return 8;
}
