#pragma once

#include "user/core/Alg/Omni/dual_yaw_control.hpp"

#include <cmath>

namespace ALG::Omni
{

/** @brief 根据视觉角误差和趋势，为大 yaw 输出保持角度及速度参考。 */
class VisionBigYawAngleHold
{
  public:
    struct Config
    {
        float kp_radps_per_rad{2.0f};
        float correction_limit_radps{0.8f};
        float correction_accel_limit_radps2{12.0f};
        float motion_trend_cutoff_hz{0.6f};
        float motion_speed_start_radps{0.08f};
        float motion_speed_full_radps{0.40f};
        float motion_rise_time_sec{0.03f};
        float motion_release_time_sec{0.20f};
        float anchor_follow_time_sec{0.02f};
        float center_ref_filter_time_sec{0.50f};
        float ki_radps_per_rad_sec{0.25f};
        float integral_speed_limit_radps{0.25f};
        float integral_release_time_sec{0.20f};
        float integral_speed_start_radps{0.03f};
        float integral_speed_full_radps{0.12f};
    };

    struct Output
    {
        float planned_speed_radps{0.0f};
        float reference_speed_radps{0.0f};
        float correction_speed_radps{0.0f};
        float anchor_angle_rad{0.0f};
        float angle_error_rad{0.0f};
        float motion_level{0.0f};
        float trend_speed_radps{0.0f};
        float center_ref_angle_rad{0.0f};
        float integral_speed_radps{0.0f};
    };

    VisionBigYawAngleHold() = default;
    explicit VisionBigYawAngleHold(const Config &config)
        : config_(config), trend_filter_(config.motion_trend_cutoff_hz)
    {
    }

    void configure(const Config &config)
    {
        config_ = config;
        trend_filter_.configure(config.motion_trend_cutoff_hz);
    }

    bool set_position_kp(float kp_radps_per_rad)
    {
        if (!std::isfinite(kp_radps_per_rad) || kp_radps_per_rad < 0.0f || kp_radps_per_rad > 20.0f)
        {
            return false;
        }
        config_.kp_radps_per_rad = kp_radps_per_rad;
        return true;
    }

    void reset(float measured_angle_rad)
    {
        anchor_angle_rad_ = DualYawDetail::wrap_angle_rad(measured_angle_rad);
        center_ref_angle_rad_ = anchor_angle_rad_;
        trend_filter_.reset(0.0f);
        motion_level_ = 0.0f;
        correction_speed_radps_ = 0.0f;
        integral_speed_radps_ = 0.0f;
        ready_ = std::isfinite(measured_angle_rad);
        output_ = {};
        output_.anchor_angle_rad = anchor_angle_rad_;
        output_.center_ref_angle_rad = center_ref_angle_rad_;
    }

    void clear()
    {
        ready_ = false;
        trend_filter_.clear();
        anchor_angle_rad_ = 0.0f;
        center_ref_angle_rad_ = 0.0f;
        motion_level_ = 0.0f;
        correction_speed_radps_ = 0.0f;
        integral_speed_radps_ = 0.0f;
        output_ = {};
    }

    Output update(float measured_angle_rad, float measured_speed_radps,
                  float requested_center_angle_rad, float planned_speed_radps, float q_limit_weight,
                  float speed_limit_radps, float dt_sec)
    {
        if (!std::isfinite(measured_angle_rad) || !std::isfinite(measured_speed_radps) ||
            !std::isfinite(requested_center_angle_rad) || !std::isfinite(planned_speed_radps) ||
            !std::isfinite(q_limit_weight) || !std::isfinite(speed_limit_radps) ||
            !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            reset(measured_angle_rad);
            return output_;
        }
        if (!ready_)
        {
            reset(measured_angle_rad);
        }

        center_ref_angle_rad_ = DualYawDetail::wrap_angle_rad(
            center_ref_angle_rad_ +
            time_alpha(config_.center_ref_filter_time_sec, dt_sec) *
                DualYawDetail::wrap_angle_rad(requested_center_angle_rad - center_ref_angle_rad_));

        const float speed_limit = std::fmax(speed_limit_radps, 0.0f);
        const float planned_speed =
            DualYawDetail::clamp(planned_speed_radps, -speed_limit, speed_limit);
        const float trend_speed = trend_filter_.update(planned_speed, dt_sec);
        const float move_from_trend = speed_weight(std::fabs(trend_speed));
        const float move_from_limit = DualYawDetail::clamp(q_limit_weight, 0.0f, 1.0f) *
                                      speed_weight(std::fabs(planned_speed));
        const float motion_ref = std::fmax(move_from_trend, move_from_limit);

        const float switch_time = motion_ref > motion_level_ ? config_.motion_rise_time_sec
                                                             : config_.motion_release_time_sec;
        motion_level_ += time_alpha(switch_time, dt_sec) * (motion_ref - motion_level_);
        motion_level_ = DualYawDetail::clamp(motion_level_, 0.0f, 1.0f);

        const float anchor_goal = DualYawDetail::wrap_angle_rad(
            measured_angle_rad +
            (1.0f - motion_level_) *
                DualYawDetail::wrap_angle_rad(center_ref_angle_rad_ - measured_angle_rad));
        const float anchor_error = DualYawDetail::wrap_angle_rad(anchor_goal - anchor_angle_rad_);
        anchor_angle_rad_ = DualYawDetail::wrap_angle_rad(
            anchor_angle_rad_ + time_alpha(config_.anchor_follow_time_sec, dt_sec) * anchor_error);

        const float angle_error =
            DualYawDetail::wrap_angle_rad(anchor_angle_rad_ - measured_angle_rad);
        const float correction_limit = std::fmax(config_.correction_limit_radps, 0.0f);
        const float integral_limit =
            std::fmin(std::fmax(config_.integral_speed_limit_radps, 0.0f), correction_limit);
        const float kp = std::fmax(config_.kp_radps_per_rad, 0.0f);
        const float active_error = (1.0f - motion_level_) * angle_error;
        const float unsaturated_correction = kp * active_error + integral_speed_radps_;

        if (motion_level_ > 0.0f)
        {
            integral_speed_radps_ *=
                1.0f - motion_level_ * time_alpha(config_.integral_release_time_sec, dt_sec);
        }
        if (std::fabs(unsaturated_correction) < correction_limit ||
            active_error * unsaturated_correction < 0.0f)
        {
            const float integral_speed_gate =
                1.0f - smooth_weight(std::fabs(measured_speed_radps),
                                     config_.integral_speed_start_radps,
                                     config_.integral_speed_full_radps);
            integral_speed_radps_ += std::fmax(config_.ki_radps_per_rad_sec, 0.0f) * active_error *
                                     integral_speed_gate * dt_sec;
        }
        integral_speed_radps_ =
            DualYawDetail::clamp(integral_speed_radps_, -integral_limit, integral_limit);
        const float correction_ref = DualYawDetail::clamp(kp * active_error + integral_speed_radps_,
                                                          -correction_limit, correction_limit);
        const float max_correction_delta =
            std::fmax(config_.correction_accel_limit_radps2, 0.0f) * dt_sec;
        if (max_correction_delta > 0.0f)
        {
            correction_speed_radps_ +=
                DualYawDetail::clamp(correction_ref - correction_speed_radps_,
                                     -max_correction_delta, max_correction_delta);
        }
        else
        {
            correction_speed_radps_ = correction_ref;
        }

        output_.planned_speed_radps = planned_speed;
        output_.reference_speed_radps = DualYawDetail::clamp(
            planned_speed + correction_speed_radps_, -speed_limit, speed_limit);
        output_.correction_speed_radps = correction_speed_radps_;
        output_.anchor_angle_rad = anchor_angle_rad_;
        output_.angle_error_rad = angle_error;
        output_.motion_level = motion_level_;
        output_.trend_speed_radps = trend_speed;
        output_.center_ref_angle_rad = center_ref_angle_rad_;
        output_.integral_speed_radps = integral_speed_radps_;
        return output_;
    }

    const Output &output() const { return output_; }
    bool initialized() const { return ready_; }
    float position_kp() const { return config_.kp_radps_per_rad; }

  private:
    static float time_alpha(float time_sec, float dt_sec)
    {
        if (!std::isfinite(time_sec) || time_sec <= 0.0f)
        {
            return 1.0f;
        }
        return dt_sec / (time_sec + dt_sec);
    }

    float speed_weight(float abs_speed_radps) const
    {
        return smooth_weight(abs_speed_radps, config_.motion_speed_start_radps,
                             config_.motion_speed_full_radps);
    }

    static float smooth_weight(float abs_speed_radps, float start_radps, float full_radps)
    {
        const float start = std::fmax(start_radps, 0.0f);
        const float full = std::fmax(full_radps, start + 1.0e-4f);
        const float x =
            DualYawDetail::clamp((abs_speed_radps - start) / (full - start), 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }

    Config config_{};
    CutoffLowPassFilter trend_filter_{0.6f};
    float anchor_angle_rad_{0.0f};
    float center_ref_angle_rad_{0.0f};
    float motion_level_{0.0f};
    float correction_speed_radps_{0.0f};
    float integral_speed_radps_{0.0f};
    bool ready_{false};
    Output output_{};
};

} // namespace ALG::Omni
