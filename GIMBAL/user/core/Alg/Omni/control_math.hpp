#pragma once
#include "angle_math.hpp"
#include "time_pid.hpp"
#include <cmath>
#include <cstdint>
namespace Sentry::Utils
{
namespace Constants
{
// 常量集中标注角度、转速、编码器比例及默认反馈超时单位。
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float G = 9.80665f;

constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;

constexpr float RPM_TO_RAD_PER_SEC = TWO_PI / 60.0f;
constexpr float RAD_PER_SEC_TO_RPM = 60.0f / TWO_PI;

constexpr uint16_t DJI_ENCODER_MAX = 8192;
constexpr uint16_t DJI_ENCODER_HALF = 4096;

constexpr float DJI_ENCODER_TO_DEG = 360.0f / static_cast<float>(DJI_ENCODER_MAX);
constexpr float DEG_TO_DJI_ENCODER = static_cast<float>(DJI_ENCODER_MAX) / 360.0f;

constexpr float HI14_YAW_SCALE = 180.0f / 2048.0f;

constexpr uint32_t DEFAULT_IMU_TIMEOUT_MS = 100;
constexpr uint32_t DEFAULT_MOTOR_TIMEOUT_MS = 100;
constexpr uint32_t DEFAULT_MOTOR_OFFLINE_TIMEOUT_MS = 50;
} // namespace Constants
inline float deg_to_rad(float deg)
{
    return deg * Constants::DEG_TO_RAD;
}
inline float normalize_angle(float angle)
{
    return ALG::Omni::AngleMath::symmetric(angle, Constants::PI, Constants::TWO_PI);
}
inline float normalize_angle_0_2pi(float angle)
{
    return ALG::Omni::AngleMath::positive_radians(angle);
}
inline float normalize_angle_error(float error_rad)
{
    return ALG::Omni::AngleMath::error_radians(error_rad);
}
inline float clamp(float value, float min_val, float max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}
inline void normalize_planar_speed(float &vx_ref, float &vy_ref, float max_planar_speed)
{
    if (max_planar_speed <= 0.0f)
        return;

    const float planar_speed = std::sqrtf(vx_ref * vx_ref + vy_ref * vy_ref);

    if (planar_speed > max_planar_speed && planar_speed > 0.0f)
    {
        const float scale = max_planar_speed / planar_speed;
        vx_ref *= scale;
        vy_ref *= scale;
    }
}
inline float calculate_follow_wz(float yaw_standard_rad, float yaw_fb_rad,
                                 ALG::Omni::TimePid &follow_pid, float dt_sec, float deadband_rad,
                                 float half_range_rad = Constants::HALF_PI)
{
    if (!std::isfinite(yaw_standard_rad) || !std::isfinite(yaw_fb_rad) || !std::isfinite(dt_sec) ||
        dt_sec <= 0.0f || !std::isfinite(deadband_rad) || !std::isfinite(half_range_rad) ||
        half_range_rad <= 0.0f)
    {
        follow_pid.reset();
        return 0.0f;
    }

    float follow_error = yaw_standard_rad - yaw_fb_rad;
    const float full_range = 2.0f * half_range_rad;
    follow_error = std::fmod(follow_error + half_range_rad, full_range);
    if (follow_error < 0.0f)
    {
        follow_error += full_range;
    }
    follow_error -= half_range_rad;

    if (std::fabs(follow_error) < std::fabs(deadband_rad))
    {
        follow_error = 0.0f;
    }

    return follow_pid.update(0.0f, follow_error, dt_sec);
}
inline void calculate_translation_xy(float theta, float vx, float vy, float yaw_standard,
                                     float remote_to_chassis_gain, float phi, float psi,
                                     float actual_vx, float actual_vy, float actual_wz,
                                     float *out_vx, float *out_vy, float dynamic_k_base = 0.02f)
{
    if (out_vx == nullptr || out_vy == nullptr)
    {
        return;
    }

    theta = normalize_angle(theta - yaw_standard + phi);

    const float s = std::sin(theta + psi);
    const float c = std::cos(theta + psi);

    const float raw_vx = remote_to_chassis_gain * vy;
    const float raw_vy = -remote_to_chassis_gain * vx;

    const float ref_chassis_vx = raw_vx * c - raw_vy * s;
    const float ref_chassis_vy = raw_vx * s + raw_vy * c;

    const float kp = dynamic_k_base * std::fabs(actual_wz);
    *out_vx = ref_chassis_vx + kp * (ref_chassis_vx - actual_vx);
    *out_vy = ref_chassis_vy + kp * (ref_chassis_vy - actual_vy);
}
} // namespace Sentry::Utils

namespace Sentry
{
/** @brief 在连续角度参考中限制机械行程，输入和输出单位为 rad。 */
class AngleLimitGuard
{
  public:
    AngleLimitGuard() : target_(0), min_(0), max_(0), recovery_(false) {}

    AngleLimitGuard(float min_val, float max_val)
        : target_(0), min_(min_val), max_(max_val), recovery_(false)
    {
    }

    void set_limits(float min_val, float max_val)
    {
        min_ = min_val;
        max_ = max_val;
    }

    void reset(float current)
    {
        target_ = current;
        recovery_ = (current > max_ || current < min_);
    }

    float update(float delta)
    {
        if (recovery_)
        {
            return update_recovery(delta);
        }
        return update_normal(delta);
    }

    bool is_in_recovery() const { return recovery_; }

    float target() const { return target_; }

  private:
    float update_normal(float delta)
    {
        target_ += delta;
        if (target_ > max_)
            target_ = max_;
        if (target_ < min_)
            target_ = min_;
        return target_;
    }

    float update_recovery(float delta)
    {
        float candidate = target_ + delta;

        if (target_ > max_)
        {

            target_ = (candidate < target_) ? candidate : target_;
        }
        else if (target_ < min_)
        {

            target_ = (candidate > target_) ? candidate : target_;
        }

        if (target_ <= max_ && target_ >= min_)
        {
            recovery_ = false;
        }
        return target_;
    }

    float target_;
    float min_, max_;
    bool recovery_;
};
} // namespace Sentry
