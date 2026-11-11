#pragma once

#include <cmath>
#include "user/core/Alg/Omni/angle_math.hpp"

namespace ALG
{
namespace Omni
{

namespace DualYawDetail
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;

inline float clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

inline float clamp_abs(float value, float limit)
{
    if (!std::isfinite(limit) || limit <= 0.0f)
    {
        return value;
    }
    const float abs_limit = std::fabs(limit);
    return clamp(value, -abs_limit, abs_limit);
}

inline float sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    if (value < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

inline float wrap_angle_rad(float angle_rad)
{
    if (!std::isfinite(angle_rad))
    {
        return 0.0f;
    }
    return AngleMath::error_radians(angle_rad);
}

inline float first_order_alpha(float cutoff_hz, float dt_sec)
{
    if (!std::isfinite(cutoff_hz) || !std::isfinite(dt_sec) || cutoff_hz <= 0.0f || dt_sec <= 0.0f)
    {
        return 1.0f;
    }
    return clamp(1.0f - std::exp(-kTwoPi * cutoff_hz * dt_sec), 0.0f, 1.0f);
}

/** @brief 由截止频率和周期计算一阶低通滤波系数。 */
class FirstOrderAlphaCache
{
  public:
    float get(float cutoff_hz, float dt_sec)
    {
        if (cutoff_hz != cutoff_hz_ || dt_sec != dt_sec_)
        {
            alpha_ = first_order_alpha(cutoff_hz, dt_sec);
            cutoff_hz_ = cutoff_hz;
            dt_sec_ = dt_sec;
        }
        return alpha_;
    }

  private:
    float cutoff_hz_{-1.0f}, dt_sec_{-1.0f}, alpha_{1.0f};
};

/** @brief 由时间常数和周期计算一阶低通滤波系数。 */
class TimeConstantAlphaCache
{
  public:
    float get(float tau_sec, float dt_sec)
    {
        tau_sec = std::fmax(tau_sec, 0.0f);
        if (tau_sec != tau_sec_ || dt_sec != dt_sec_)
        {
            alpha_ = tau_sec <= 0.0f ? 1.0f : dt_sec / (tau_sec + dt_sec);
            tau_sec_ = tau_sec;
            dt_sec_ = dt_sec;
        }
        return alpha_;
    }

  private:
    float tau_sec_{-1.0f}, dt_sec_{-1.0f}, alpha_{1.0f};
};

inline float deadband_error(float value, float deadband)
{
    const float abs_deadband = std::fabs(deadband);
    const float abs_value = std::fabs(value);
    if (abs_value <= abs_deadband)
    {
        return 0.0f;
    }
    return sign(value) * (abs_value - abs_deadband);
}

} // namespace DualYawDetail

/** @brief 按截止频率滤除传感器反馈中的高频噪声。 */
class CutoffLowPassFilter
{
  public:
    explicit CutoffLowPassFilter(float cutoff_hz = 1.0f) : cutoff_hz_(cutoff_hz) {}

    void configure(float cutoff_hz) { cutoff_hz_ = cutoff_hz; }

    void reset(float value = 0.0f)
    {
        state_ = value;
        ready_ = true;
    }

    void clear()
    {
        state_ = 0.0f;
        ready_ = false;
    }

    float update(float input, float dt_sec)
    {
        if (!std::isfinite(input))
        {
            return state_;
        }
        if (!ready_)
        {
            reset(input);
            return state_;
        }
        const float alpha = alpha_cache_.get(cutoff_hz_, dt_sec);
        state_ += alpha * (input - state_);
        return state_;
    }

    float output() const { return state_; }
    float cutoff_hz() const { return cutoff_hz_; }
    bool initialized() const { return ready_; }

  private:
    float cutoff_hz_{1.0f};
    DualYawDetail::FirstOrderAlphaCache alpha_cache_{};
    float state_{0.0f};
    bool ready_{false};
};

/** @brief 融合陀螺仪角速度与编码器角度，估计连续 yaw 角。 */
class YawComplementaryFilter
{
  public:
    struct Config
    {
        float correction_cutoff_hz{1.0f};
        float gyro_bias_radps{0.0f};
    };

    YawComplementaryFilter() = default;
    explicit YawComplementaryFilter(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    void reset(float yaw_rad = 0.0f)
    {
        yaw_rad_ = DualYawDetail::wrap_angle_rad(yaw_rad);
        ready_ = true;
    }

    void clear()
    {
        yaw_rad_ = 0.0f;
        ready_ = false;
    }

    float update(float yaw_measurement_rad, float gyro_z_radps, float dt_sec)
    {
        if (!std::isfinite(yaw_measurement_rad) || !std::isfinite(gyro_z_radps) ||
            !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            return yaw_rad_;
        }
        if (!ready_)
        {
            reset(yaw_measurement_rad);
            return yaw_rad_;
        }

        const float predicted = yaw_rad_ + (gyro_z_radps - config_.gyro_bias_radps) * dt_sec;
        const float error = DualYawDetail::wrap_angle_rad(yaw_measurement_rad - predicted);
        const float alpha = alpha_cache_.get(config_.correction_cutoff_hz, dt_sec);
        yaw_rad_ = DualYawDetail::wrap_angle_rad(predicted + alpha * error);
        return yaw_rad_;
    }

    void set_gyro_bias(float gyro_bias_radps) { config_.gyro_bias_radps = gyro_bias_radps; }

    float yaw_rad() const { return yaw_rad_; }
    float gyro_bias_radps() const { return config_.gyro_bias_radps; }
    bool initialized() const { return ready_; }

  private:
    Config config_{};
    float yaw_rad_{0.0f};
    DualYawDetail::FirstOrderAlphaCache alpha_cache_{};
    bool ready_{false};
};

/** @brief 受速度、加速度和 jerk 约束的角度跟踪规划器。 */
class LimitedVelocityPlanner
{
  public:
    struct Config
    {
        float max_speed_radps{0.0f};
        float max_accel_radps2{0.0f};
        float max_jerk_radps3{0.0f};
    };

    LimitedVelocityPlanner() = default;
    explicit LimitedVelocityPlanner(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    void reset(float speed_radps = 0.0f, float accel_radps2 = 0.0f)
    {
        speed_radps_ = DualYawDetail::clamp_abs(speed_radps, config_.max_speed_radps);
        accel_radps2_ = accel_radps2;
    }

    float update(float ref_speed_radps, float dt_sec)
    {
        if (!std::isfinite(ref_speed_radps) || !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            return speed_radps_;
        }

        float target = ref_speed_radps;
        if (config_.max_speed_radps > 0.0f)
        {
            target = DualYawDetail::clamp_abs(target, config_.max_speed_radps);
        }

        if (config_.max_accel_radps2 <= 0.0f || !std::isfinite(config_.max_accel_radps2))
        {
            speed_radps_ = target;
            accel_radps2_ = 0.0f;
            return speed_radps_;
        }

        const float speed_error = target - speed_radps_;
        float ref_accel = DualYawDetail::clamp_abs(speed_error / dt_sec, config_.max_accel_radps2);

        if (config_.max_jerk_radps3 > 0.0f && std::isfinite(config_.max_jerk_radps3))
        {
            const float max_delta_accel = std::fabs(config_.max_jerk_radps3) * dt_sec;
            const float accel_delta =
                DualYawDetail::clamp(ref_accel - accel_radps2_, -max_delta_accel, max_delta_accel);
            accel_radps2_ += accel_delta;
        }
        else
        {
            accel_radps2_ = ref_accel;
        }

        float next_speed = speed_radps_ + accel_radps2_ * dt_sec;
        if ((target - speed_radps_) * (target - next_speed) <= 0.0f)
        {
            next_speed = target;
            accel_radps2_ = 0.0f;
        }
        if (config_.max_speed_radps > 0.0f)
        {
            next_speed = DualYawDetail::clamp_abs(next_speed, config_.max_speed_radps);
        }
        speed_radps_ = next_speed;
        return speed_radps_;
    }

    float speed_radps() const { return speed_radps_; }
    float accel_radps2() const { return accel_radps2_; }

  private:
    Config config_{};
    float speed_radps_{0.0f};
    float accel_radps2_{0.0f};
};

/** @brief 规划双 yaw 机构回中轨迹并输出角度参考。 */
class DualYawRecenterPlanner
{
  public:
    struct Config
    {
        float trend_cutoff_hz{0.8f};
        float recenter_kp_radps_per_rad{4.0f};
        float hysteresis_on_rad{0.052359878f};
        float hysteresis_off_rad{0.017453292f};
        float max_speed_radps{1.745329252f};
        float max_accel_radps2{6.981317008f};
        float max_jerk_radps3{34.90658504f};
        float limit_start_rad{0.418879021f};
        float limit_hard_rad{0.523598776f};
        float limit_gain_radps_per_rad{8.0f};
        float limit_max_speed_radps{2.443460953f};
    };

    struct Output
    {
        float theta_trend_rad{0.0f};
        float theta_high_rad{0.0f};
        float recenter_error_rad{0.0f};
        float speed_cmd_radps{0.0f};
        float speed_ref_radps{0.0f};
        float accel_ref_radps2{0.0f};
        bool recenter_active{false};
        bool limit_assist_active{false};
        bool hard_limit_active{false};
    };

    DualYawRecenterPlanner() { apply_planner_config(); }

    explicit DualYawRecenterPlanner(const Config &config)
        : config_(config), trend_filter_(config.trend_cutoff_hz)
    {
        apply_planner_config();
    }

    void configure(const Config &config)
    {
        config_ = config;
        trend_filter_.configure(config_.trend_cutoff_hz);
        apply_planner_config();
    }

    void reset(float yaw_offset_rad = 0.0f)
    {
        trend_filter_.reset(yaw_offset_rad);
        speed_planner_.reset(0.0f, 0.0f);
        recenter_active_ = false;
        last_output_ = Output{};
        last_output_.theta_trend_rad = yaw_offset_rad;
        last_output_.theta_high_rad = 0.0f;
    }

    Output update(float yaw_offset_rad, float dt_sec)
    {
        if (!std::isfinite(yaw_offset_rad) || !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            return last_output_;
        }

        const float theta_l = trend_filter_.update(yaw_offset_rad, dt_sec);
        const float abs_theta_l = std::fabs(theta_l);
        const float hysteresis_on = std::fabs(config_.hysteresis_on_rad);
        const float hysteresis_off = std::fabs(config_.hysteresis_off_rad);

        if (abs_theta_l > hysteresis_on)
        {
            recenter_active_ = true;
        }
        else if (abs_theta_l < hysteresis_off)
        {
            recenter_active_ = false;
        }

        const float recenter_error =
            recenter_active_ ? DualYawDetail::deadband_error(theta_l, hysteresis_off) : 0.0f;

        float speed_limit = config_.max_speed_radps;
        float speed_cmd = config_.recenter_kp_radps_per_rad * recenter_error;

        const float abs_theta0 = std::fabs(yaw_offset_rad);
        const bool limit_assist = abs_theta0 > std::fabs(config_.limit_start_rad);
        const bool hard_limit = abs_theta0 >= std::fabs(config_.limit_hard_rad);
        if (limit_assist)
        {
            const float limit_error = DualYawDetail::sign(yaw_offset_rad) *
                                      (abs_theta0 - std::fabs(config_.limit_start_rad));
            speed_cmd += config_.limit_gain_radps_per_rad * limit_error;
            if (config_.limit_max_speed_radps > speed_limit)
            {
                speed_limit = config_.limit_max_speed_radps;
            }
        }

        speed_cmd = DualYawDetail::clamp_abs(speed_cmd, speed_limit);
        speed_planner_.configure({
            speed_limit,
            config_.max_accel_radps2,
            config_.max_jerk_radps3,
        });
        const float speed_ref = speed_planner_.update(speed_cmd, dt_sec);

        last_output_.theta_trend_rad = theta_l;
        last_output_.theta_high_rad = yaw_offset_rad - theta_l;
        last_output_.recenter_error_rad = recenter_error;
        last_output_.speed_cmd_radps = speed_cmd;
        last_output_.speed_ref_radps = speed_ref;
        last_output_.accel_ref_radps2 = speed_planner_.accel_radps2();
        last_output_.recenter_active = recenter_active_;
        last_output_.limit_assist_active = limit_assist;
        last_output_.hard_limit_active = hard_limit;
        return last_output_;
    }

    const Output &output() const { return last_output_; }
    float trend_rad() const { return last_output_.theta_trend_rad; }
    float speed_ref_radps() const { return last_output_.speed_ref_radps; }

  private:
    void apply_planner_config()
    {
        speed_planner_.configure({
            config_.max_speed_radps,
            config_.max_accel_radps2,
            config_.max_jerk_radps3,
        });
    }

    Config config_{};
    CutoffLowPassFilter trend_filter_{config_.trend_cutoff_hz};
    LimitedVelocityPlanner speed_planner_{};
    bool recenter_active_{false};
    Output last_output_{};
};

/** @brief 根据目标和反馈生成大 yaw 平滑参考轨迹。 */
class BigYawReferenceTracker
{
  public:
    struct Config
    {
        float angle_kp_radps_per_rad{8.0f};
        float max_control_speed_radps{12.0f};
    };

    struct Output
    {
        float angle_ref_rad{0.0f};
        float angle_error_rad{0.0f};
        float control_speed_radps{0.0f};
    };

    BigYawReferenceTracker() = default;
    explicit BigYawReferenceTracker(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    void reset(float current_base_rad = 0.0f)
    {
        angle_ref_rad_ = DualYawDetail::wrap_angle_rad(current_base_rad);
        last_output_ = Output{};
        last_output_.angle_ref_rad = angle_ref_rad_;
    }

    Output update(float speed_ref_radps, float base_angle_rad, float dt_sec)
    {
        if (!std::isfinite(speed_ref_radps) || !std::isfinite(base_angle_rad) ||
            !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            return last_output_;
        }

        angle_ref_rad_ = DualYawDetail::wrap_angle_rad(angle_ref_rad_ + speed_ref_radps * dt_sec);
        const float angle_error = DualYawDetail::wrap_angle_rad(angle_ref_rad_ - base_angle_rad);
        float control_speed = speed_ref_radps + config_.angle_kp_radps_per_rad * angle_error;
        control_speed = DualYawDetail::clamp_abs(control_speed, config_.max_control_speed_radps);

        last_output_.angle_ref_rad = angle_ref_rad_;
        last_output_.angle_error_rad = angle_error;
        last_output_.control_speed_radps = control_speed;
        return last_output_;
    }

    const Output &output() const { return last_output_; }
    float angle_ref_rad() const { return angle_ref_rad_; }

  private:
    Config config_{};
    float angle_ref_rad_{0.0f};
    Output last_output_{};
};

/** @brief 从大 yaw 速度变化估计外部扰动力矩。 */
class BigYawSpeedDisturbanceObserver
{
  public:
    struct Config
    {
        float nominal_inertia_kgm2{0.0f};
        float nominal_damping_nms_rad{0.0f};
        float bandwidth_rad_s{0.0f};
        float comp_gain{0.0f};
        float comp_limit_nm{0.0f};
    };

    BigYawSpeedDisturbanceObserver() = default;
    explicit BigYawSpeedDisturbanceObserver(const Config &config) { configure(config); }

    void configure(const Config &config)
    {
        config_ = config;
        enabled_ = std::isfinite(config_.nominal_inertia_kgm2) &&
                   config_.nominal_inertia_kgm2 > 1.0e-6f &&
                   std::isfinite(config_.bandwidth_rad_s) && config_.bandwidth_rad_s > 0.0f &&
                   std::isfinite(config_.comp_gain) && config_.comp_gain != 0.0f &&
                   std::isfinite(config_.comp_limit_nm) && config_.comp_limit_nm > 0.0f;
        inverse_inertia_ = enabled_ ? 1.0f / config_.nominal_inertia_kgm2 : 0.0f;
        beta_1_ = 2.0f * config_.bandwidth_rad_s;
        inertia_beta_2_ =
            config_.nominal_inertia_kgm2 * (config_.bandwidth_rad_s * config_.bandwidth_rad_s);
    }

    void reset(float omega_radps = 0.0f)
    {
        z_omega_radps_ = std::isfinite(omega_radps) ? omega_radps : 0.0f;
        z_dist_nm_ = 0.0f;
        comp_nm_ = 0.0f;
        ready_ = false;
    }

    float update(float omega_measured_radps, float torque_applied_nm, float known_dist_nm,
                 float dt_sec)
    {
        if (!enabled() || !std::isfinite(omega_measured_radps) ||
            !std::isfinite(torque_applied_nm) || !std::isfinite(known_dist_nm) ||
            !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            comp_nm_ = 0.0f;
            return comp_nm_;
        }

        if (!ready_)
        {
            z_omega_radps_ = omega_measured_radps;
            z_dist_nm_ = 0.0f;
            comp_nm_ = 0.0f;
            ready_ = true;
            return comp_nm_;
        }

        const float error = omega_measured_radps - z_omega_radps_;
        const float model_accel = (torque_applied_nm + known_dist_nm -
                                   config_.nominal_damping_nms_rad * z_omega_radps_ + z_dist_nm_) *
                                  inverse_inertia_;

        z_omega_radps_ += dt_sec * (model_accel + beta_1_ * error);
        z_dist_nm_ += dt_sec * (inertia_beta_2_ * error);

        comp_nm_ = -config_.comp_gain * z_dist_nm_;
        comp_nm_ = DualYawDetail::clamp_abs(comp_nm_, config_.comp_limit_nm);
        return comp_nm_;
    }

    bool enabled() const { return enabled_; }

    float estimated_dist_nm() const { return z_dist_nm_; }
    float comp_nm() const { return comp_nm_; }

  private:
    Config config_{};
    float z_omega_radps_{0.0f};
    bool enabled_{false};
    float inverse_inertia_{0.0f}, beta_1_{0.0f}, inertia_beta_2_{0.0f};
    float z_dist_nm_{0.0f};
    float comp_nm_{0.0f};
    bool ready_{false};
};

/** @brief 根据双 yaw 相对运动补偿大 yaw 反作用力矩。 */
class DualYawReactionFeedforward
{
  public:
    struct Config
    {
        float yaw_torque_constant_nm_per_a{0.0f};
        float comp_gain{0.0f};
        float filter_cutoff_hz{30.0f};
        float torque_limit_nm{0.0f};
        float torque_slew_rate_nmps{0.0f};

        float coupled_inertia_kgm2{0.0f};
        float inertia_comp_gain{0.0f};
        float rel_speed_filter_cutoff_hz{20.0f};
        float rel_accel_filter_cutoff_hz{12.0f};
    };

    DualYawReactionFeedforward() = default;
    explicit DualYawReactionFeedforward(const Config &config)
        : config_(config), filter_(config.filter_cutoff_hz),
          rel_speed_filter_(config.rel_speed_filter_cutoff_hz),
          rel_accel_filter_(config.rel_accel_filter_cutoff_hz)
    {
    }

    void configure(const Config &config)
    {
        config_ = config;
        filter_.configure(config_.filter_cutoff_hz);
        rel_speed_filter_.configure(config_.rel_speed_filter_cutoff_hz);
        rel_accel_filter_.configure(config_.rel_accel_filter_cutoff_hz);
    }

    void reset(float torque_nm = 0.0f)
    {
        filter_.reset(torque_nm);
        rel_speed_filter_.clear();
        rel_accel_filter_.clear();
        rel_speed_last_radps_ = 0.0f;
        rel_speed_ready_ = false;
        output_torque_nm_ = torque_nm;
        inertia_comp_nm_ = 0.0f;
        rel_speed_radps_ = 0.0f;
        rel_accel_radps2_ = 0.0f;
    }

    float update_from_current(float yaw_current_a, float dt_sec)
    {
        const float estimated_torque = config_.yaw_torque_constant_nm_per_a * yaw_current_a;
        return update_from_yaw_torque(estimated_torque, dt_sec);
    }

    float update_from_yaw_torque(float yaw_torque_nm, float dt_sec)
    {
        return update_from_yaw_torque_and_rel_speed(yaw_torque_nm, 0.0f, dt_sec, false);
    }

    float update_from_yaw_torque_and_rel_speed(float yaw_torque_nm, float rel_speed_radps,
                                               float dt_sec)
    {
        return update_from_yaw_torque_and_rel_speed(yaw_torque_nm, rel_speed_radps, dt_sec, true);
    }

    float update_from_yaw_torque_and_rel_speed(float yaw_torque_nm, float rel_speed_radps,
                                               float dt_sec, bool rel_speed_valid)
    {
        if (!std::isfinite(yaw_torque_nm) || !std::isfinite(dt_sec) || dt_sec <= 0.0f)
        {
            return output_torque_nm_;
        }

        float torque_comp = config_.comp_gain * yaw_torque_nm;

        inertia_comp_nm_ = 0.0f;
        const bool inertia_comp_enabled =
            std::isfinite(config_.coupled_inertia_kgm2) && config_.coupled_inertia_kgm2 > 0.0f &&
            std::isfinite(config_.inertia_comp_gain) && config_.inertia_comp_gain != 0.0f;
        if (rel_speed_valid && std::isfinite(rel_speed_radps))
        {
            rel_speed_radps_ = rel_speed_filter_.update(rel_speed_radps, dt_sec);
            if (!rel_speed_ready_)
            {
                rel_speed_last_radps_ = rel_speed_radps_;
                rel_speed_ready_ = true;
            }
            else
            {
                const float accel_raw = (rel_speed_radps_ - rel_speed_last_radps_) / dt_sec;
                rel_speed_last_radps_ = rel_speed_radps_;
                rel_accel_radps2_ = rel_accel_filter_.update(accel_raw, dt_sec);
                if (inertia_comp_enabled)
                {
                    inertia_comp_nm_ = config_.inertia_comp_gain * config_.coupled_inertia_kgm2 *
                                       rel_accel_radps2_;
                }
            }
        }
        else
        {
            rel_speed_radps_ = 0.0f;
            rel_accel_radps2_ = 0.0f;
            rel_speed_last_radps_ = 0.0f;
            rel_speed_ready_ = false;
            rel_speed_filter_.clear();
            rel_accel_filter_.clear();
        }

        float target = torque_comp + inertia_comp_nm_;
        target = DualYawDetail::clamp_abs(target, config_.torque_limit_nm);
        target = filter_.update(target, dt_sec);

        if (config_.torque_slew_rate_nmps > 0.0f && std::isfinite(config_.torque_slew_rate_nmps))
        {
            const float max_delta = config_.torque_slew_rate_nmps * dt_sec;
            output_torque_nm_ +=
                DualYawDetail::clamp(target - output_torque_nm_, -max_delta, max_delta);
        }
        else
        {
            output_torque_nm_ = target;
        }
        output_torque_nm_ = DualYawDetail::clamp_abs(output_torque_nm_, config_.torque_limit_nm);
        return output_torque_nm_;
    }

    float output_torque_nm() const { return output_torque_nm_; }
    float inertia_comp_nm() const { return inertia_comp_nm_; }
    float rel_speed_radps() const { return rel_speed_radps_; }
    float rel_accel_radps2() const { return rel_accel_radps2_; }

  private:
    Config config_{};
    CutoffLowPassFilter filter_{30.0f};
    CutoffLowPassFilter rel_speed_filter_{20.0f};
    CutoffLowPassFilter rel_accel_filter_{12.0f};
    float output_torque_nm_{0.0f};
    float inertia_comp_nm_{0.0f};
    float rel_speed_radps_{0.0f};
    float rel_accel_radps2_{0.0f};
    float rel_speed_last_radps_{0.0f};
    bool rel_speed_ready_{false};
};

} // namespace Omni
} // namespace ALG
