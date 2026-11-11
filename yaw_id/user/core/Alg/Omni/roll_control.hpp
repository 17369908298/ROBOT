#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "user/core/Alg/Omni/angle_math.hpp"

namespace ALG::Omni
{

namespace RollDetail
{
inline constexpr float pi = 3.14159265358979323846f;
inline float clamp(float x, float lo, float hi)
{
    return std::max(lo, std::min(x, hi));
}
inline float angle_error(float target, float current)
{
    const float delta = target - current;
    return delta >= -pi && delta <= pi ? delta : std::remainder(delta, 2.0f * pi);
}
inline float approach(float current, float target, float step)
{
    return current + clamp(target - current, -step, step);
}
inline float bernstein(const float *points, unsigned degree, float s)
{
    float work[8]{};
    for (unsigned i = 0; i <= degree; ++i)
        work[i] = points[i];
    for (unsigned n = degree; n > 0; --n)
        for (unsigned i = 0; i < n; ++i)
            work[i] += s * (work[i + 1] - work[i]);
    return work[0];
}

inline bool bounded_bernstein(const float *points, unsigned degree, float limit)
{

    float pending[6][8]{}, curve[8]{}, work[8]{}, left[8]{};
    unsigned pending_depth[6]{}, count = 0, depth = 6;
    for (unsigned i = 0; i <= degree; ++i)
        curve[i] = points[i];
    for (;;)
    {
        bool inside = true;
        for (unsigned i = 0; i <= degree; ++i)
            inside = inside && std::fabs(curve[i]) <= limit;
        if (inside)
        {
            if (count == 0)
                return true;
            --count;
            for (unsigned i = 0; i <= degree; ++i)
                curve[i] = pending[count][i];
            depth = pending_depth[count];
            continue;
        }
        if (depth == 0 || std::fabs(curve[0]) > limit || std::fabs(curve[degree]) > limit)
            return false;
        for (unsigned i = 0; i <= degree; ++i)
            work[i] = curve[i];
        left[0] = work[0];
        pending[count][degree] = work[degree];
        for (unsigned n = degree; n > 0; --n)
        {
            for (unsigned i = 0; i < n; ++i)
                work[i] = 0.5f * (work[i] + work[i + 1]);
            left[degree - n + 1] = work[0];
            pending[count][n - 1] = work[n - 1];
        }
        pending_depth[count++] = --depth;
        for (unsigned i = 0; i <= degree; ++i)
            curve[i] = left[i];
    }
}
} // namespace RollDetail

/** @brief roll 轴位置轨迹规划器，输出平滑角度和角速度参考。 */
class RollPositionTrajectory final
{
  public:
    struct State
    {
        float position{}, velocity{}, acceleration{}, jerk{};
    };

    void reset(float position)
    {
        state_ = {std::isfinite(position) ? position : 0.0f, 0, 0, 0};
        ref_ = state_.position;
        elapsed_ = duration_ = 0.0f;
        done_ = true;
    }

    bool plan(float target, float vmax, float amax, float jmax)
    {
        if (!std::isfinite(target) || !std::isfinite(vmax) || !std::isfinite(amax) ||
            !std::isfinite(jmax) || vmax <= 0 || amax <= 0 || jmax <= 0)
            return false;
        const float delta = RollDetail::angle_error(target, state_.position);
        if (std::fabs(delta) < 1.0e-6f && done_)
            return true;

        float t = std::max(0.05f, 2.1875f * std::fabs(delta) / vmax);
        t = std::max(t, std::sqrt(7.52f * std::fabs(delta) / amax));
        t = std::max(t, std::cbrt(52.5f * std::fabs(delta) / jmax));
        for (unsigned attempt = 0; attempt < 20; ++attempt, t *= 1.12f)
        {
            if (t > 2.8f)
                return false;
            float p[8] = {0.0f,
                          state_.velocity * t / 7.0f,
                          2.0f * state_.velocity * t / 7.0f + state_.acceleration * t * t / 42.0f,
                          3.0f * state_.velocity * t / 7.0f + state_.acceleration * t * t / 14.0f +
                              state_.jerk * t * t * t / 210.0f,
                          delta,
                          delta,
                          delta,
                          delta};
            float v[7]{}, a[6]{}, j[5]{};
            for (unsigned i = 0; i < 7; ++i)
            {
                v[i] = 7.0f * (p[i + 1] - p[i]) / t;
            }
            for (unsigned i = 0; i < 6; ++i)
            {
                a[i] = 6.0f * (v[i + 1] - v[i]) / t;
            }
            for (unsigned i = 0; i < 5; ++i)
            {
                j[i] = 5.0f * (a[i + 1] - a[i]) / t;
            }
            const bool bounded = RollDetail::bounded_bernstein(v, 6, vmax * 1.00001f) &&
                                 RollDetail::bounded_bernstein(a, 5, amax * 1.00001f) &&
                                 RollDetail::bounded_bernstein(j, 4, jmax * 1.00001f);
            if (!bounded)
                continue;
            start_ = state_.position;
            ref_ = start_ + delta;
            for (unsigned i = 0; i < 8; ++i)
                p_[i] = p[i];
            for (unsigned i = 0; i < 7; ++i)
                v_[i] = v[i];
            for (unsigned i = 0; i < 6; ++i)
                a_[i] = a[i];
            for (unsigned i = 0; i < 5; ++i)
                j_[i] = j[i];
            duration_ = t;
            elapsed_ = 0;
            done_ = false;
            return true;
        }
        return false;
    }

    const State &update(float dt)
    {
        if (done_ || !std::isfinite(dt) || dt <= 0)
            return state_;
        elapsed_ = std::min(duration_, elapsed_ + dt);
        const float s = elapsed_ / duration_;
        state_ = {start_ + RollDetail::bernstein(p_, 7, s), RollDetail::bernstein(v_, 6, s),
                  RollDetail::bernstein(a_, 5, s), RollDetail::bernstein(j_, 4, s)};
        if (elapsed_ >= duration_)
        {
            state_ = {ref_, 0, 0, 0};
            done_ = true;
        }
        return state_;
    }
    const State &state() const { return state_; }
    float target() const { return ref_; }
    float duration() const { return duration_; }
    bool done() const { return done_; }

  private:
    State state_{};
    float p_[8]{}, v_[7]{}, a_[6]{}, j_[5]{};
    float start_{}, ref_{}, duration_{}, elapsed_{};
    bool done_{true};
};

/** @brief roll 轴阻抗控制器，计算速度、阻尼和前馈力矩。 */
class RollImpedanceController final
{
  public:
    struct Config
    {

        float stiffness_nm_per_rad{48.4f};

        float damping_nms_rad{2.2f};

        float tf_stiffness_ratio{0.5f};

        float speed_limit_radps{30.0f};

        float torque_limit_nm{12.5f};

        float gravity_amplitude_nm{1.3f};

        float gravity_offset_rad{-1.457349925f};

        float torque_offset_nm{0.0f};

        float max_speed_radps{10.0f};

        float max_accel_radps2{40.0f};

        float max_jerk_radps3{300.0f};
    };
    struct Output
    {
        float reference_angle_rad{}, reference_speed_radps{}, reference_accel_radps2{};
        float stiffness_nm_per_rad{}, damping_nms_rad{};
        float position_torque_nm{}, gravity_torque_nm{}, inertia_torque_nm{};
        float dist_estimate_nm{}, comp_torque_nm{};
        float mit_ff_nm{}, applied_torque_estimate_nm{};
        bool valid{false};
    };

    void configure(const Config &config) { config_ = config; }
    const Config &config() const { return config_; }
    void set_gains(float stiffness, float damping)
    {
        config_.stiffness_nm_per_rad = stiffness;
        config_.damping_nms_rad = damping;
    }
    void reset(float angle = 0.0f)
    {
        traj_.reset(angle);
        output_ = {};
        previous_command_ = {};
        observer_speed_ = dist_ = comp_ = 0.0f;
        last_fb_ms_ = 0;
        observer_ready_ = gains_ready_ = false;
        stiffness_scale_ = config_.tf_stiffness_ratio;
        normal_elapsed_ = 0.0f;
    }

    const Output &update(float target, float angle, float velocity, std::uint32_t fb_ms, float dt,
                         bool normal_owner)
    {
        if (!valid_config() || !std::isfinite(target) || !std::isfinite(angle) ||
            !std::isfinite(velocity) || !std::isfinite(dt) || dt <= 0 || dt > 0.02f)
        {
            output_.valid = false;
            return output_;
        }
        if (std::fabs(RollDetail::angle_error(target, traj_.target())) > 1.0e-5f &&
            !traj_.plan(target, std::min(config_.max_speed_radps, config_.speed_limit_radps),
                        config_.max_accel_radps2, config_.max_jerk_radps3))
        {
            output_.valid = false;
            return output_;
        }
        const auto &reference = traj_.update(dt);
        const bool normal_hold = normal_owner && traj_.done();
        normal_elapsed_ = normal_hold ? normal_elapsed_ + dt : 0.0f;
        const float scale_ref = normal_elapsed_ >= 0.1f ? 1.0f : config_.tf_stiffness_ratio;
        stiffness_scale_ = RollDetail::approach(stiffness_scale_, scale_ref, 2.5f * dt);
        const float stiffness_ref = config_.stiffness_nm_per_rad * stiffness_scale_;
        if (!gains_ready_)
        {
            output_.stiffness_nm_per_rad = stiffness_ref;
            output_.damping_nms_rad = config_.damping_nms_rad;
            gains_ready_ = true;
        }
        else
        {

            output_.stiffness_nm_per_rad =
                RollDetail::approach(output_.stiffness_nm_per_rad, stiffness_ref,
                                     std::max(1.0f, config_.stiffness_nm_per_rad) * 5.0f * dt);
            output_.damping_nms_rad =
                RollDetail::approach(output_.damping_nms_rad, config_.damping_nms_rad, 25.0f * dt);
        }

        update_observer(velocity, fb_ms, normal_hold);
        const float wanted_comp = normal_hold ? RollDetail::clamp(-dist_, -1.0f, 1.0f) : 0.0f;
        comp_ = RollDetail::approach(comp_, wanted_comp, 2.0f * dt);
        output_.reference_angle_rad = reference.position;
        output_.reference_speed_radps = reference.velocity;
        output_.reference_accel_radps2 = reference.acceleration;
        output_.position_torque_nm =
            output_.stiffness_nm_per_rad * RollDetail::angle_error(reference.position, angle);
        output_.gravity_torque_nm =
            config_.gravity_amplitude_nm == 0.0f
                ? 0.0f
                : config_.gravity_amplitude_nm *
                      gravity_cosine_.get(angle + config_.gravity_offset_rad);
        output_.inertia_torque_nm = nominal_inertia * reference.acceleration;
        output_.dist_estimate_nm = dist_;
        output_.comp_torque_nm = comp_;
        const float damping_torque = output_.damping_nms_rad * (reference.velocity - velocity);
        const float requested_ff = output_.position_torque_nm + output_.gravity_torque_nm +
                                   output_.inertia_torque_nm + comp_ + config_.torque_offset_nm;

        const float limited_total = RollDetail::clamp(
            requested_ff + damping_torque, -config_.torque_limit_nm, config_.torque_limit_nm);
        output_.mit_ff_nm = RollDetail::clamp(limited_total - damping_torque,
                                              -config_.torque_limit_nm, config_.torque_limit_nm);
        output_.applied_torque_estimate_nm = RollDetail::clamp(
            output_.mit_ff_nm + damping_torque, -config_.torque_limit_nm, config_.torque_limit_nm);
        output_.valid = std::isfinite(output_.mit_ff_nm);
        return output_;
    }

    void set_applied_command(float velocity, float damping, float torque, float gravity,
                             bool enabled)
    {
        previous_command_ = enabled ? Command{velocity, damping, torque, gravity} : Command{};
    }
    const Output &output() const { return output_; }
    const RollPositionTrajectory &trajectory() const { return traj_; }

  private:
    struct Command
    {
        float velocity{}, damping{}, torque{}, gravity{};
    };
    AngleMath::CosineCache gravity_cosine_{};

    static constexpr float nominal_inertia = 0.05f;

    static constexpr float physical_damping = 0.05f;

    static constexpr float observer_bandwidth = 2.0f * RollDetail::pi;

    bool valid_config() const
    {
        return std::isfinite(config_.stiffness_nm_per_rad) && config_.stiffness_nm_per_rad >= 0 &&
               config_.stiffness_nm_per_rad <= 500 && std::isfinite(config_.damping_nms_rad) &&
               config_.damping_nms_rad > 0 && config_.damping_nms_rad <= 5 &&
               std::isfinite(config_.tf_stiffness_ratio) && config_.tf_stiffness_ratio >= 0 &&
               config_.tf_stiffness_ratio <= 1 && std::isfinite(config_.speed_limit_radps) &&
               config_.speed_limit_radps > 0 && config_.speed_limit_radps <= 30 &&
               std::isfinite(config_.torque_limit_nm) && config_.torque_limit_nm > 0 &&
               config_.torque_limit_nm <= 12.5f && std::isfinite(config_.gravity_amplitude_nm) &&
               std::isfinite(config_.gravity_offset_rad) &&
               std::isfinite(config_.torque_offset_nm) && std::isfinite(config_.max_speed_radps) &&
               config_.max_speed_radps > 0 && std::isfinite(config_.max_accel_radps2) &&
               config_.max_accel_radps2 > 0 && std::isfinite(config_.max_jerk_radps3) &&
               config_.max_jerk_radps3 > 0;
    }
    void update_observer(float velocity, std::uint32_t fb_ms, bool enabled)
    {
        if (!enabled)
        {
            observer_ready_ = false;
            dist_ = 0;
            last_fb_ms_ = fb_ms;
            return;
        }
        if (fb_ms == 0 || fb_ms == last_fb_ms_)
            return;
        const std::uint32_t elapsed_ms = fb_ms - last_fb_ms_;
        last_fb_ms_ = fb_ms;
        if (!observer_ready_ || elapsed_ms > 20)
        {
            observer_speed_ = velocity;
            dist_ = 0;
            observer_ready_ = true;
            return;
        }
        const float dt = static_cast<float>(elapsed_ms) * 0.001f;
        const float applied_torque = RollDetail::clamp(
            previous_command_.torque +
                previous_command_.damping * (previous_command_.velocity - velocity),
            -config_.torque_limit_nm, config_.torque_limit_nm);
        const float residual_input =
            applied_torque - previous_command_.gravity - physical_damping * velocity;
        const float error = velocity - observer_speed_;
        observer_speed_ +=
            dt * ((residual_input + dist_) / nominal_inertia + 2.0f * observer_bandwidth * error);
        dist_ = RollDetail::clamp(dist_ + dt * nominal_inertia * observer_bandwidth *
                                              observer_bandwidth * error,
                                  -2.0f, 2.0f);
        if (!std::isfinite(observer_speed_) || !std::isfinite(dist_))
        {
            observer_speed_ = velocity;
            dist_ = 0;
        }
    }

    Config config_{};
    Output output_{};
    Command previous_command_{};
    RollPositionTrajectory traj_{};
    float observer_speed_{}, dist_{}, comp_{}, stiffness_scale_{0.5f};
    float normal_elapsed_{};
    std::uint32_t last_fb_ms_{};
    bool observer_ready_{false}, gains_ready_{false};
};

} // namespace ALG::Omni
