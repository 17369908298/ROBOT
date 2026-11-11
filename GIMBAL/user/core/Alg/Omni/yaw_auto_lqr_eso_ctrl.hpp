#pragma once

#include <cmath>
#include <cstdint>

namespace ALG
{
namespace Omni
{

namespace YawAutoLqrEsoDetail
{

constexpr float kMinInertiaKgM2 = 1.0e-6f;
constexpr float kMinDtSec = 1.0e-5f;
constexpr float kMaxDtSec = 0.05f;
constexpr float kDefaultDtSec = 0.001f;

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

inline float deadband(float value, float width)
{
    if (!std::isfinite(value))
    {
        return 0.0f;
    }
    if (!std::isfinite(width) || width <= 0.0f)
    {
        return value;
    }

    const float abs_value = std::fabs(value);
    if (abs_value <= width)
    {
        return 0.0f;
    }
    return value > 0.0f ? value - width : value + width;
}

inline float sanitize_dt(float dt_sec)
{
    if (!std::isfinite(dt_sec) || dt_sec < kMinDtSec || dt_sec > kMaxDtSec)
    {
        return kDefaultDtSec;
    }
    return dt_sec;
}

inline bool gate_pass(float value, float limit)
{
    if (!std::isfinite(limit) || limit <= 0.0f)
    {
        return true;
    }
    return std::isfinite(value) && std::fabs(value) <= limit;
}

} // namespace YawAutoLqrEsoDetail

/** @brief 使用 LQR 与扩张状态观测器生成小 yaw 力矩指令。 */
class YawAutoLqrEsoController
{
  public:
    struct Config
    {
        float j_kg_m2{0.0f};
        float b_nms_rad{0.0f};
        float k_theta{0.0f};
        float k_omega{0.0f};
        float k_i{0.0f};
        float theta_integral_limit_rad_s{0.0f};

        float ff_gain{1.0f};

        float tau_coulomb_nm{0.0f};
        float coulomb_smooth_rad_s{0.0f};

        float eso_bandwidth_rad_s{0.0f};
        float eso_comp_gain{0.0f};
        float eso_comp_limit_nm{0.0f};
        float eso_omega_gate_rad_s{0.0f};
        float eso_alpha_gate_rad_s2{0.0f};

        float tau_bias_ki{0.0f};
        float tau_bias_limit_nm{0.0f};
        float tau_meas_lpf_alpha{0.0f};
        float theta_deadband_rad{0.0f};

        float torque_soft_limit_nm{0.0f};
        float torque_min_nm{0.0f};
        float torque_max_nm{0.0f};
        float torque_slew_rate_nm_s{0.0f};

        bool eso_enable{false};
        bool eso_comp_enable{false};
        bool torque_slew_enable{false};

        float omega_deadband_rad_s{0.0f};

        float reference_speed_deadband_rad_s{0.0f};
    };

    struct State
    {
        float theta_rad{0.0f};
        float omega_rad_s{0.0f};
    };

    struct Reference
    {
        float theta_rad{0.0f};
        float omega_rad_s{0.0f};
        float alpha_rad_s2{0.0f};
    };

    struct Feedback
    {
        float theta_rad{0.0f};
        float omega_rad_s{0.0f};
        float tau_meas_nm{0.0f};
        bool fb_ok{false};
    };

    struct Observer
    {
        float z1{0.0f};
        float z2{0.0f};
        float z3{0.0f};
        float b0{0.0f};
        float w0{0.0f};
        float beta1{0.0f};
        float beta2{0.0f};
        float beta3{0.0f};
    };

    struct StateSpace
    {
        float a[2][2]{};
        float b[2][2]{};
        float c[2]{};
    };

    struct Output
    {
        float theta_ref_rad{0.0f};
        float omega_ref_rad_s{0.0f};
        float alpha_ref_rad_s2{0.0f};
        float e_theta_rad{0.0f};
        float e_omega_rad_s{0.0f};

        float tau_ff_alpha_nm{0.0f};
        float tau_ff_viscous_nm{0.0f};
        float tau_ff_coulomb_nm{0.0f};
        float tau_ff_nm{0.0f};
        float tau_i_nm{0.0f};
        float tau_lqr_nm{0.0f};

        float tau_eso_raw_nm{0.0f};
        float tau_eso_active_nm{0.0f};
        float tau_bias_nm{0.0f};
        float tau_pre_limit_nm{0.0f};
        float tau_cmd_before_slew_nm{0.0f};
        float tau_cmd_nm{0.0f};

        bool eso_comp_active{false};
        bool soft_limit_active{false};
        bool hard_limit_active{false};
        bool slew_limit_active{false};
    };

    YawAutoLqrEsoController() { configure(Config{}); }
    explicit YawAutoLqrEsoController(const Config &config) { configure(config); }

    void configure(const Config &config)
    {
        config_ = config;
        model_valid_ = std::isfinite(config_.j_kg_m2) && std::isfinite(config_.b_nms_rad) &&
                       config_.j_kg_m2 > YawAutoLqrEsoDetail::kMinInertiaKgM2;
        eso_damping_ = model_valid_ ? config_.b_nms_rad / config_.j_kg_m2 : 0.0f;
        ff_gain_ = std::isfinite(config_.ff_gain) ? config_.ff_gain : 1.0f;
        ff_inertia_ = ff_gain_ * config_.j_kg_m2;
        ff_damping_ = ff_gain_ * config_.b_nms_rad;
        tau_meas_alpha_ = YawAutoLqrEsoDetail::clamp(config_.tau_meas_lpf_alpha, 0.0f, 1.0f);
        coulomb_valid_ = std::isfinite(config_.tau_coulomb_nm) &&
                         std::isfinite(config_.coulomb_smooth_rad_s) &&
                         config_.tau_coulomb_nm != 0.0f && config_.coulomb_smooth_rad_s > 1.0e-6f;
        inverse_coulomb_smooth_ = coulomb_valid_ ? 1.0f / config_.coulomb_smooth_rad_s : 0.0f;
        reference_speed_deadband_ = std::isfinite(config_.reference_speed_deadband_rad_s)
                                        ? std::fmax(0.0f, config_.reference_speed_deadband_rad_s)
                                        : 0.0f;
        update_eso_gains();
    }

    void set_config(const Config &config) { configure(config); }

    Config get_config() const { return config_; }

    void reset() { reset(0.0f, 0.0f); }

    void reset(float theta_rad, float omega_rad_s)
    {
        observer_.z1 = std::isfinite(theta_rad) ? theta_rad : 0.0f;
        observer_.z2 = std::isfinite(omega_rad_s) ? omega_rad_s : 0.0f;
        observer_.z3 = 0.0f;
        theta_integral_rad_s_ = 0.0f;
        tau_bias_nm_ = 0.0f;
        tau_meas_filt_nm_ = 0.0f;
        tau_cmd_last_nm_ = 0.0f;
        fb_ready_ = false;
        last_output_ = Output{};
    }

    Output update(const Feedback &feedback, const Reference &reference, float dt_sec)
    {
        Output out{};
        dt_sec = YawAutoLqrEsoDetail::sanitize_dt(dt_sec);

        if (!feedback.fb_ok || !valid_fb(feedback) || !valid_reference(reference))
        {
            reset(feedback.theta_rad, feedback.omega_rad_s);
            last_output_ = out;
            return last_output_;
        }

        if (!fb_ready_)
        {
            fb_ready_ = true;
            tau_meas_filt_nm_ = feedback.tau_meas_nm;
            observer_.z1 = feedback.theta_rad;
            observer_.z2 = feedback.omega_rad_s;
            observer_.z3 = 0.0f;
            out.theta_ref_rad = reference.theta_rad;
            out.omega_ref_rad_s = reference.omega_rad_s;
            out.alpha_ref_rad_s2 = reference.alpha_rad_s2;
            last_output_ = out;
            return last_output_;
        }

        tau_meas_filt_nm_ += tau_meas_alpha_ * (feedback.tau_meas_nm - tau_meas_filt_nm_);

        update_eso(feedback.theta_rad, tau_cmd_last_nm_, dt_sec);

        const float e_theta = YawAutoLqrEsoDetail::deadband(
            feedback.theta_rad - reference.theta_rad, config_.theta_deadband_rad);
        const float e_omega = YawAutoLqrEsoDetail::deadband(
            feedback.omega_rad_s - reference.omega_rad_s, config_.omega_deadband_rad_s);

        theta_integral_rad_s_ += e_theta * dt_sec;
        theta_integral_rad_s_ = YawAutoLqrEsoDetail::clamp_abs(theta_integral_rad_s_,
                                                               config_.theta_integral_limit_rad_s);

        out.theta_ref_rad = reference.theta_rad;
        out.omega_ref_rad_s = reference.omega_rad_s;
        out.alpha_ref_rad_s2 = reference.alpha_rad_s2;
        out.e_theta_rad = e_theta;
        out.e_omega_rad_s = e_omega;

        out.tau_ff_alpha_nm = ff_inertia_ * reference.alpha_rad_s2;
        out.tau_ff_viscous_nm = ff_damping_ * reference.omega_rad_s;
        out.tau_ff_coulomb_nm = ff_gain_ * coulomb_comp(reference.omega_rad_s);
        out.tau_ff_nm = out.tau_ff_alpha_nm + out.tau_ff_viscous_nm + out.tau_ff_coulomb_nm;

        out.tau_i_nm = -config_.k_i * theta_integral_rad_s_;
        out.tau_lqr_nm =
            out.tau_ff_nm - config_.k_theta * e_theta - config_.k_omega * e_omega + out.tau_i_nm;

        out.tau_eso_raw_nm = eso_comp();
        if (config_.eso_comp_enable &&
            YawAutoLqrEsoDetail::gate_pass(feedback.omega_rad_s, config_.eso_omega_gate_rad_s) &&
            YawAutoLqrEsoDetail::gate_pass(reference.alpha_rad_s2, config_.eso_alpha_gate_rad_s2))
        {
            out.eso_comp_active = true;
            out.tau_eso_active_nm = out.tau_eso_raw_nm;
        }

        const float tau_without_bias = out.tau_lqr_nm + out.tau_eso_active_nm;
        const float tau_track_error = tau_without_bias - tau_meas_filt_nm_;
        tau_bias_nm_ += config_.tau_bias_ki * tau_track_error * dt_sec;
        tau_bias_nm_ = YawAutoLqrEsoDetail::clamp_abs(tau_bias_nm_, config_.tau_bias_limit_nm);

        out.tau_bias_nm = tau_bias_nm_;
        out.tau_pre_limit_nm = tau_without_bias + tau_bias_nm_;

        float tau_cmd = out.tau_pre_limit_nm;
        if (config_.torque_soft_limit_nm > 0.0f)
        {
            const float limited_tau =
                YawAutoLqrEsoDetail::clamp_abs(tau_cmd, config_.torque_soft_limit_nm);
            out.soft_limit_active = limited_tau != tau_cmd;
            tau_cmd = limited_tau;
        }

        if (config_.torque_min_nm < config_.torque_max_nm)
        {
            const float limited_tau =
                YawAutoLqrEsoDetail::clamp(tau_cmd, config_.torque_min_nm, config_.torque_max_nm);
            out.hard_limit_active = limited_tau != tau_cmd;
            tau_cmd = limited_tau;
        }

        out.tau_cmd_before_slew_nm = tau_cmd;
        if (config_.torque_slew_enable && config_.torque_slew_rate_nm_s > 0.0f &&
            std::isfinite(config_.torque_slew_rate_nm_s))
        {
            const float max_delta = config_.torque_slew_rate_nm_s * dt_sec;
            const float limited_tau = YawAutoLqrEsoDetail::clamp(
                tau_cmd, tau_cmd_last_nm_ - max_delta, tau_cmd_last_nm_ + max_delta);
            out.slew_limit_active = limited_tau != tau_cmd;
            tau_cmd = limited_tau;
        }

        out.tau_cmd_nm = tau_cmd;
        tau_cmd_last_nm_ = tau_cmd;
        last_output_ = out;
        return last_output_;
    }

    void update(const Feedback &feedback, const Reference &reference, float dt_sec, Output &output)
    {
        output = update(feedback, reference, dt_sec);
    }

    bool get_state_space(StateSpace &model) const { return get_state_space(config_, model); }

    static bool get_state_space(const Config &config, StateSpace &model)
    {
        model = StateSpace{};
        if (!std::isfinite(config.j_kg_m2) || !std::isfinite(config.b_nms_rad) ||
            config.j_kg_m2 <= YawAutoLqrEsoDetail::kMinInertiaKgM2)
        {
            return false;
        }

        const float inv_j = 1.0f / config.j_kg_m2;
        model.a[0][0] = 0.0f;
        model.a[0][1] = 1.0f;
        model.a[1][0] = 0.0f;
        model.a[1][1] = -config.b_nms_rad * inv_j;

        model.b[0][0] = 0.0f;
        model.b[0][1] = 0.0f;
        model.b[1][0] = inv_j;
        model.b[1][1] = inv_j;

        model.c[0] = 1.0f;
        model.c[1] = 0.0f;
        return true;
    }

    void set_applied_torque_nm(float torque_nm)
    {
        tau_cmd_last_nm_ = std::isfinite(torque_nm) ? torque_nm : 0.0f;
    }

    const Output &output() const { return last_output_; }

    const Observer &observer() const { return observer_; }

    float theta_integral_rad_s() const { return theta_integral_rad_s_; }

    float tau_bias_nm() const { return tau_bias_nm_; }

    float tau_meas_filt_nm() const { return tau_meas_filt_nm_; }

    float tau_cmd_last_nm() const { return tau_cmd_last_nm_; }

    bool fb_ready() const { return fb_ready_; }

  private:
    static bool valid_fb(const Feedback &feedback)
    {
        return std::isfinite(feedback.theta_rad) && std::isfinite(feedback.omega_rad_s) &&
               std::isfinite(feedback.tau_meas_nm);
    }

    static bool valid_reference(const Reference &reference)
    {
        return std::isfinite(reference.theta_rad) && std::isfinite(reference.omega_rad_s) &&
               std::isfinite(reference.alpha_rad_s2);
    }

    bool model_valid() const { return model_valid_; }

    void update_eso_gains()
    {
        if (!config_.eso_enable || !model_valid())
        {
            observer_.b0 = 0.0f;
            observer_.w0 = 0.0f;
            observer_.beta1 = 0.0f;
            observer_.beta2 = 0.0f;
            observer_.beta3 = 0.0f;
            return;
        }

        const float w0 =
            (std::isfinite(config_.eso_bandwidth_rad_s) && config_.eso_bandwidth_rad_s > 0.0f)
                ? config_.eso_bandwidth_rad_s
                : 0.0f;
        observer_.b0 = 1.0f / config_.j_kg_m2;
        observer_.w0 = w0;
        observer_.beta1 = 3.0f * w0;
        observer_.beta2 = 3.0f * w0 * w0;
        observer_.beta3 = w0 * w0 * w0;
    }

    void update_eso(float theta_meas_rad, float command_torque_nm, float dt_sec)
    {
        if (!config_.eso_enable || !model_valid() || observer_.w0 <= 0.0f)
        {
            return;
        }

        const float error = theta_meas_rad - observer_.z1;
        const float z1_dot = observer_.z2 + observer_.beta1 * error;
        const float z2_dot = -eso_damping_ * observer_.z2 + observer_.b0 * command_torque_nm +
                             observer_.z3 + observer_.beta2 * error;
        const float z3_dot = observer_.beta3 * error;

        observer_.z1 += dt_sec * z1_dot;
        observer_.z2 += dt_sec * z2_dot;
        observer_.z3 += dt_sec * z3_dot;
    }

    float coulomb_comp(float omega_ref_rad_s) const
    {
        if (!coulomb_valid_)
        {
            return 0.0f;
        }
        const float speed_deadband = reference_speed_deadband_;
        const float speed_abs = std::fabs(omega_ref_rad_s);
        if (speed_abs <= speed_deadband)
        {
            return 0.0f;
        }
        const float effective_speed = std::copysign(speed_abs - speed_deadband, omega_ref_rad_s);
        return config_.tau_coulomb_nm *
               static_cast<float>(std::tanh(effective_speed * inverse_coulomb_smooth_));
    }

    float eso_comp() const
    {
        if (!config_.eso_enable || observer_.b0 <= 1.0e-6f ||
            !std::isfinite(config_.eso_comp_gain) || config_.eso_comp_gain == 0.0f)
        {
            return 0.0f;
        }

        const float tau_eso = -config_.eso_comp_gain * observer_.z3 / observer_.b0;
        return YawAutoLqrEsoDetail::clamp_abs(tau_eso, config_.eso_comp_limit_nm);
    }

    Config config_{};
    bool model_valid_{false}, coulomb_valid_{false};
    float eso_damping_{0.0f}, ff_gain_{1.0f};
    float ff_inertia_{0.0f}, ff_damping_{0.0f}, tau_meas_alpha_{0.0f};
    float inverse_coulomb_smooth_{0.0f}, reference_speed_deadband_{0.0f};
    Observer observer_{};
    float theta_integral_rad_s_{0.0f};
    float tau_bias_nm_{0.0f};
    float tau_meas_filt_nm_{0.0f};
    float tau_cmd_last_nm_{0.0f};
    bool fb_ready_{false};
    Output last_output_{};
};

} // namespace Omni
} // namespace ALG
