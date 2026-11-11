#pragma once

#include <cmath>

namespace ALG
{
namespace Omni
{

namespace VelocityPlannerDetail
{

inline bool is_valid(float value)
{
    return std::isfinite(value);
}

inline bool is_valid_dt(float dt_sec)
{
    return is_valid(dt_sec) && dt_sec > 0.0f;
}

inline float abs(float value)
{
    return std::fabs(value);
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
    if (!is_valid(limit) || limit <= 0.0f)
    {
        return value;
    }
    const float abs_limit = abs(limit);
    return clamp(value, -abs_limit, abs_limit);
}

inline float positive_or(float value, float fallback)
{
    if (is_valid(value) && value > 0.0f)
    {
        return value;
    }
    if (is_valid(fallback) && fallback > 0.0f)
    {
        return fallback;
    }
    return 0.0f;
}

inline float max_positive(float lhs, float rhs)
{
    const float a = (is_valid(lhs) && lhs > 0.0f) ? lhs : 0.0f;
    const float b = (is_valid(rhs) && rhs > 0.0f) ? rhs : 0.0f;
    return (a > b) ? a : b;
}

inline float select_accel_limit(float current, float target, float max_accel, float max_decel)
{
    const float accel_limit = positive_or(max_accel, max_decel);
    const float decel_limit = positive_or(max_decel, max_accel);
    const bool same_direction = current * target >= 0.0f;
    const bool increasing_speed = same_direction && abs(target) > abs(current);
    return increasing_speed ? accel_limit : decel_limit;
}

} // namespace VelocityPlannerDetail

/** @brief 按最大加速度限制目标变化率。 */
class SlopePlanning
{
  public:
    SlopePlanning(float increase_value = 0.0f, float decrease_value = 0.0f)
        : Increase_Value(increase_value), Decrease_Value(decrease_value), Now_Planning(0.0f),
          Now_Real(0.0f), Target(0.0f), Out(0.0f)
    {
    }

    void update(float target, float feedback = 0.0f)
    {
        Target = target;
        Now_Real = feedback;

        const float diff = Target - Now_Planning;
        const float abs_diff = VelocityPlannerDetail::abs(diff);
        const float step = (diff >= 0.0f) ? Increase_Value : Decrease_Value;

        if (step <= 0.0f || abs_diff <= step)
        {
            Out = Target;
        }
        else
        {
            Out = Now_Planning + ((diff > 0.0f) ? step : -step);
        }

        Now_Planning = Out;
    }

    void reset(float value = 0.0f)
    {
        Now_Planning = value;
        Now_Real = value;
        Target = value;
        Out = value;
    }

    float GetOut() const { return Out; }

    float GetNowPlanning() const { return Now_Planning; }

    float GetTarget() const { return Target; }

    void SetNowReal(float now_real) { Now_Real = now_real; }

    void SetIncreaseValue(float value) { Increase_Value = value; }

    void SetDecreaseValue(float value) { Decrease_Value = value; }

    void SetTarget(float target) { Target = target; }

  private:
    float Increase_Value, Decrease_Value, Now_Planning, Now_Real, Target, Out;
};

/** @brief 生成速度、加速度受限的梯形速度曲线。 */
class TrapezoidVelocityPlanning
{
  public:
    struct Config
    {
        float max_speed = 0.0f;
        float max_accel = 0.05f;
        float max_decel = 0.08f;
    };

    TrapezoidVelocityPlanning() = default;
    explicit TrapezoidVelocityPlanning(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    float update(float target, float dt_sec)
    {
        if (!VelocityPlannerDetail::is_valid(target))
        {
            return Out;
        }

        Target = VelocityPlannerDetail::clamp_abs(target, config_.max_speed);
        if (!VelocityPlannerDetail::is_valid_dt(dt_sec))
        {
            return Out;
        }

        const float previous = Now_Planning;
        const float diff = Target - Now_Planning;
        const float accel_limit = VelocityPlannerDetail::select_accel_limit(
            Now_Planning, Target, config_.max_accel, config_.max_decel);

        if (accel_limit <= 0.0f || VelocityPlannerDetail::abs(diff) <= accel_limit * dt_sec)
        {
            Now_Planning = Target;
        }
        else
        {
            Now_Planning += VelocityPlannerDetail::sign(diff) * accel_limit * dt_sec;
        }

        Now_Planning = VelocityPlannerDetail::clamp_abs(Now_Planning, config_.max_speed);
        Accel = (Now_Planning - previous) / dt_sec;
        Out = Now_Planning;
        return Out;
    }

    void reset(float value = 0.0f)
    {
        Now_Planning = value;
        Target = value;
        Out = value;
        Accel = 0.0f;
    }

    float GetOut() const { return Out; }
    float GetNowPlanning() const { return Now_Planning; }
    float GetTarget() const { return Target; }
    float GetAcceleration() const { return Accel; }

  private:
    Config config_{};
    float Now_Planning{0.0f};
    float Target{0.0f};
    float Out{0.0f};
    float Accel{0.0f};
};

/** @brief 生成速度、加速度和 jerk 受限的 S 曲线。 */
class SCurveVelocityPlanning
{
  public:
    struct Config
    {
        float max_speed = 0.0f;
        float max_accel = 0.05f;
        float max_decel = 0.08f;
        float max_jerk = 0.02f;
    };

    SCurveVelocityPlanning() = default;
    explicit SCurveVelocityPlanning(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    float update(float target, float dt_sec)
    {
        if (!VelocityPlannerDetail::is_valid(target))
        {
            return Out;
        }

        Target = VelocityPlannerDetail::clamp_abs(target, config_.max_speed);
        if (!VelocityPlannerDetail::is_valid_dt(dt_sec))
        {
            return Out;
        }

        const float diff = Target - Now_Planning;
        const float accel_limit = VelocityPlannerDetail::select_accel_limit(
            Now_Planning, Target, config_.max_accel, config_.max_decel);

        if (accel_limit <= 0.0f)
        {
            Now_Planning = Target;
            Accel = 0.0f;
            Out = Now_Planning;
            return Out;
        }

        if (!VelocityPlannerDetail::is_valid(config_.max_jerk) || config_.max_jerk <= 0.0f)
        {
            return update_trapezoid(target, dt_sec, accel_limit);
        }

        const float previous = Now_Planning;
        const float ref_accel =
            VelocityPlannerDetail::clamp(diff / dt_sec, -accel_limit, accel_limit);
        const float max_delta_accel = config_.max_jerk * dt_sec;
        Accel += VelocityPlannerDetail::clamp(ref_accel - Accel, -max_delta_accel, max_delta_accel);

        const float abs_accel_limit =
            VelocityPlannerDetail::max_positive(config_.max_accel, config_.max_decel);
        if (abs_accel_limit > 0.0f)
        {
            Accel = VelocityPlannerDetail::clamp(Accel, -abs_accel_limit, abs_accel_limit);
        }

        float next = Now_Planning + Accel * dt_sec;
        if (diff == 0.0f || (Target - previous) * (Target - next) <= 0.0f)
        {
            next = Target;
            Accel = 0.0f;
        }

        Now_Planning = VelocityPlannerDetail::clamp_abs(next, config_.max_speed);
        Out = Now_Planning;
        return Out;
    }

    void reset(float value = 0.0f, float accel = 0.0f)
    {
        Now_Planning = value;
        Target = value;
        Out = value;
        Accel = accel;
    }

    float GetOut() const { return Out; }
    float GetNowPlanning() const { return Now_Planning; }
    float GetTarget() const { return Target; }
    float GetAcceleration() const { return Accel; }

  private:
    float update_trapezoid(float target, float dt_sec, float accel_limit)
    {
        Target = VelocityPlannerDetail::clamp_abs(target, config_.max_speed);
        const float previous = Now_Planning;
        const float diff = Target - Now_Planning;

        if (VelocityPlannerDetail::abs(diff) <= accel_limit * dt_sec)
        {
            Now_Planning = Target;
        }
        else
        {
            Now_Planning += VelocityPlannerDetail::sign(diff) * accel_limit * dt_sec;
        }

        Now_Planning = VelocityPlannerDetail::clamp_abs(Now_Planning, config_.max_speed);
        Accel = (Now_Planning - previous) / dt_sec;
        Out = Now_Planning;
        return Out;
    }

    Config config_{};
    float Now_Planning{0.0f};
    float Target{0.0f};
    float Out{0.0f};
    float Accel{0.0f};
};

enum class VelocityPlannerMode
{
    Slope = 0,
    Trapezoid,
    SCurve,
};

struct VelocityPlannerConfig
{
    float slope_increase = 0.05f;
    float slope_decrease = 0.08f;
    VelocityPlannerMode mode = VelocityPlannerMode::Slope;
    float dt_sec = 1.0f;
    float max_speed = 0.0f;
    float max_accel = 0.05f;
    float max_decel = 0.08f;
    float max_jerk = 0.02f;
};

/** @brief 根据配置选择并更新速度规划器。 */
class VelocityPlanner
{
  public:
    VelocityPlanner() { apply_config(); }

    explicit VelocityPlanner(const VelocityPlannerConfig &config) : config_(config)
    {
        apply_config();
    }

    void init(const VelocityPlannerConfig &config)
    {
        config_ = config;
        apply_config();
        reset();
    }

    void configure(const VelocityPlannerConfig &config)
    {
        config_ = config;
        apply_config();
    }

    void update(float ref_vx, float ref_vy, float ref_vw, float &out_vx, float &out_vy,
                float &out_vw)
    {
        update(ref_vx, ref_vy, ref_vw, config_.dt_sec, out_vx, out_vy, out_vw);
    }

    void update(float ref_vx, float ref_vy, float ref_vw, float dt_sec, float &out_vx,
                float &out_vy, float &out_vw)
    {
        switch (config_.mode)
        {
        case VelocityPlannerMode::Trapezoid:
            out_vx = trapezoid_x_.update(ref_vx, dt_sec);
            out_vy = trapezoid_y_.update(ref_vy, dt_sec);
            out_vw = trapezoid_w_.update(ref_vw, dt_sec);
            sync_slope(out_vx, out_vy, out_vw);
            sync_scurve(out_vx, out_vy, out_vw);
            break;

        case VelocityPlannerMode::SCurve:
            out_vx = scurve_x_.update(ref_vx, dt_sec);
            out_vy = scurve_y_.update(ref_vy, dt_sec);
            out_vw = scurve_w_.update(ref_vw, dt_sec);
            sync_slope(out_vx, out_vy, out_vw);
            sync_trapezoid(out_vx, out_vy, out_vw);
            break;

        case VelocityPlannerMode::Slope:
        default:
            slope_x_.update(ref_vx, 0.0f);
            slope_y_.update(ref_vy, 0.0f);
            slope_w_.update(ref_vw, 0.0f);

            out_vx = slope_x_.GetOut();
            out_vy = slope_y_.GetOut();
            out_vw = slope_w_.GetOut();
            sync_trapezoid(out_vx, out_vy, out_vw);
            sync_scurve(out_vx, out_vy, out_vw);
            break;
        }
    }

    void reset(float vx = 0.0f, float vy = 0.0f, float vw = 0.0f)
    {
        slope_x_.reset(vx);
        slope_y_.reset(vy);
        slope_w_.reset(vw);
        trapezoid_x_.reset(vx);
        trapezoid_y_.reset(vy);
        trapezoid_w_.reset(vw);
        scurve_x_.reset(vx);
        scurve_y_.reset(vy);
        scurve_w_.reset(vw);
    }

    bool is_enabled() const
    {
        switch (config_.mode)
        {
        case VelocityPlannerMode::Trapezoid:
            return config_.max_speed > 0.0f || config_.max_accel > 0.0f || config_.max_decel > 0.0f;

        case VelocityPlannerMode::SCurve:
            return config_.max_speed > 0.0f || config_.max_accel > 0.0f ||
                   config_.max_decel > 0.0f || config_.max_jerk > 0.0f;

        case VelocityPlannerMode::Slope:
        default:
            return config_.slope_increase > 0.0f || config_.slope_decrease > 0.0f;
        }
    }

    VelocityPlannerMode mode() const { return config_.mode; }

  private:
    void apply_config()
    {
        slope_x_.SetIncreaseValue(config_.slope_increase);
        slope_x_.SetDecreaseValue(config_.slope_decrease);
        slope_y_.SetIncreaseValue(config_.slope_increase);
        slope_y_.SetDecreaseValue(config_.slope_decrease);
        slope_w_.SetIncreaseValue(config_.slope_increase);
        slope_w_.SetDecreaseValue(config_.slope_decrease);

        TrapezoidVelocityPlanning::Config trapezoid_config;
        trapezoid_config.max_speed = config_.max_speed;
        trapezoid_config.max_accel = config_.max_accel;
        trapezoid_config.max_decel = config_.max_decel;
        trapezoid_x_.configure(trapezoid_config);
        trapezoid_y_.configure(trapezoid_config);
        trapezoid_w_.configure(trapezoid_config);

        SCurveVelocityPlanning::Config scurve_config;
        scurve_config.max_speed = config_.max_speed;
        scurve_config.max_accel = config_.max_accel;
        scurve_config.max_decel = config_.max_decel;
        scurve_config.max_jerk = config_.max_jerk;
        scurve_x_.configure(scurve_config);
        scurve_y_.configure(scurve_config);
        scurve_w_.configure(scurve_config);
    }

    void sync_slope(float vx, float vy, float vw)
    {
        slope_x_.reset(vx);
        slope_y_.reset(vy);
        slope_w_.reset(vw);
    }

    void sync_trapezoid(float vx, float vy, float vw)
    {
        trapezoid_x_.reset(vx);
        trapezoid_y_.reset(vy);
        trapezoid_w_.reset(vw);
    }

    void sync_scurve(float vx, float vy, float vw)
    {
        scurve_x_.reset(vx);
        scurve_y_.reset(vy);
        scurve_w_.reset(vw);
    }

    VelocityPlannerConfig config_;
    SlopePlanning slope_x_, slope_y_, slope_w_;
    TrapezoidVelocityPlanning trapezoid_x_, trapezoid_y_, trapezoid_w_;
    SCurveVelocityPlanning scurve_x_, scurve_y_, scurve_w_;
};

} // namespace Omni
} // namespace ALG
