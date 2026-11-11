#pragma once
#include <cmath>
namespace ALG::Omni
{
/** @brief 按实际 dt（秒）积分的 PID，沿用 Omni 参数和 anti-windup 计算。 */
class TimePid
{
  public:
    struct Config
    {
        float kp, ki, kd, output_max, output_min, integral_max;
    };
    TimePid() = default;
    explicit TimePid(const Config &c) { set_config(c); }
    void set_config(const Config &c)
    {
        kp_ = c.kp;
        ki_ = c.ki;
        kd_ = c.kd;
        output_max_ = c.output_max;
        output_min_ = c.output_min;
        integral_max_ = c.integral_max;
    }
    Config get_config() const { return {kp_, ki_, kd_, output_max_, output_min_, integral_max_}; }
    void set_ref(float ref) { ref_ = ref; }
    float get_ref() const { return ref_; }
    float update(float ref, float fb, float dt)
    {
        ref_ = ref;
        return update(fb, dt);
    }
    void reset()
    {
        integral_ = prev_error_ = 0.0f;
        first_update_ = true;
    }
    float update(float current_value, float dt)
    {
        if (dt <= 0.0f)
        {
            return 0.0f;
        }

        float error = ref_ - current_value;

        float p_term = kp_ * error;

        const float abs_error = std::fabs(error);
        bool integrate = true;
        if (integral_separation_enter_ > 0.0f)
        {
            const float reset_threshold = integral_separation_reset_ > integral_separation_enter_
                                              ? integral_separation_reset_
                                              : integral_separation_enter_;
            if (abs_error >= reset_threshold)
            {
                integral_ = 0.0f;
                integrate = false;
            }
            else if (abs_error > integral_separation_enter_)
            {
                integrate = false;
            }
        }

        if (integrate)
        {
            integral_ += error * dt;
        }
        if (integral_ > integral_max_)
        {
            integral_ = integral_max_;
        }
        else if (integral_ < -integral_max_)
        {
            integral_ = -integral_max_;
        }
        float i_term = ki_ * integral_;

        float d_term = 0.0f;
        if (!first_update_ && kd_ != 0.0f)
        {
            d_term = kd_ * (error - prev_error_) / dt;
        }
        prev_error_ = error;
        first_update_ = false;

        float output = p_term + i_term + d_term;

        if (output > output_max_)
        {
            output = output_max_;

            if (error > 0.0f && ki_ > 0.0f)
            {
                integral_ = (output_max_ - p_term - d_term) / ki_;
                if (integral_ < 0.0f)
                    integral_ = 0.0f;
            }
        }
        else if (output < output_min_)
        {
            output = output_min_;
            if (error < 0.0f && ki_ > 0.0f)
            {
                integral_ = (output_min_ - p_term - d_term) / ki_;
                if (integral_ > 0.0f)
                    integral_ = 0.0f;
            }
        }

        return output;
    }
    void set_integral_separation(float enter_error, float reset_error)
    {
        if (enter_error > 0.0f && std::isfinite(enter_error))
        {
            integral_separation_enter_ = enter_error;
            integral_separation_reset_ = (reset_error > enter_error && std::isfinite(reset_error))
                                             ? reset_error
                                             : enter_error;
        }
        else
        {
            integral_separation_enter_ = 0.0f;
            integral_separation_reset_ = 0.0f;
        }
        integral_ = 0.0f;
    }

  private:
    float kp_ = 0, ki_ = 0, kd_ = 0, output_max_ = 0, output_min_ = 0, integral_max_ = 0;
    float integral_separation_enter_ = 0, integral_separation_reset_ = 0;
    float ref_ = 0, integral_ = 0, prev_error_ = 0;
    bool first_update_ = true;
};
} // namespace ALG::Omni
