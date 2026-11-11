#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace ALG
{
namespace Omni
{

/** @brief 根据电流、轮速和拟合系数估算各电机功率。 */
template <uint8_t N> class PowerControlBase
{
  public:
    static_assert(N > 0, "power control needs at least one motor");

    static float EstimatePower(float current_a, float speed_radps, const float *coeffs,
                               bool absolute_speed_loss = false)
    {
        const float loss_speed = absolute_speed_loss ? std::fabs(speed_radps) : speed_radps;
        return coeffs[0] + coeffs[1] * current_a + coeffs[2] * loss_speed +
               coeffs[3] * current_a * speed_radps + coeffs[4] * current_a * current_a +
               coeffs[5] * speed_radps * speed_radps;
    }

    void SetMotorParameters(uint8_t index, float current_a, float speed_radps, const float *coeffs,
                            bool absolute_speed_loss = false)
    {
        if (index >= N || coeffs == nullptr)
            return;
        auto &p = params_[index];
        p.current = current_a;
        p.velocity = speed_radps;
        for (uint8_t k = 0; k < 6; ++k)
            p.coeffs[k] = coeffs[k];
        p.power = EstimatePower(current_a, speed_radps, coeffs, absolute_speed_loss);
    }

    float GetPower(uint8_t i) const { return i < N ? params_[i].power : 0.0f; }
    float GetCurrent(uint8_t i) const { return i < N ? params_[i].current : 0.0f; }
    float GetVelocity(uint8_t i) const { return i < N ? params_[i].velocity : 0.0f; }

  protected:
    struct MotorParams
    {
        float current{0.0f};
        float velocity{0.0f};
        float power{0.0f};
        float coeffs[6]{};
    };
    MotorParams params_[N]{};
};

/** @brief 在底盘总功率约束下分配各轮电流。 */
template <uint8_t N> class PowerControl : public PowerControlBase<N>
{
  public:
    struct Config
    {
        bool absolute_speed_loss{false};

        bool credit_regeneration{false};

        float current_step_a{0.0f};
    };

    struct Result
    {
        float power_total{0.0f};
        float eta{1.0f};
        float rho{1.0f};
        bool limited{false};
        float power_limited{0.0f};
        bool feasible{true};
        bool valid{true};
    };

    void configure(const Config &config)
    {
        config_ = config;
        reset();
    }

    void reset()
    {
        result_ = {};
        correction_w_ = power_max_w_ = 0.0f;
        for (uint8_t i = 0; i < N; ++i)
        {
            current_[i] = 0.0f;
            active_[i] = false;
            this->params_[i] = {};
        }
    }

    Result AttenuatedPower(const float *I, const float *V, const float *K, float correction_w,
                           float power_max_w, const bool *active = nullptr)
    {
        if (!prepare(I, V, K, nullptr, correction_w, power_max_w, active))
            return result_;
        if (result_.power_total <= power_max_w_)
        {
            for (uint8_t i = 0; i < N; ++i)
                current_[i] = scaled_current(i, 1.0f);
            return finish();
        }
        const float baseline = power_at(0.0f);
        if (baseline > power_max_w_)
            return baseline_failure(baseline);
        float increments = 0.0f;
        for (uint8_t i = 0; i < N; ++i)
        {
            if (active_[i])
                increments += std::fmax(0.0f, power_at(1.0f, i) - power_at(0.0f, i));
        }
        result_.rho = increments > 0.0f
                          ? std::clamp((power_max_w_ - baseline) / increments, 0.0f, 1.0f)
                          : 0.0f;
        for (uint8_t i = 0; i < N; ++i)
        {
            if (!active_[i])
                continue;
            const float base = power_at(0.0f, i);
            const float demand = power_at(1.0f, i);
            const float budget = base + result_.rho * std::fmax(0.0f, demand - base);
            const float scale = demand <= budget ? 1.0f : find_scale(budget, i);
            current_[i] = scaled_current(i, scale);
            result_.limited = result_.limited || scale < 1.0f;
        }
        return finish();
    }

    Result DecayingCurrent(const float *I, const float *V, const float *K, const float *I_other,
                           float correction_w, float power_max_w, const bool *active = nullptr)
    {
        if (!prepare(I, V, K, I_other, correction_w, power_max_w, active))
            return result_;
        if (result_.power_total > power_max_w_)
        {
            const float baseline = power_at(0.0f);
            if (baseline > power_max_w_)
                return baseline_failure(baseline);
            result_.eta = find_scale(power_max_w_);
            result_.limited = result_.eta < 1.0f;
        }
        for (uint8_t i = 0; i < N; ++i)
            current_[i] = scaled_current(i, result_.eta);
        return finish();
    }

    float getCurrent(uint8_t i) const { return i < N ? current_[i] : 0.0f; }
    float getPowerTotal() const { return result_.power_total; }
    Result getResult() const { return result_; }

  private:
    static constexpr float POWER_TOLERANCE_W = 1.0e-4f;
    Config config_{};
    Result result_{};
    float current_[N]{};
    bool active_[N]{};
    float correction_w_{0.0f};
    float power_max_w_{0.0f};

    float budget_power(float power) const
    {
        return config_.credit_regeneration ? power : std::fmax(0.0f, power);
    }

    float scaled_current(uint8_t i, float eta) const
    {
        if (!active_[i])
            return 0.0f;
        const float current = eta * this->params_[i].current;
        return config_.current_step_a > 0.0f
                   ? std::trunc(current / config_.current_step_a) * config_.current_step_a
                   : current;
    }

    float motor_power(uint8_t i, float current) const
    {
        const auto &p = this->params_[i];
        return PowerControlBase<N>::EstimatePower(current, p.velocity, p.coeffs,
                                                  config_.absolute_speed_loss);
    }

    float power_at(float eta, int motor = -1) const
    {
        float power = motor < 0 ? correction_w_ : 0.0f;
        for (uint8_t i = 0; i < N; ++i)
        {
            if (active_[i] && (motor < 0 || motor == i))
            {
                const float p = motor_power(i, scaled_current(i, eta));
                if (!std::isfinite(p))
                    return p;
                power += budget_power(p);
            }
        }
        return power;
    }

    Result invalid_input()
    {
        for (uint8_t i = 0; i < N; ++i)
            current_[i] = 0.0f;
        result_ = {};
        result_.eta = result_.rho = 0.0f;
        result_.limited = true;
        result_.valid = result_.feasible = false;
        return result_;
    }

    Result baseline_failure(float baseline)
    {
        result_.eta = result_.rho = 0.0f;
        result_.limited = true;
        result_.feasible = false;
        result_.power_limited = baseline;
        return result_;
    }

    bool prepare(const float *I, const float *V, const float *K, const float *other,
                 float correction, float max_power, const bool *active)
    {
        reset();
        if (I == nullptr || V == nullptr || K == nullptr ||
            !std::isfinite(config_.current_step_a) || config_.current_step_a < 0.0f ||
            !std::isfinite(correction) || correction < 0.0f || !std::isfinite(max_power) ||
            max_power < 0.0f)
        {
            invalid_input();
            return false;
        }
        for (uint8_t k = 0; k < 6; ++k)
        {
            if (!std::isfinite(K[k]))
            {
                invalid_input();
                return false;
            }
        }

        if (K[4] < 0.0f)
        {
            invalid_input();
            return false;
        }
        correction_w_ = correction;
        power_max_w_ = max_power;
        for (uint8_t i = 0; i < N; ++i)
        {
            active_[i] = active == nullptr || active[i];
            if (!active_[i])
                continue;
            const float total = I[i] + (other != nullptr ? other[i] : 0.0f);
            if (!std::isfinite(total) || !std::isfinite(V[i]))
            {
                invalid_input();
                return false;
            }
            this->SetMotorParameters(i, total, V[i], K, config_.absolute_speed_loss);
            if (!std::isfinite(this->GetPower(i)))
            {
                invalid_input();
                return false;
            }
        }
        result_.power_total = power_at(1.0f);
        if (!std::isfinite(result_.power_total) || !std::isfinite(power_at(0.0f)))
        {
            invalid_input();
            return false;
        }
        return true;
    }

    float find_scale(float budget, int motor = -1) const
    {
        float a = 0.0f, b = 0.0f, c = motor < 0 ? correction_w_ : 0.0f;
        for (uint8_t i = 0; i < N; ++i)
        {
            if (!active_[i] || (motor >= 0 && motor != i))
                continue;
            const auto &p = this->params_[i];
            a += p.coeffs[4] * p.current * p.current;
            b += (p.coeffs[1] + p.coeffs[3] * p.velocity) * p.current;
            c += motor_power(i, 0.0f);
        }

        c -= budget;
        float candidate = 0.0f;
        if (a > 1.0e-8f)
        {
            const float discriminant = b * b - 4.0f * a * c;
            if (std::isfinite(discriminant) && discriminant >= 0.0f)
            {
                const float root = std::sqrt(discriminant);

                candidate = b >= 0.0f && b + root > 0.0f ? -2.0f * c / (b + root)
                                                         : (-b + root) / (2.0f * a);
            }
        }
        else if (b > 0.0f)
        {
            candidate = -c / b;
        }
        float low = 0.0f, high = 1.0f;
        if (std::isfinite(candidate) && candidate > 0.0f && candidate < 1.0f)
        {

            if (power_at(candidate, motor) <= budget)
                return candidate;
            else
                high = candidate;
        }
        for (uint8_t iteration = 0; iteration < 16; ++iteration)
        {
            const float mid = low + (high - low) * 0.5f;
            if (power_at(mid, motor) <= budget)
                low = mid;
            else
                high = mid;
        }
        return low;
    }

    Result finish()
    {
        float power = correction_w_;
        for (uint8_t i = 0; i < N; ++i)
        {
            if (!active_[i])
                continue;
            const float p = motor_power(i, current_[i]);
            if (!std::isfinite(current_[i]) || !std::isfinite(p))
                return invalid_input();
            power += budget_power(p);
        }
        if (!std::isfinite(power))
            return invalid_input();
        result_.power_limited = power;
        result_.feasible = power <= power_max_w_ + POWER_TOLERANCE_W;
        if (!result_.feasible)
        {
            for (uint8_t i = 0; i < N; ++i)
                current_[i] = 0.0f;
            result_.eta = result_.rho = 0.0f;
            result_.limited = true;
            result_.power_limited = power_at(0.0f);
        }
        return result_;
    }
};

/** @brief 用滑动窗口累计能量，用于功率策略的缓冲估算。 */
class EnergyRing
{
  public:
    EnergyRing(float abundanceline, float povertyline, float pmin_restriction)
        : AbundanceLine(abundanceline), PovertyLine(povertyline),
          Pmin_restriction(pmin_restriction), PowerMax(0.0f)
    {
    }

    void update(float AbundanceOut, float PovertyOut, float P_referee, float CurrentEnergy,
                bool isShift, bool isPower)
    {
        if (isPower)
        {
            PowerMax = 0.8f * P_referee;
        }
        else
        {
            float PowerMax_abundance = P_referee - AbundanceOut;
            float PowerMax_poverty = P_referee - PovertyOut;

            if (CurrentEnergy < PovertyLine)
            {
                PowerMax = 0.8f * P_referee;
            }
            else if (isShift)
            {
                PowerMax = PowerMax_poverty;
            }
            else
            {
                if (CurrentEnergy >= AbundanceLine)
                {
                    PowerMax = PowerMax_abundance;
                }
                else
                {
                    PowerMax = P_referee;
                }
            }

            Pmin_restriction = 0.7f * P_referee;
            if (PowerMax < Pmin_restriction)
                PowerMax = Pmin_restriction;
        }
    }

    float GetPowerMax() const { return PowerMax; }
    float GetAbundanceLine() const { return AbundanceLine; }
    float GetPovertyLine() const { return PovertyLine; }

  private:
    float AbundanceLine, PovertyLine, Pmin_restriction, PowerMax;
};

/** @brief 结合裁判功率上限和缓冲量计算允许功率。 */
class PowerControlStrategy
{
  public:
    struct RefereeConfig
    {
        float fallback_w{60.0f};
        float margin_w{5.0f};
        uint32_t timeout_ms{500U};
        float buffer_low_j{20.0f};
        float buffer_recover_j{30.0f};
        float low_buffer_scale{0.8f};
    };
    enum class RefereeSource : uint8_t
    {
        Fallback = 0,
        Normal,
        LowBuffer,
        Disabled,
        InvalidConfig,
    };

    bool ConfigureReferee(const RefereeConfig &config)
    {
        referee_config_ = config;
        referee_config_valid_ = std::isfinite(config.fallback_w) && config.fallback_w > 0.0f &&
                                std::isfinite(config.margin_w) && config.margin_w >= 0.0f &&
                                config.timeout_ms > 0U && std::isfinite(config.buffer_low_j) &&
                                config.buffer_low_j >= 0.0f &&
                                std::isfinite(config.buffer_recover_j) &&
                                config.buffer_recover_j > config.buffer_low_j &&
                                std::isfinite(config.low_buffer_scale) &&
                                config.low_buffer_scale > 0.0f && config.low_buffer_scale <= 1.0f;
        status_seen_ = buffer_seen_ = false;
        status_count_ = buffer_count_ = status_ms_ = buffer_ms_ = 0U;
        low_buffer_ = true;
        source_ = referee_config_valid_ ? RefereeSource::Fallback : RefereeSource::InvalidConfig;
        input_limit = referee_config_valid_ ? config.fallback_w : 0.0f;
        input_energy = 0.0f;
        return referee_config_valid_;
    }

    void UpdateReferee(uint32_t now_ms, bool referee_online, uint32_t status_count,
                       uint32_t buffer_count, float referee_limit_w, float buffer_j,
                       bool chassis_enabled)
    {
        update_counter(now_ms, status_count, status_count_, status_ms_, status_seen_);
        update_counter(now_ms, buffer_count, buffer_count_, buffer_ms_, buffer_seen_);
        if (!referee_config_valid_)
        {
            source_ = RefereeSource::InvalidConfig;
            input_limit = input_energy = 0.0f;
            return;
        }
        const bool status_valid = referee_online && status_seen_ &&
                                  now_ms - status_ms_ < referee_config_.timeout_ms &&
                                  std::isfinite(referee_limit_w) && referee_limit_w >= 0.0f;
        const bool buffer_valid = referee_online && buffer_seen_ &&
                                  now_ms - buffer_ms_ < referee_config_.timeout_ms &&
                                  std::isfinite(buffer_j) && buffer_j >= 0.0f;
        input_energy = buffer_valid ? buffer_j : 0.0f;
        if (!status_valid)
        {

            input_limit = referee_config_.fallback_w;
            source_ = RefereeSource::Fallback;
            low_buffer_ = true;
        }
        else if (!chassis_enabled || referee_limit_w == 0.0f)
        {
            input_limit = 0.0f;
            source_ = RefereeSource::Disabled;
        }
        else
        {
            if (!buffer_valid || buffer_j < referee_config_.buffer_low_j)
                low_buffer_ = true;
            else if (buffer_j >= referee_config_.buffer_recover_j)
                low_buffer_ = false;
            input_limit = std::fmax(0.0f, referee_limit_w - referee_config_.margin_w);
            if (low_buffer_)
                input_limit *= referee_config_.low_buffer_scale;
            source_ = low_buffer_ ? RefereeSource::LowBuffer : RefereeSource::Normal;
        }
    }

    RefereeSource GetRefereeSource() const { return source_; }
    uint32_t GetStatusAge(uint32_t now_ms) const
    {
        return status_seen_ ? now_ms - status_ms_ : UINT32_MAX;
    }
    uint32_t GetBufferAge(uint32_t now_ms) const
    {
        return buffer_seen_ ? now_ms - buffer_ms_ : UINT32_MAX;
    }

    PowerControlStrategy(float abundanceline)
        : last_valid_limit(60.0f), input_limit(60.0f), input_energy(0.0f),
          AbundanceLine(abundanceline)
    {
    }

    void Update(bool isSupercapOnline, bool isRefereeOnline, float referee_limit,
                float referee_buffer, float supercap_energy)
    {
        if (isRefereeOnline)
            last_valid_limit = referee_limit;

        if (isSupercapOnline && !isRefereeOnline)
        {
            input_limit = last_valid_limit;
            clampFallbackLimit();
            input_energy = supercap_energy;
        }
        else if (!isSupercapOnline && isRefereeOnline)
        {
            if (referee_buffer > 55.0f)
            {
                input_limit = last_valid_limit;
                input_energy = AbundanceLine;
            }
            else
            {
                input_limit = last_valid_limit;
                input_energy = 0.0f;
            }
        }
        else if (!isSupercapOnline && !isRefereeOnline)
        {
            input_limit = last_valid_limit;
            clampFallbackLimit();
            input_energy = 0.0f;
        }
        else
        {
            input_limit = referee_limit;
            input_energy = supercap_energy;
        }
    }

    float GetInputLimit() const { return input_limit; }
    float GetInputEnergy() const { return input_energy; }

  private:
    static void update_counter(uint32_t now, uint32_t value, uint32_t &previous,
                               uint32_t &timestamp, bool &seen)
    {
        if (value == previous)
            return;

        const bool reset_to_zero = value == 0U && value - previous > UINT32_MAX / 2U;
        seen = !reset_to_zero;
        previous = value;
        timestamp = now;
    }

    RefereeConfig referee_config_{};
    RefereeSource source_{RefereeSource::Fallback};
    bool referee_config_valid_{true};
    bool status_seen_{false}, buffer_seen_{false}, low_buffer_{true};
    uint32_t status_count_{0U}, buffer_count_{0U}, status_ms_{0U}, buffer_ms_{0U};

    void clampFallbackLimit()
    {
        if (input_limit < 60.0f)
            input_limit = 60.0f;
    }

    float last_valid_limit, input_limit, input_energy, AbundanceLine;
};

} // namespace Omni
} // namespace ALG
