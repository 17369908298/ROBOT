#pragma once

#include <cmath>
#include <cstdint>

namespace ALG::Omni
{

/** @brief 换算电机电流、原始命令和力矩，按配置应用减速比与效率。 */
class TorqueCurrentConverter final
{
  public:
    struct Config
    {
        float torque_constant_nm_per_a = 0.0f;
        float current_max_a = 0.0f;
        float command_max = 0.0f;
        float command_limit = 0.0f;

        float output_reduction = 1.0f;

        float transmission_efficiency = 1.0f;
    };

    constexpr TorqueCurrentConverter() = default;
    constexpr explicit TorqueCurrentConverter(const Config &config) : config_(config) {}

    void configure(const Config &config) { config_ = config; }

    float torque_to_current_A(float torque_nm) const
    {
        if (!valid() || !std::isfinite(torque_nm))
        {
            return 0.0f;
        }
        return torque_nm / config_.torque_constant_nm_per_a;
    }

    float current_to_torque_Nm(float current_a) const
    {
        if (!valid() || !std::isfinite(current_a))
        {
            return 0.0f;
        }
        return current_a * config_.torque_constant_nm_per_a;
    }

    float current_to_output_torque_Nm(float current_a) const
    {
        if (!output_model_valid() || !std::isfinite(current_a))
        {
            return 0.0f;
        }
        return current_to_torque_Nm(current_a) * config_.output_reduction *
               config_.transmission_efficiency;
    }

    float output_torque_to_current_A(float torque_nm) const
    {
        if (!output_model_valid() || !std::isfinite(torque_nm))
        {
            return 0.0f;
        }
        return torque_nm / (config_.torque_constant_nm_per_a * config_.output_reduction *
                            config_.transmission_efficiency);
    }

    std::int16_t current_to_command(float current_a) const
    {
        if (!valid() || !std::isfinite(current_a))
        {
            return 0;
        }

        const float raw = current_a / config_.current_max_a * config_.command_max;
        return clamp_command(raw);
    }

    std::int16_t torque_to_command(float torque_nm) const
    {
        return current_to_command(torque_to_current_A(torque_nm));
    }

    std::int16_t output_torque_to_command(float torque_nm) const
    {
        return current_to_command(output_torque_to_current_A(torque_nm));
    }

    float command_to_current_A(std::int16_t command) const
    {
        if (!valid())
        {
            return 0.0f;
        }
        return static_cast<float>(command) / config_.command_max * config_.current_max_a;
    }

    float command_to_torque_Nm(std::int16_t command) const
    {
        return current_to_torque_Nm(command_to_current_A(command));
    }

    float command_to_output_torque_Nm(std::int16_t command) const
    {
        return current_to_output_torque_Nm(command_to_current_A(command));
    }

    float command_limit() const
    {
        if (!valid())
        {
            return 0.0f;
        }
        return effective_command_limit();
    }

    float current_limit_A() const
    {
        if (!valid())
        {
            return 0.0f;
        }
        return effective_command_limit() / config_.command_max * config_.current_max_a;
    }

    float torque_limit_Nm() const { return current_to_torque_Nm(current_limit_A()); }

    float output_torque_limit_Nm() const { return current_to_output_torque_Nm(current_limit_A()); }

    float output_torque_constant_nm_per_a() const
    {
        if (!output_model_valid())
        {
            return 0.0f;
        }
        return config_.torque_constant_nm_per_a * config_.output_reduction *
               config_.transmission_efficiency;
    }

    const Config &config() const { return config_; }

  private:
    bool valid() const
    {
        return std::isfinite(config_.torque_constant_nm_per_a) &&
               config_.torque_constant_nm_per_a > 0.0f && std::isfinite(config_.current_max_a) &&
               config_.current_max_a > 0.0f && std::isfinite(config_.command_max) &&
               config_.command_max > 0.0f;
    }

    bool output_model_valid() const
    {
        return valid() && std::isfinite(config_.output_reduction) &&
               config_.output_reduction > 0.0f && std::isfinite(config_.transmission_efficiency) &&
               config_.transmission_efficiency > 0.0f;
    }

    float effective_command_limit() const
    {
        const float physical_limit = config_.command_max;
        if (!std::isfinite(config_.command_limit) || config_.command_limit <= 0.0f)
        {
            return physical_limit;
        }
        return config_.command_limit < physical_limit ? config_.command_limit : physical_limit;
    }

    std::int16_t clamp_command(float raw) const
    {
        const float limit = effective_command_limit();
        if (!std::isfinite(raw))
        {
            return 0;
        }
        if (raw > limit)
        {
            raw = limit;
        }
        else if (raw < -limit)
        {
            raw = -limit;
        }
        return static_cast<std::int16_t>(std::lround(raw));
    }

    Config config_{};
};

} // namespace ALG::Omni
