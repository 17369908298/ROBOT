#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ALG::Omni
{

/** @brief 按双轴位置误差、连续确认时间及退出滞回判定跟踪状态。 */
class PositionTrackingGate
{
  public:
    enum class State : std::uint8_t
    {
        BLOCKED = 0U,
        QUALIFYING,
        READY,
    };

    struct Config
    {
        float yaw_enter_rad{}, yaw_exit_rad{};
        float pitch_enter_rad{}, pitch_exit_rad{};
        std::uint32_t stable_hold_ms{}, sample_timeout_ms{};
    };

    struct Output
    {
        State state{State::BLOCKED};
        bool valid{};
        std::uint32_t stable_elapsed_ms{};
    };

    void configure(const Config &config)
    {
        config_ = config;
        config_valid_ = std::isfinite(config.yaw_enter_rad) && std::isfinite(config.yaw_exit_rad) &&
                        std::isfinite(config.pitch_enter_rad) && std::isfinite(config.pitch_exit_rad) &&
                        config.yaw_enter_rad > 0.0f && config.yaw_exit_rad > config.yaw_enter_rad &&
                        config.pitch_enter_rad > 0.0f && config.pitch_exit_rad > config.pitch_enter_rad &&
                        config.stable_hold_ms > 0U && config.sample_timeout_ms > 0U;
        reset();
    }

    void reset()
    {
        output_ = {};
        sample_ready_ = false;
        stable_since_ms_ = 0U;
        last_sample_ms_ = 0U;
    }

    Output update(float yaw_error_rad, float pitch_error_rad, bool eligible, std::uint32_t now_ms)
    {
        if (!config_valid_ || !eligible || !std::isfinite(yaw_error_rad) || !std::isfinite(pitch_error_rad))
        {
            reset();
            return output_;
        }

        // 采样中断后重新确认，不能把没有观测的时间累计为稳定时间。
        if (sample_ready_ && now_ms - last_sample_ms_ > config_.sample_timeout_ms)
            reset();
        last_sample_ms_ = now_ms;
        sample_ready_ = true;
        output_.valid = true;

        const float yaw_error = std::fabs(yaw_error_rad);
        const float pitch_error = std::fabs(pitch_error_rad);
        if (output_.state == State::READY)
        {
            if (yaw_error >= config_.yaw_exit_rad || pitch_error >= config_.pitch_exit_rad)
            {
                output_.state = State::BLOCKED;
                output_.stable_elapsed_ms = 0U;
            }
            return output_;
        }

        if (yaw_error > config_.yaw_enter_rad || pitch_error > config_.pitch_enter_rad)
        {
            output_.state = State::BLOCKED;
            output_.stable_elapsed_ms = 0U;
            return output_;
        }

        if (output_.state != State::QUALIFYING)
        {
            stable_since_ms_ = now_ms;
            output_.state = State::QUALIFYING;
        }
        output_.stable_elapsed_ms = std::min(now_ms - stable_since_ms_, config_.stable_hold_ms);
        if (output_.stable_elapsed_ms >= config_.stable_hold_ms)
            output_.state = State::READY;
        return output_;
    }

  private:
    Config config_{};
    Output output_{};
    bool config_valid_{false}, sample_ready_{false};
    std::uint32_t stable_since_ms_{}, last_sample_ms_{};
};

} // namespace ALG::Omni
