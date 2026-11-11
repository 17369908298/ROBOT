#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>

namespace ALG::Omni
{

/** @brief 根据裁判热量和摩擦轮状态限制射速并检测发弹。 */
class ShooterHeatLimiter
{
  public:
    static constexpr uint8_t kMaxWindowSize = 200;

    struct Config
    {
        uint8_t window_size = 20;
        uint32_t shot_cooldown_ms = 80;
        uint32_t startup_ignore_ms = 1500;

        float integral_threshold = 8.0f;
        float noise_threshold = 1.5f;
        float ref_start_threshold = 1000.0f;
        float wheel_ready_speed = 3000.0f;

        float heat_limit = 40.0f;
        float cooling_rate = 12.0f;
        float heat_per_shot = 10.0f;
        float ref_fire_rate = 8.0f;

        float free_margin_ratio = 0.80f;
        float stop_margin_ratio = 0.30f;
    };

    struct WheelFeedback
    {
        float left_current = 0.0f;
        float right_current = 0.0f;
        float left_speed = 0.0f;
        float right_speed = 0.0f;
    };

    struct Input
    {
        uint32_t now_ms = 0;
        float dt_s = 0.0f;
        float ref_speed = 0.0f;
        WheelFeedback feedback{};

        float heat_limit = 0.0f;
        float cooling_rate = 0.0f;
        float ref_fire_rate = 0.0f;

        bool use_external_heat = false;
        float external_heat = 0.0f;
    };

    struct Result
    {
        bool shot_detected = false;
        bool allow_shoot = false;
        bool detect_ready = false;

        float estimated_heat = 0.0f;
        float used_heat = 0.0f;
        float heat_limit = 0.0f;
        float heat_margin = 0.0f;
        float allowed_fire_rate = 0.0f;
        float current_integral = 0.0f;

        uint32_t bullet_count = 0;
    };

    ShooterHeatLimiter()
    {
        set_config(Config{});
        reset();
    }

    explicit ShooterHeatLimiter(const Config &config)
    {
        set_config(config);
        reset();
    }

    void set_config(const Config &config)
    {
        config_ = sanitize_config(config);
        clear_window();
    }

    Config get_config() const { return config_; }

    void reset()
    {
        clear_window();
        last_shot_time_ms_ = 0;
        startup_start_ms_ = 0;
        last_ref_speed_ = 0.0f;
        estimated_heat_ = 0.0f;
        bullet_count_ = 0;
        result_ = Result{};
        result_.heat_limit = config_.heat_limit;
    }

    Result update(const Input &input)
    {
        const float heat_limit = choose_positive(input.heat_limit, config_.heat_limit);
        const float cooling_rate = choose_positive(input.cooling_rate, config_.cooling_rate);
        const float ref_fire_rate = choose_positive(input.ref_fire_rate, config_.ref_fire_rate);
        const float dt_s = input.dt_s > 0.0f ? input.dt_s : 0.0f;

        const bool shot = detect_shot(input.now_ms, input.ref_speed, input.feedback);
        if (shot)
        {
            estimated_heat_ += config_.heat_per_shot;
            ++bullet_count_;
        }

        estimated_heat_ -= cooling_rate * dt_s;
        if (estimated_heat_ < 0.0f)
        {
            estimated_heat_ = 0.0f;
        }

        const float used_heat =
            input.use_external_heat ? std::max(0.0f, input.external_heat) : estimated_heat_;
        const float heat_margin = heat_limit - used_heat;
        const float allowed_fire_rate =
            compute_allowed_fire_rate(heat_margin, heat_limit, cooling_rate, ref_fire_rate);

        result_.shot_detected = shot;
        result_.allow_shoot = allowed_fire_rate > 0.0f && heat_margin > 0.0f;
        result_.estimated_heat = estimated_heat_;
        result_.used_heat = used_heat;
        result_.heat_limit = heat_limit;
        result_.heat_margin = heat_margin;
        result_.allowed_fire_rate = allowed_fire_rate;
        result_.current_integral = window_sum_;
        result_.bullet_count = bullet_count_;
        return result_;
    }

    Result update(uint32_t now_ms, float dt_s, float ref_speed, float left_current,
                  float right_current, float left_speed, float right_speed, float heat_limit,
                  float cooling_rate, float ref_fire_rate)
    {
        Input input{};
        input.now_ms = now_ms;
        input.dt_s = dt_s;
        input.ref_speed = ref_speed;
        input.feedback.left_current = left_current;
        input.feedback.right_current = right_current;
        input.feedback.left_speed = left_speed;
        input.feedback.right_speed = right_speed;
        input.heat_limit = heat_limit;
        input.cooling_rate = cooling_rate;
        input.ref_fire_rate = ref_fire_rate;
        return update(input);
    }

    void set_estimated_heat(float heat) { estimated_heat_ = std::max(0.0f, heat); }

    const Result &result() const { return result_; }
    bool shot_detected() const { return result_.shot_detected; }
    bool allow_shoot() const { return result_.allow_shoot; }
    bool detect_ready() const { return result_.detect_ready; }
    float allowed_fire_rate() const { return result_.allowed_fire_rate; }
    float estimated_heat() const { return estimated_heat_; }
    float used_heat() const { return result_.used_heat; }
    float heat_margin() const { return result_.heat_margin; }
    float current_integral() const { return window_sum_; }
    uint32_t bullet_count() const { return bullet_count_; }

  private:
    static Config sanitize_config(Config config)
    {
        if (config.window_size == 0)
        {
            config.window_size = 1;
        }
        else if (config.window_size > kMaxWindowSize)
        {
            config.window_size = kMaxWindowSize;
        }

        config.integral_threshold = std::max(0.0f, config.integral_threshold);
        config.noise_threshold = std::max(0.0f, config.noise_threshold);
        config.ref_start_threshold = std::max(0.0f, config.ref_start_threshold);
        config.wheel_ready_speed = std::max(0.0f, config.wheel_ready_speed);
        config.heat_limit = std::max(0.0f, config.heat_limit);
        config.cooling_rate = std::max(0.0f, config.cooling_rate);
        config.heat_per_shot = std::max(0.001f, config.heat_per_shot);
        config.ref_fire_rate = std::max(0.0f, config.ref_fire_rate);

        config.stop_margin_ratio = clamp(config.stop_margin_ratio, 0.0f, 1.0f);
        config.free_margin_ratio = clamp(config.free_margin_ratio, 0.0f, 1.0f);
        if (config.free_margin_ratio < config.stop_margin_ratio)
        {
            config.free_margin_ratio = config.stop_margin_ratio;
        }
        return config;
    }

    bool detect_shot(uint32_t now_ms, float ref_speed, const WheelFeedback &feedback)
    {
        result_.shot_detected = false;
        result_.detect_ready = false;

        if ((now_ms - last_shot_time_ms_) < config_.shot_cooldown_ms)
        {
            return false;
        }

        const float ref_abs = absf(ref_speed);
        if (last_ref_speed_ < 100.0f && ref_abs > config_.ref_start_threshold)
        {
            startup_start_ms_ = now_ms;
        }
        last_ref_speed_ = ref_abs;

        if (ref_abs < config_.ref_start_threshold ||
            (now_ms - startup_start_ms_) < config_.startup_ignore_ms)
        {
            clear_window();
            return false;
        }

        float current_input = 0.0f;
        const float left_speed = absf(feedback.left_speed);
        const float right_speed = absf(feedback.right_speed);
        if (left_speed > config_.wheel_ready_speed && right_speed > config_.wheel_ready_speed)
        {
            result_.detect_ready = true;
            const float total_current = absf(feedback.left_current) + absf(feedback.right_current);
            current_input = std::max(0.0f, total_current - config_.noise_threshold);
        }

        window_sum_ -= window_[window_index_];
        window_[window_index_] = current_input;
        window_sum_ += current_input;

        ++window_index_;
        if (window_index_ >= config_.window_size)
        {
            window_index_ = 0;
        }

        if (window_sum_ > config_.integral_threshold)
        {
            last_shot_time_ms_ = now_ms;
            clear_window();
            return true;
        }

        return false;
    }

    float compute_allowed_fire_rate(float heat_margin, float heat_limit, float cooling_rate,
                                    float ref_fire_rate) const
    {
        if (heat_limit <= 0.0f || ref_fire_rate <= 0.0f)
        {
            return 0.0f;
        }

        const float free_margin = config_.free_margin_ratio * heat_limit;
        const float stop_margin = config_.stop_margin_ratio * heat_limit;
        if (heat_margin > free_margin)
        {
            return ref_fire_rate;
        }
        if (heat_margin < stop_margin)
        {
            return 0.0f;
        }

        const float span = free_margin - stop_margin;
        if (span <= 0.0001f)
        {
            return 0.0f;
        }

        const float ref_weight = (heat_margin - stop_margin) / span;
        const float cooling_weight = (free_margin - heat_margin) / span;
        const float sustainable_fire_rate = cooling_rate / config_.heat_per_shot;
        const float limited_rate =
            ref_fire_rate * ref_weight + sustainable_fire_rate * cooling_weight;
        return clamp(limited_rate, 0.0f, ref_fire_rate);
    }

    void clear_window()
    {
        window_.fill(0.0f);
        window_index_ = 0;
        window_sum_ = 0.0f;
    }

    static float choose_positive(float value, float fallback)
    {
        return value > 0.0f ? value : fallback;
    }

    static float absf(float value) { return value < 0.0f ? -value : value; }

    static float clamp(float value, float low, float high)
    {
        return std::max(low, std::min(value, high));
    }

    Config config_{};
    Result result_{};

    std::array<float, kMaxWindowSize> window_{};
    uint8_t window_index_{0};
    float window_sum_{0.0f};

    uint32_t last_shot_time_ms_{0};
    uint32_t startup_start_ms_{0};
    float last_ref_speed_{0.0f};

    float estimated_heat_{0.0f};
    uint32_t bullet_count_{0};
};

} // namespace ALG::Omni
