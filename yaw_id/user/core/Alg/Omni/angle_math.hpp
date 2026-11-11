#pragma once

#include <cmath>
#include <limits>

namespace ALG::Omni::AngleMath
{

inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float two_pi = 2.0f * pi;

inline float symmetric(float value, float half_period, float period)
{
    if (value >= -half_period && value <= half_period)
        return value;
    if (!std::isfinite(value))
        return std::numeric_limits<float>::quiet_NaN();
    if (value > half_period && value <= half_period + period)
        return value - period;
    if (value < -half_period && value >= -half_period - period)
        return value + period;
    value = std::fmod(value, period);
    if (value > half_period)
        value -= period;
    else if (value < -half_period)
        value += period;
    return value;
}

inline float positive_radians(float value)
{
    if (value >= 0.0f && value < two_pi)
        return value;
    if (!std::isfinite(value))
        return std::numeric_limits<float>::quiet_NaN();
    if (value >= two_pi && value < 2.0f * two_pi)
        return value - two_pi;
    if (value < 0.0f && value >= -two_pi)
        value += two_pi;
    else
    {
        value = std::fmod(value, two_pi);
        if (value < 0.0f)
            value += two_pi;
    }

    return value < two_pi ? value : 0.0f;
}

inline float error_radians(float value)
{
    value = symmetric(value, pi, two_pi);
    return value >= pi ? -pi : value;
}

/** @brief 缓存角度余弦，减少相邻控制周期内的重复计算。 */
class CosineCache
{
  public:
    float get(float angle)
    {
        if (angle != angle_)
        {
            cosine_ = std::cos(angle);
            angle_ = angle;
        }
        return cosine_;
    }

  private:
    float angle_{std::numeric_limits<float>::quiet_NaN()};
    float cosine_{1.0f};
};

} // namespace ALG::Omni::AngleMath
