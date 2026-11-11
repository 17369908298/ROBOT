#pragma once

#include <cmath>

namespace ALG::Omni
{

/** @brief 根据轮速误差限制底盘惯性前馈力矩，输入输出单位为 N*m。 */
inline float guard_chassis_inertial_ff(float torque_nm, float speed_error_radps,
                                       float fade_error_radps)
{
    if (!std::isfinite(torque_nm) || !std::isfinite(speed_error_radps) ||
        !std::isfinite(fade_error_radps) || fade_error_radps <= 0.0f)
    {
        return 0.0f;
    }

    if (torque_nm == 0.0f || speed_error_radps == 0.0f ||
        (torque_nm > 0.0f) != (speed_error_radps > 0.0f))
    {
        return 0.0f;
    }

    const float magnitude = std::fabs(speed_error_radps);
    const float weight = magnitude < fade_error_radps ? magnitude / fade_error_radps : 1.0f;
    return torque_nm * weight;
}

} // namespace ALG::Omni
