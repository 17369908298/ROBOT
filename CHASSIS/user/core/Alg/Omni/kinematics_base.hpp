#pragma once

#include <cstdint>

namespace ALG
{
namespace Omni
{

/** @brief 单轮运动学结果，速度以 rpm 表示，方向角以 rad 表示。 */
struct WheelOutput
{
    float speed_rpm = 0.0f;
    float angle_enc = 0.0f;
};

/** @brief 底盘运动学接口，统一正向和逆向速度换算。 */
class IKinematics
{
  public:
    virtual ~IKinematics() = default;

    virtual void inverse_kinematics(float vx, float vy, float vw, WheelOutput out[4]) = 0;
};

} // namespace Omni
} // namespace ALG
