#pragma once

#include "user/core/Alg/Omni/kinematics_base.hpp"
#include <cstdint>

namespace ALG
{
namespace Omni
{

/** @brief 平面底盘里程计，速度单位 m/s 与 rad/s。 */
struct ChassisOdometry
{
    float vx = 0.0f;
    float vy = 0.0f;
    float vw = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

/** @brief 多种轮式底盘共用的正、逆运动学接口。 */
class IKinematicsSolver
{
  public:
    virtual ~IKinematicsSolver() = default;

    virtual uint8_t get_wheel_count() const = 0;

    virtual void inverse_kinematics(float vx, float vy, float vw, WheelOutput *out) = 0;

    virtual void forward_kinematics(const float *wheel_speeds, ChassisOdometry &odom) = 0;

    virtual const char *get_solver_name() const = 0;
};

} // namespace Omni
} // namespace ALG
