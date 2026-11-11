#pragma once
#include "RtosTask/shooter_config.hpp"
#include "RtosTask/gimbal_config.hpp"

// Keep these navigation-frame settings aligned with CHASSIS/RtosTask/chassis_config.hpp.
namespace AppConfig::NavHostComm
{
inline constexpr float gimbal_yaw_zero_rad = 4.688f;
inline constexpr float gimbal_yaw_sign = 1.0f;
}
namespace nav_cfg = AppConfig::NavHostComm;
