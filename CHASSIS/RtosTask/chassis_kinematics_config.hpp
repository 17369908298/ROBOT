#pragma once

#include "RtosTask/robot_config.hpp"
#include "user/core/Alg/Omni/kinematics_solvers.hpp"

namespace App::Chassis
{

// 电机方向符号，顺序为左前、右前、左后、右后。
inline constexpr float WHEEL_SIGN[] = {
    chassis_cfg::chassis.wheel.sign.fl,
    chassis_cfg::chassis.wheel.sign.fr,
    chassis_cfg::chassis.wheel.sign.bl,
    chassis_cfg::chassis.wheel.sign.br,
};

/**
 * @brief 从底盘几何参数生成正、逆运动学共用的求解器配置。
 * @return 运动学配置，车轮按左前、右前、左后、右后排列。
 */
inline ALG::Omni::OmniSolver::Config make_kinematics_config()
{
    const auto &geometry = chassis_cfg::chassis.geometry;
    return {
        geometry.chassis_radius_m,
        geometry.wheel_radius_m,
        {geometry.azimuth.fl_rad, geometry.azimuth.fr_rad, geometry.azimuth.bl_rad, geometry.azimuth.br_rad},
        {geometry.direction.fl_rad, geometry.direction.fr_rad, geometry.direction.bl_rad, geometry.direction.br_rad},
        {1.0f, 1.0f, 1.0f, 1.0f},
    };
}

} // namespace App::Chassis
