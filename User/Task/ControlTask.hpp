#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/Alg/Filter/Filter.hpp"
#include "../User/core/Alg/Feedforward/Feedforward.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/Alg/ADRC/adrc.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/DM/DmMotor.hpp"
#include "../User/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../User/Task/SerialTask.hpp"
#include "../User/core/APP/Heat_Detector/Heat_Control_Private.hpp"
#include "../User/core/Alg/VMC/VMC.hpp"
#include "../User/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"

namespace GimbalConfig
{
// Pitch1 single-axis VMC parameters.
constexpr float kPitch1_VMC_Kp = 45.0f; // Virtual torsion spring stiffness (Nm/rad)
constexpr float kPitch1_VMC_Kd = 1.5f;  // Virtual rotary damping (Nm/(rad/s))
constexpr float kPitch1GravityOffsetDeg = 0.0f;

// Pitch2 transform composite VMC parameters.
constexpr float kTransformPitch2LevelKp = 2.0f;
constexpr float kTransformPitch2LevelKd = 0.2f;
constexpr float kTransformPitch2VirtualWallAngle = 10.0f; // deg
constexpr float kTransformPitch2WallStiffness = 10.0f;

// Transform landing threshold configuration.
constexpr float kPitch1TransformSafeDropDeg = 20.0f;
constexpr float kPitch1TransformLandingDeg = 7.0f;
constexpr float kPitch2HysteresisHighDeg = 17.0f;
constexpr float kPitch2HysteresisLowDeg = 13.0f;
constexpr float kPitch2LandingDamper = 0.6f;
}  // namespace GimbalConfig

extern bool shoot;
extern bool is_standup_complete;

/**
 * @brief Pitch2 底层控制模式。
 */
typedef enum {
    GIMBAL_MODE_ANGLE = 0,   // 角度模式：STM32 外环输出目标速度，达妙底层速度闭环
    GIMBAL_MODE_VELOCITY,    // 速度模式：STM32 ADRC 输出目标扭矩，达妙底层纯扭矩输出
} Gimbal_Control_Mode_e;

/**
 * @brief 云台控制目标，SetTarget_Gimbal() 负责更新这些期望值。
 */
typedef struct
{
    Gimbal_Control_Mode_e current_mode; // 当前 Pitch2 控制模式
    float yaw_vel;                      // Yaw 目标速度，单位 rad/s
    float yaw_angle;                    // Yaw 目标角度，单位 rad
    float pitch1;                       // Pitch1 目标角度，单位 rad
    float pitch2_vel;                   // Pitch2 目标速度，单位 rad/s
    float pitch2_angle;                 // Pitch2 目标角度，单位 rad
    float pitch1_vel;                   // Pitch1 目标速度，预留
    float dial;                         // 拨盘目标，预留
    float surgewheel[2];                // 摩擦轮/拨弹相关目标，预留
} GimbalTarget_t;

/**
 * @brief 云台控制输出，MotorTask 只读取这里的数据并下发到电机。
 */
typedef struct
{
    float yaw_vel;      // Yaw 目标速度 V_des
    float yaw_torq;     // Yaw 目标扭矩 T_ff
    float pitch1_vel;   // Pitch1 目标速度 V_des
    float pitch1_torq;  // Pitch1 前馈/目标扭矩 T_ff
    float pitch2_vel;   // Pitch2 目标速度 V_des
    float pitch2_torq;  // Pitch2 前馈/目标扭矩 T_ff
} GimbalOutput_t;

extern BSP::Motor::DM::J4310<2> MotorJ4310;
extern BSP::Motor::DM::J4340<1> MotorJ4340;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;

extern GimbalOutput_t out;
extern GimbalTarget_t cmd;

extern Gimbal_FSM gimbal_fsm;

extern ALG::ADRC::FirstLADRC yaw_adrc;
extern ALG::ADRC::FirstLADRC pitch2_adrc;
extern ALG::PID::PID yaw_transform_angle_pid;
extern ALG::ADRC::FirstLADRC yaw_adrc_try;
extern Alg::Feedforward::GimbalFullCompensation gimbal_yaw;
extern ALG::PID::PID yaw_encoder_transform_pid;

#endif
