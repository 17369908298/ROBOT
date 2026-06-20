#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/IMU/HI12_imu.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp"
#include "../user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../User/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include "../User/core/Alg/PowerControl/PowerControl.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"

/**
 * @brief 底盘调车参数集中区。
 *
 * 这里保留所有关键机械参数、PID 参数、限幅值和开关判定值，方便赛场快速查找和调整。
 */
namespace ChassisConfig
{
constexpr int kWheelCount = 4;
constexpr int kAxisCount = 3;

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr float kHalfPi = 1.5707963267f;
constexpr float kYawStandard = 2.3508f;

constexpr float kChassisRadius = 0.24f;
constexpr float kWheelRadius = 0.07f;
constexpr float kWheelNumber = 4.0f;

constexpr float kMaxSpeed = 0.5f;
constexpr float kRemoteToChassisGain = 1.0f;
constexpr float kSpinSensitivity = 5.0f;
constexpr float kFollowDeadband = 0.05f;
constexpr float kFollowOutputLimit = 5.0f;
constexpr float kRemoteDeadzone = 20.0f;

constexpr float kFollowPidKp = 5.0f;
constexpr float kFollowPidKi = 0.0f;
constexpr float kFollowPidKd = 0.0f;
constexpr float kPidOutputLimit = 16384.0f;
constexpr float kPidIntegralLimit = 2500.0f;
constexpr float kPidIntegralSeparationThreshold = 100.0f;

constexpr float kSlopeIncrease = 0.015f;
constexpr float kSlopeDecrease = 0.025f;

constexpr uint8_t kSwitchDown = 2U;
constexpr uint8_t kSwitchMiddle = 3U;

constexpr uint16_t kMotorBaseCanId = 0x200U;
constexpr uint32_t kMotorSendCanId = 0x200U;
constexpr uint8_t kMotorCanRxIds[kWheelCount] = {1U, 2U, 3U, 4U};
} // namespace ChassisConfig

/**
 * @brief 底盘目标状态。
 */
typedef struct
{
    float target_translation_x;
    float target_translation_y;
    float target_rotation;
    float target_dial;
    float target_x;
    float target_y;
} ControlTask;

/**
 * @brief 底盘输出状态。
 */
typedef struct
{
    float out_wheel[ChassisConfig::kWheelCount];
    float out_dial;
} Output_chassis;

/**
 * @brief 底盘运行上下文。
 *
 * 只做简单聚合，不做封装；所有成员公开，便于调试器实时观察和修改。
 */
struct ChassisContext
{
    ControlTask target;
    Output_chassis output;
};

extern BSP::Motor::Dji::GM3508<ChassisConfig::kWheelCount> Motor3508;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;

extern ChassisContext chassis_context;
extern ControlTask &chassis_target;
extern Output_chassis &chassis_output;
extern float motor_wheel[ChassisConfig::kWheelCount];

extern Chassis_FSM chassis_fsm;
extern float wheel_azimuth[ChassisConfig::kWheelCount];
extern float wheel_direction[ChassisConfig::kWheelCount];
extern ALG::PID::PID wheel_pid[ChassisConfig::kWheelCount];
extern ALG::PID::PID follow_pid;
extern Alg::CalculationBase::Omni_FK omni_fk;
extern Alg::CalculationBase::Omni_IK omni_ik;
extern Alg::Utility::SlopePlanning omni_target[ChassisConfig::kAxisCount];

bool check_online();
void fsm_init();
void Chassis_Velocity_Normalize(float &vx_target, float &vy_target);
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi);
void CalculateFollow();
void Notfollow_SetTarget();
void Follow_SetTarget();
void Spin_SetTarget();
void SetTarget();
void chassis_stop();
void chassis_notfollow();
void chassis_follow();
void chassis_spin();
void main_loop_chassis(uint8_t left_sw, uint8_t right_sw, bool is_online, bool *alphabet);

extern "C"
{
void Control(void const *argument);
}

#endif
