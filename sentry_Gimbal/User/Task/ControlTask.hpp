#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include <cstdint>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/Alg/Filter/Filter.hpp"
#include "../core/Alg/PID/pid.hpp"
#include "../core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kControlPeriodS = 0.001f;

enum Gimbal_Control_Mode_e
{
    GIMBAL_MODE_ANGLE = 0,
    GIMBAL_MODE_VELOCITY,
};

struct GimbalTarget_t
{
    Gimbal_Control_Mode_e current_mode;
    float yaw_vel;
    float yaw_angle;
};

struct GimbalOutput_t
{
    float yaw_vel;
    float yaw_torq;
};

extern Gimbal_FSM gimbal_fsm;
extern GimbalTarget_t cmd;
extern GimbalOutput_t out;

extern ALG::PID::PID yaw_vel_pd;
extern ALG::PID::PID yaw_angle_pid;

void Yaw_SetTarget();
void Yaw_Control();

extern "C"
{
void Control(void const *argument);
}

#endif
