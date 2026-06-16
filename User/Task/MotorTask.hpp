#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "../User/core/HAL/CAN/can_hal.hpp"
#include "../User/Task/ControlTask.hpp"

extern ALG::PID::PID yaw_pid; 
extern ALG::PID::PID yaw_angle_pid;
extern ALG::PID::PID yaw_velocity_pid;
extern ALG::PID::PID pitch_manual_pid;
extern ALG::PID::PID pitch_vision_pid; 
extern ALG::PID::PID pitch2_coder_pid;
extern ALG::PID::PID dial_pid;
extern ALG::PID::PID surgewheel_pid[2];
extern ALG::PID::PID dm4310;
extern ALG::PID::PID dm4340;
extern Gimbal_FSM gimbal_fsm;  

#endif
