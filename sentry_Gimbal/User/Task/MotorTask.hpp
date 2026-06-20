#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "../core/HAL/CAN/can_hal.hpp"
#include "ControlTask.hpp"
#include "../core/BSP/Motor/DM/DmMotor.hpp"

// 导出 6006 Yaw 轴电机实例，供其他可能需要读取电机原生反馈（如温度、绝对位置）的任务使用
extern BSP::Motor::DM::J6006<1> MotorJ6006_Yaw;

// 电机层硬件与回调初始化接口
void MotorInit(void);

extern "C"
{
void Motor(void const *argument);
}

#endif // MOTORTASK_HPP
