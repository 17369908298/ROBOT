#include "can_send_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "HAL/CAN/can_hal.hpp"
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "communication/robot_data.hpp"
#include <cmath>
#include "BSP/Motor/DM/DmMotor.hpp"
#include "BSP/Motor/LK/Lk_motor.hpp"
#include "feeder_fsm/motor_recovery_fsm.hpp"

namespace
{
// 云台任务每 1 ms 更新控制和电机输出。
using Bus = HAL::CAN::CanDeviceId;
template <class Motor> Sentry::MotorFeedback ReadMotor(Motor &motor, uint8_t id, float ratio)
{
    Sentry::MotorFeedback fb{};
    fb.timestamp = motor.getFeedbackTick(id);
    fb.online = motor.hasFeedback(id);
    fb.rotor_angle_deg = motor.getAngleDeg(id);
    fb.rotor_angle_rad = motor.getAngleRad(id);
    fb.output_angle_deg = fb.rotor_angle_deg;
    fb.output_angle_rad = fb.rotor_angle_rad;
    fb.output_speed_rpm = motor.getVelocityRpm(id);
    fb.output_speed_radps = motor.getVelocityRads(id);
    fb.rotor_speed_radps = fb.output_speed_radps * ratio;
    fb.speed_rpm_raw = int16_t(fb.output_speed_rpm * ratio);
    fb.total_angle_deg = motor.getAddAngleDeg(id);
    fb.total_angle_rad = fb.total_angle_deg * 0.017453292519611f;
    fb.motor_current_A = motor.getCurrent(id);
    fb.out_torque_Nm = motor.getTorque(id);
    fb.temperature = uint8_t(motor.getTemperature(id));
    fb.online = fb.online && std::isfinite(fb.output_angle_rad) &&
                std::isfinite(fb.output_speed_radps) && std::isfinite(fb.motor_current_A);
    return fb;
}
} // namespace
BSP::Motor::DM::J6006_2EC<1> base_motor(0, {3}, {2}, Bus::HAL_Can1);
BSP::Motor::DM::J4310_2EC<1> pitch_motor(0, {5}, {4}, Bus::HAL_Can1);
BSP::Motor::DM::J4310_2EC<1> roll_motor(0, {7}, {6}, Bus::HAL_Can1);
BSP::Motor::Dji::GM6020<1> yaw_motor(0x204, {2}, 0x1fe, Bus::HAL_Can2);
BSP::Motor::Dji::GM2006<3> friction_motor(0x200, {2, 1, 3}, 0x200, Bus::HAL_Can2, 1.0);
BSP::Motor::LK::LK4005<1> dial_motor(0x140, {1}, {1});
namespace
{
Class_Motor_Recovery_FSM<decltype(base_motor)> base_fsm(base_motor);
Class_Motor_Recovery_FSM<decltype(pitch_motor)> pitch_fsm(pitch_motor);
Class_Motor_Recovery_FSM<decltype(roll_motor)> roll_fsm(roll_motor);
void Can1Receive(const HAL::CAN::Frame &frame)
{
    if (frame.dlc != 8 || frame.is_remote_frame || frame.is_extended_id)
        return;
    base_motor.Parse(frame);
    pitch_motor.Parse(frame);
    roll_motor.Parse(frame);
    if (frame.id == 3)
        sentry_data.base_fb.publish(ReadMotor(base_motor, 1, 1));
    if (frame.id == 5)
        sentry_data.pitch_fb.publish(ReadMotor(pitch_motor, 1, 1));
    if (frame.id == 7)
        sentry_data.roll_fb.publish(ReadMotor(roll_motor, 1, 1));
}
void Can2Receive(const HAL::CAN::Frame &frame)
{
    if (frame.dlc != 8 || frame.is_remote_frame || frame.is_extended_id)
        return;
    yaw_motor.Parse(frame);
    friction_motor.Parse(frame);
    dial_motor.Parse(frame);
    if (frame.id == 0x206)
    {
        auto fb = ReadMotor(yaw_motor, 1, 1);
        fb.out_torque_Nm = fb.motor_current_A * 0.741f;
        sentry_data.yaw_fb.publish(fb);
    }
    if (frame.id == 0x202)
        sentry_data.fric_left_fb.publish(ReadMotor(friction_motor, 1, 1));
    if (frame.id == 0x201)
        sentry_data.fric_right_fb.publish(ReadMotor(friction_motor, 2, 1));
    if (frame.id == 0x203)
        sentry_data.fric_third_fb.publish(ReadMotor(friction_motor, 3, 1));
    if (frame.id == 0x141)
    {
        auto fb = ReadMotor(dial_motor, 1, 10);
        fb.total_angle_deg = dial_motor.getMultiAngle(1);
        fb.total_angle_rad = fb.total_angle_deg * 0.017453292519611f;
        sentry_data.dial_fb.publish(fb);
    }
}
void MotorInit()
{
    dial_motor.setAllowAccumulate(1, true);
    auto &bus = HAL::CAN::get_can_bus_instance();
    bus.get_can1().register_rx_callback(Can1Receive);
    bus.get_can2().register_rx_callback(Can2Receive);
}
void MotorSend(uint32_t now)
{
    sentry_data.base_status.publish(base_fsm.Update(sentry_data.base_mit.read(), 2, 3));
    sentry_data.pitch_status.publish(pitch_fsm.Update(sentry_data.pitch_mit.read(), 4, 5));
    sentry_data.roll_status.publish(roll_fsm.Update(sentry_data.roll_mit.read(), 6, 7));
    if ((now & 1U) == 0)
    {
        const auto yaw = sentry_data.yaw_cmd.read();
        yaw_motor.setCAN(yaw.enable ? yaw.current : 0, 2);
        yaw_motor.sendCAN();
        const auto left = sentry_data.fric_left_cmd.read(),
                   right = sentry_data.fric_right_cmd.read(),
                   third = sentry_data.fric_third_cmd.read();
        friction_motor.setCAN(left.enable ? left.current : 0, 2);
        friction_motor.setCAN(right.enable ? right.current : 0, 1);
        friction_motor.setCAN(third.enable ? third.current : 0, 3);
        friction_motor.sendCAN();
        const auto dial = sentry_data.dial_cmd.read();
        dial_motor.ctrl_Torque(1, 1, dial.enable ? dial.current : 0);
    }
}
} // namespace
extern "C" void can_send_task(void *)
{
    MotorInit();
    TickType_t tick = xTaskGetTickCount();
    // 1 ms 周期发送电机命令；GM6020、摩擦轮和拨盘仍按 2 ms 发送。
    for (;;)
    {
        const uint32_t now = HAL_GetTick();
        MotorSend(now);

        
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(1));
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1)
        return;
    HAL::CAN::Frame frame{};
    auto &device = HAL::CAN::get_can_bus_instance().get_can1();
    while (device.receive(frame))
    {
    }
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan2)
        return;
    HAL::CAN::Frame frame{};
    auto &device = HAL::CAN::get_can_bus_instance().get_can2();
    while (device.receive(frame))
    {
    }
}
