#include "can_send_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "HAL/CAN/can_hal.hpp"
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "communication/robot_data.hpp"
#include <cmath>

namespace
{
// 底盘任务每 1 ms 更新控制，每 10 ms 更新里程计。
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
BSP::Motor::Dji::GM3508<4> chassis_motor(0x200, {3, 4, 2, 1}, 0x200, Bus::HAL_Can1, 268.0 / 17.0);
namespace
{
void Can1Receive(const HAL::CAN::Frame &frame)
{
    if (frame.dlc != 8 || frame.is_remote_frame || frame.is_extended_id)
        return;
    chassis_motor.Parse(frame);
    const uint16_t ids[] = {0x203, 0x204, 0x202, 0x201};
    for (uint8_t i = 0; i < 4; ++i)
        if (frame.id == ids[i])
            sentry_data.wheel_fb[i].publish(
                ReadMotor(chassis_motor, uint8_t(i + 1), 268.0f / 17.0f));
}
} // namespace
extern "C" void can_send_task(void *)
{
    HAL::CAN::get_can_bus_instance().get_can1().register_rx_callback(Can1Receive);
    TickType_t tick = xTaskGetTickCount();
    // 1 ms 周期只读取最新命令并发送到底盘电机。
    for (;;)
    {
        const uint8_t ids[] = {3, 4, 2, 1};
        for (uint8_t i = 0; i < 4; ++i)
        {
            const auto cmd = sentry_data.wheel_cmd[i].read();
            chassis_motor.setCAN(cmd.enable ? cmd.current : 0, ids[i]);
        }
        chassis_motor.sendCAN();
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
