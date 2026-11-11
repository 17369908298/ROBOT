#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "HAL/UART/uart_hal.hpp"
#include "communication/robot_data.hpp"
#include "communication/boards_communication.hpp"
#include <cstring>
#include <cmath>
#ifdef SENTRY_GIMBAL
#include "BSP/RemoteControl/DT7.hpp"
#include "BSP/IMU/HI12_imu.hpp"
#include "communication/host_communication.hpp"
BSP::REMOTE_CONTROL::RemoteController remote(300);
BSP::IMU::HI12_float imu, base_imu;
#else
#include "communication/referee_communication.hpp"
#endif
namespace
{
// UART1 接收裁判系统，UART6 接收云台板数据；本板不使用独立遥控器。
uint8_t rx1[256]{}, rx3[128]{}, rx6[128]{};
#ifdef SENTRY_GIMBAL
uint8_t rx7[256]{};
#endif
struct RxPort
{
    UART_HandleTypeDef *handle;
    uint8_t *bytes;
    uint16_t capacity;
};
RxPort ports[] = {
    {&huart1, rx1, sizeof(rx1)},
    {&huart3, rx3, sizeof(rx3)},
    {&huart6, rx6, sizeof(rx6)},
#ifdef SENTRY_GIMBAL
    {&huart7, rx7, sizeof(rx7)},
#endif
};
void StartRx(RxPort &port)
{
    auto &bus = HAL::UART::get_uart_bus_instance();
    HAL::UART::Data data{port.bytes, port.capacity};
    HAL::UART::IUartDevice *device = nullptr;
    if (port.handle == &huart1)
        device = &bus.get_uart1();
    if (port.handle == &huart3)
        device = &bus.get_uart3();
    if (port.handle == &huart6)
        device = &bus.get_uart6();
#ifdef SENTRY_GIMBAL
    if (port.handle == &huart7)
        device = &bus.get_uart7();
#endif
    if (device && device->receive_dma_idle(data) && port.handle->hdmarx)
        __HAL_DMA_DISABLE_IT(port.handle->hdmarx, DMA_IT_HT);
}
#ifdef SENTRY_GIMBAL
bool ValidRemote(const uint8_t *p)
{
    const int ch[] = {int((p[0] | p[1] << 8) & 0x7ff), int((p[1] >> 3 | p[2] << 5) & 0x7ff),
                      int((p[2] >> 6 | p[3] << 2 | p[4] << 10) & 0x7ff),
                      int((p[4] >> 1 | p[5] << 7) & 0x7ff)};
    for (int c : ch)
        if (c < 364 || c > 1684)
            return false;
    const auto s1 = (p[5] >> 6) & 3, s2 = (p[5] >> 4) & 3;
    return s1 >= 1 && s2 >= 1;
}
void RemoteReceive(const uint8_t *bytes, uint16_t size)
{
    if (size % 18 != 0)
        return;
    for (uint16_t i = 0; i < size; i += 18)
    {
        if (!ValidRemote(bytes + i))
            continue;
        remote.parseData(bytes + i);
        Sentry::RemoteData r{};
        r.ch0 = remote.get_ch0();
        r.ch1 = remote.get_ch1();
        r.ch2 = remote.get_ch2();
        r.ch3 = remote.get_ch3();
        r.scroll = remote.get_scroll();
        r.s1 = remote.get_s1();
        r.s2 = remote.get_s2();
        r.mouse_x = remote.get_mouseX();
        r.mouse_y = remote.get_mouseY();
        r.mouse_z = remote.get_mouseZ();
        r.mouse_left = remote.get_mouseLeft();
        r.mouse_right = remote.get_mouseRight();
        r.keyboard = remote.get_keyboard();
        r.left_x = remote.get_left_x();
        r.left_y = remote.get_left_y();
        r.right_x = remote.get_right_x();
        r.right_y = remote.get_right_y();
        r.scroll_norm = remote.get_scroll_();
        r.left_coord_x = remote.get_left_stick_x();
        r.left_coord_y = remote.get_left_stick_y();
        r.right_coord_x = remote.get_right_stick_x();
        r.right_coord_y = remote.get_right_stick_y();
        r.scroll_coord = remote.get_scroll_660();
        r.timestamp = HAL_GetTick();
        sentry_data.remote.publish(r);
    }
}
void ImuReceive(BSP::IMU::HI12_float &sensor, Sentry::Snapshot<Sentry::IMUData> &channel,
                const uint8_t *data, uint16_t size)
{
    sensor.DataUpdate(data, size);
    const uint32_t tick = sensor.GetFeedbackTick();
    if (tick == 0 || tick == channel.read().timestamp)
        return;
    Sentry::IMUData value{};
    for (int i = 0; i < 3; ++i)
    {
        value.angle_deg[i] = sensor.GetAngle(i);
        value.gyro_dps[i] = sensor.GetGyro(i);
        value.acc_mss[i] = sensor.GetAcc(i) * 9.80665f;
        if (!std::isfinite(value.angle_deg[i]) || !std::isfinite(value.gyro_dps[i]))
            return;
    }
    for (int i = 0; i < 4; ++i)
    {
        value.quaternion[i] = sensor.GetQuaternion(i);
        if (!std::isfinite(value.quaternion[i]))
            return;
    }
    value.timestamp = tick;
    channel.publish(value);
}
#endif
} // namespace
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart->hdmarx && HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_HT)
        return;
    for (auto &port : ports)
    {
        if (port.handle != uart)
            continue;
        if (size > port.capacity)
            size = port.capacity;
#ifdef SENTRY_GIMBAL
        if (uart == &huart1)
            RemoteReceive(port.bytes, size);
        if (uart == &huart3)
            Communication::BoardReceive(port.bytes, size);
        if (uart == &huart6)
            ImuReceive(base_imu, sentry_data.big_imu, port.bytes, size);
        if (uart == &huart7)
            ImuReceive(imu, sentry_data.main_imu, port.bytes, size);
#else
        if (uart == &huart1)
            Communication::RefereeReceive(port.bytes, size);
        if (uart == &huart6)
            Communication::BoardReceive(port.bytes, size);
#endif
        StartRx(port);
        return;
    }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    for (auto &port : ports)
    {
        if (port.handle != uart)
            continue;
        HAL_UART_AbortReceive(uart);
        __HAL_UART_CLEAR_OREFLAG(uart);
        __HAL_UART_CLEAR_NEFLAG(uart);
        __HAL_UART_CLEAR_FEFLAG(uart);
        __HAL_UART_CLEAR_PEFLAG(uart);
#ifdef SENTRY_GIMBAL
        if (uart == &huart3)
            Communication::BoardResetRx();
        if (uart == &huart6)
            base_imu.ResetReceiveBuffer();
        if (uart == &huart7)
            imu.ResetReceiveBuffer();
#else
        if (uart == &huart6)
            Communication::BoardResetRx();
        if (uart == &huart1)
            Communication::RefereeResetRx();
#endif
        StartRx(port);
        return;
    }
}
extern "C" void remote_task(void *)
{
    HAL::UART::get_uart_bus_instance();
    for (auto &port : ports)
    {
#ifndef SENTRY_GIMBAL
        if (port.handle == &huart3)
            continue; // 交接协议未配置底盘独立遥控器。
#endif
        StartRx(port);
    }
    TickType_t tick = xTaskGetTickCount();
    // 以 2 ms 周期检查并发送有更新的板间数据。
    for (;;)
    {
        Communication::BoardTransmit();
#ifdef SENTRY_GIMBAL
        Communication::HostUpdate();
#endif
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(2));
    }
}
