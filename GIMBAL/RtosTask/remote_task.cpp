#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "../../shared/uart_support.hpp"
#include "communication/robot_data.hpp"
#include "communication/boards_communication.hpp"
#include <cmath>
#ifdef SENTRY_GIMBAL
#include "../../shared/board_wire.hpp"
#include "../../shared/dt7_snapshot.hpp"
#include "BSP/IMU/HI12_imu.hpp"
#include "communication/host_communication.hpp"
BSP::REMOTE_CONTROL::RemoteController remote(300);
BSP::IMU::HI12_float imu, base_imu;
#else
#include "communication/referee_communication.hpp"
#endif
namespace
{
// UART1 接收 DT7；云台板另接 UART6/UART7 的 IMU，UART3 用于板间通信。
uint8_t rx1[256]{}, rx6[128]{};
#ifdef SENTRY_GIMBAL
uint8_t rx7[256]{};
#endif
struct RxPort
{
    UART_HandleTypeDef *handle;
    HAL::UART::UartDeviceId id;
    uint8_t *bytes;
    uint16_t capacity;
};
RxPort ports[] = {
    {&huart1, HAL::UART::UartDeviceId::HAL_Uart1, rx1, sizeof(rx1)},
    {&huart6, HAL::UART::UartDeviceId::HAL_Uart6, rx6, sizeof(rx6)},
#ifdef SENTRY_GIMBAL
    {&huart7, HAL::UART::UartDeviceId::HAL_Uart7, rx7, sizeof(rx7)},
#endif
};

void StartRx(HAL::UART::IUartDevice &device, const RxPort &port)
{
    HAL::UART::Data data{port.bytes, port.capacity};
    Communication::UartSupport::StartWithoutHalfEvent(device, data);
}
#ifdef SENTRY_GIMBAL
void RemoteReceive(const uint8_t *bytes, uint16_t size)
{
    constexpr auto frame_size = Communication::BoardWire::kRemoteSize;
    if (size % frame_size != 0)
        return;
    for (size_t i = 0; i < size; i += frame_size)
    {
        const auto frame = bytes + i;
        if (!Communication::IsValidDt7Frame(frame, frame_size))
            continue;
        remote.parseData(frame);
        const auto tick = HAL_GetTick();
        sentry_data.remote.publish(Communication::MakeDt7Snapshot(remote, tick));
        Communication::BoardStoreRemote(frame, tick);
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
        // HI12 加速度以 g 输出，在发布公共数据前换算为 m/s²。
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

void BoardTask(void *argument)
{
    Communication::BoardRun(*static_cast<HAL::UART::IUartDevice *>(argument));
}
} // namespace
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (Communication::BoardRxEventFromISR(uart))
        return;
    for (auto &port : ports)
    {
        if (port.handle != uart)
            continue;
        auto &device = HAL::UART::get_uart_bus_instance().get_device(port.id);
        if (Communication::UartSupport::IsHalfEvent(device))
            return;
        if (size > port.capacity)
            size = port.capacity;
#ifdef SENTRY_GIMBAL
        if (uart == &huart1)
            RemoteReceive(port.bytes, size);
        if (uart == &huart6)
            ImuReceive(base_imu, sentry_data.big_imu, port.bytes, size);
        if (uart == &huart7)
            ImuReceive(imu, sentry_data.main_imu, port.bytes, size);
#else
        if (uart == &huart1)
            Communication::RefereeReceive(port.bytes, size);
#endif
        StartRx(device, port);
        return;
    }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (Communication::BoardRxErrorFromISR(uart))
        return;
    for (auto &port : ports)
    {
        if (port.handle != uart)
            continue;
        auto &device = HAL::UART::get_uart_bus_instance().get_device(port.id);
        Communication::UartSupport::AbortReceive(device);
#ifdef SENTRY_GIMBAL
        if (uart == &huart6)
            base_imu.ResetReceiveBuffer();
        if (uart == &huart7)
            imu.ResetReceiveBuffer();
#else
        if (uart == &huart1)
            Communication::RefereeResetRx();
#endif
        StartRx(device, port);
        return;
    }
}
extern "C" void remote_task(void *)
{
    auto &bus = HAL::UART::get_uart_bus_instance();
    for (auto &port : ports)
    {
#ifndef SENTRY_GIMBAL
        if (port.handle == &huart3)
            continue; // 交接协议未配置底盘独立遥控器。
#endif
        StartRx(bus.get_device(port.id), port);
    }
    // The bus is initialized before the board task starts and before it enables its RX DMA.
    const auto created = xTaskCreate(BoardTask, "BoardLink", 512, &bus.get_uart3(), uxTaskPriorityGet(nullptr), nullptr);
    configASSERT(created == pdPASS);
    TickType_t tick = xTaskGetTickCount();
    // USB 仍由原 2 ms 任务处理，板间 UART 由 board_task 调度。
    for (;;)
    {
#ifdef SENTRY_GIMBAL
        Communication::HostUpdate();
#endif
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(2));
    }
}
