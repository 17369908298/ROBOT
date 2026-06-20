#include "CommunicationTask.hpp"
#include "SerialTask.hpp"
#include <cstring>
#include "MotorTask.hpp"

BoardCommunication Cboard;

// Keil Watch 窗口监变量：板间通讯发送的 Yaw 轴角度 (°)
volatile float debug_yaw_tx_deg = 0.0f;

namespace
{
constexpr uint32_t UART6_TX_BUSY_TIMEOUT_MS = 100U;
uint32_t uart6_tx_busy_start_tick = 0U;

uint8_t CalculateChecksum(const BoardPacket_t &packet)
{
    uint8_t checksum = 0;
    const uint8_t *packet_buffer = reinterpret_cast<const uint8_t *>(&packet);

    for (uint16_t i = 0; i < sizeof(BoardPacket_t) - 1U; ++i)
    {
        checksum += packet_buffer[i];
    }

    return checksum;
}

void RecoverUart6TxIfTimeout(UART_HandleTypeDef *huart)
{
    if (huart == nullptr)
    {
        uart6_tx_busy_start_tick = 0U;
        return;
    }

    if (huart->gState != HAL_UART_STATE_BUSY_TX)
    {
        uart6_tx_busy_start_tick = 0U;
        return;
    }

    const uint32_t now_tick = HAL_GetTick();
    if (uart6_tx_busy_start_tick == 0U)
    {
        uart6_tx_busy_start_tick = now_tick;
        return;
    }

    if ((now_tick - uart6_tx_busy_start_tick) >= UART6_TX_BUSY_TIMEOUT_MS)
    {
        HAL_UART_AbortTransmit(huart);
        uart6_tx_busy_start_tick = 0U;
    }
}
} // namespace

void SendBoardPacket()
{
    static BoardPacket_t packet;

    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    UART_HandleTypeDef *uart6_handle = uart6.get_handle();

    RecoverUart6TxIfTimeout(uart6_handle);

    if (uart6_handle->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    packet.header = BOARD_PACKET_HEADER;
    std::memcpy(packet.dt7_raw, DT7Rx_buffer, 18);
    std::memcpy(packet.hi12_raw, HI12RX_buffer, sizeof(packet.hi12_raw));

    // 注入云台当前的 Yaw 轴输出轴角度 [0°, 360°)
    Cboard.SetYawAngle(MotorJ6006_Yaw.getOutputAngleDeg(1));
    packet.yaw_angle = Cboard.GetYawAngle();

    // 同步到 Keil Watch 监看变量
    debug_yaw_tx_deg = packet.yaw_angle;

    // 必须在赋值完毕后再计算校验和
    packet.checksum = CalculateChecksum(packet);

    HAL::UART::Data tx_data{reinterpret_cast<uint8_t *>(&packet), sizeof(BoardPacket_t)};
    uart6.transmit_dma(tx_data);
}

extern "C"
{
void Communication(void const *argument)
{
    (void)argument;

    for (;;)
    {
        SendBoardPacket();
        osDelay(2);
    }
}
}
