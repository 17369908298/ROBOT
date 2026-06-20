#include "MotorTask.hpp"
#include "SerialTask.hpp"

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);
    }
}

namespace
{
void clear_uart_error_flags(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
}
} // namespace

extern "C"
{
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &uart_bus = HAL::UART::get_uart_bus_instance();

    if (huart->Instance == UART8)
    {
        auto &uart8 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart8);

        if (huart == uart8.get_handle())
        {
            const uint16_t rx_size = (Size <= sizeof(HI12RX_buffer)) ? Size : sizeof(HI12RX_buffer);
            HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
            HAL::UART::Data uart8_rx_data{HI12RX_buffer, rx_size};

            uart8.receive_dma_idle(uart8_rx_buffer);
            uart8.trigger_rx_callbacks(uart8_rx_data);
        }
    }
    else if (huart->Instance == USART1)
    {
        auto &uart1 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart1);

        if (huart == uart1.get_handle())
        {
            const uint16_t rx_size = (Size <= sizeof(DT7Rx_buffer)) ? Size : sizeof(DT7Rx_buffer);
            HAL::UART::Data uart1_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
            HAL::UART::Data uart1_rx_data{DT7Rx_buffer, rx_size};

            uart1.receive_dma_idle(uart1_rx_buffer);
            uart1.trigger_rx_callbacks(uart1_rx_data);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    auto &uart_bus = HAL::UART::get_uart_bus_instance();

    if (huart->Instance == UART8)
    {
        auto &uart8 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart8);

        if (huart == uart8.get_handle())
        {
            clear_uart_error_flags(huart);
            HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
            uart8.receive_dma_idle(uart8_rx_buffer);
        }
    }
    else if (huart->Instance == USART1)
    {
        auto &uart1 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart1);

        if (huart == uart1.get_handle())
        {
            clear_uart_error_flags(huart);
            HAL::UART::Data uart1_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
            uart1.receive_dma_idle(uart1_rx_buffer);
        }
    }
    // ：UART6 错误恢复 ---
    else if (huart->Instance == USART6) 
    {
        auto &uart6 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart6);
        if (huart == uart6.get_handle())
        {
            clear_uart_error_flags(huart);
            HAL_UART_AbortTransmit(huart); // 强制终止可能卡死的发送
        }
    }
}
}
