#include "SerialTask.hpp"

BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];

void SerivalInit()
{
    auto &uart_bus = HAL::UART::get_uart_bus_instance();
    auto &uart8 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    auto &uart1 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart6 = uart_bus.get_device(HAL::UART::UartDeviceId::HAL_Uart6);

    uart8.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(HI12RX_buffer))
        {
            HI12.DataUpdate(data.buffer);
        }
    });

    uart1.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(DT7Rx_buffer))
        {
            DT7.parseData(data.buffer);
        }
    });

    HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
    HAL::UART::Data uart1_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};

    uart8.receive_dma_idle(uart8_rx_buffer);
    uart1.receive_dma_idle(uart1_rx_buffer);
}

extern "C"
{
void Serival(void const *argument)
{
    SerivalInit();

    for (;;)
    {
        osDelay(1);
    }
}
}
