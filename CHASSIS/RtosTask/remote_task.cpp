#include "remote_task.hpp"
#include "usart.h"
#include "../../shared/uart_support.hpp"
#include "communication/boards_communication.hpp"
#include "communication/referee_communication.hpp"

namespace
{
uint8_t referee_rx[256]{};

void StartRefereeRx(HAL::UART::IUartDevice &device)
{
    HAL::UART::Data buffer{referee_rx, sizeof(referee_rx)};
    Communication::UartSupport::StartWithoutHalfEvent(device, buffer);
}
} // namespace

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (Communication::BoardRxEventFromISR(uart) || uart != &huart1)
        return;
    auto &device = HAL::UART::get_uart_bus_instance().get_uart1();
    if (Communication::UartSupport::IsHalfEvent(device))
        return;
    if (size > sizeof(referee_rx))
        size = sizeof(referee_rx);
    Communication::RefereeReceive(referee_rx, size);
    StartRefereeRx(device);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (Communication::BoardRxErrorFromISR(uart) || uart != &huart1)
        return;
    auto &device = HAL::UART::get_uart_bus_instance().get_uart1();
    Communication::UartSupport::AbortReceive(device);
    Communication::RefereeResetRx();
    StartRefereeRx(device);
}

extern "C" void remote_task(void *)
{
    auto &bus = HAL::UART::get_uart_bus_instance();
    StartRefereeRx(bus.get_uart1());
    Communication::BoardRun(bus.get_uart6());
}
