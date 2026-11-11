#pragma once
#include "uart_device.hpp"
#include <cstdint>
namespace HAL::UART {
enum class UartDeviceId:uint8_t {HAL_Uart1,HAL_Uart3,HAL_Uart6,HAL_Uart7,HAL_Uart8,MAX_DEVICES};
class IUartBus {public:virtual ~IUartBus()=default;virtual IUartDevice& get_device(UartDeviceId)=0;virtual bool has_device(UartDeviceId) const=0;
IUartDevice& get_uart1(){return get_device(UartDeviceId::HAL_Uart1);}
IUartDevice& get_uart3(){return get_device(UartDeviceId::HAL_Uart3);}
IUartDevice& get_uart6(){return get_device(UartDeviceId::HAL_Uart6);}
IUartDevice& get_uart7(){return get_device(UartDeviceId::HAL_Uart7);}
IUartDevice& get_uart8(){return get_device(UartDeviceId::HAL_Uart8);}
};
IUartBus& get_uart_bus_instance();
}
