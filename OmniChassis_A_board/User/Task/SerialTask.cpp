#include "SerialTask.hpp"

#include "CommunicationTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];





/* 设备通讯回调与数据解析 -------------------------------------------------------------------------------------*/

void SerivalInit()
{

    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    
    HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};
    HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
    // 【核心命令】真正开启底层 DMA 空闲中断接收
    uart8.receive_dma_idle(uart8_rx_buffer);
    uart6.receive_dma_idle(uart6_rx_buffer);
    // 注册硬件层回调，将收到的生肉数据直接丢给 Cboard 解析层处理
    uart8.register_rx_callback([](const HAL::UART::Data &data) 
    {
       if(data.buffer != nullptr && data.size >= 48 )
        {
            Cboard.ParseProtocol(data.buffer, data.size);
        }
    });
		uart6.register_rx_callback([](const HAL::UART::Data &data) 
    {
      
    });
}





extern "C" {
void Serival(void const * argument)
{
    SerivalInit();
    for(;;)
    {
       
       
        osDelay(1);
    }
}

}
