
/* 串口回调 ---------------------------------------------------------------------------------------------*/
#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp"


/* CAN回调 ---------------------------------------------------------------------------------------------*/
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  
    }
}


extern "C" 
{
    // 处理DMA接收中的空闲线路检测
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        // 板间通讯
        if(huart->Instance == UART8)
        {
            HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
            
            if(huart == uart8.get_handle())
            {
                uart8.receive_dma_idle(uart8_rx_buffer);
                uart8.trigger_rx_callbacks(uart8_rx_buffer);
            }
        }
				  if(huart->Instance == USART6)
        {
            HAL::UART::Data uart6_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            
            if(huart == uart6.get_handle())
            {
                uart6.receive_dma_idle(uart6_rx_buffer);
                uart6.trigger_rx_callbacks(uart6_rx_buffer);
            }
        }
			}

    // 极其关键：防止通信刚建立时因为数据量太大导致的 ORE 死锁
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == UART8) 
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            
            HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
            uart8.receive_dma_idle(uart8_rx_buffer);
        }
    }

		}