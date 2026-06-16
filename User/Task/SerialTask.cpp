#include "SerialTask.hpp"
#include "../User/Task/ControlTask.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "ControlTask.hpp"



/**
 * @brief 初始化
 */
/* 陀螺仪 ---------------------------------------------------------------------------------------------------*/
BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

/* 遥控器 ---------------------------------------------------------------------------------------------------*/
BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];

/* 数据缓冲区实例化 ---------------------------------------------------------------------------------------------*/
uint8_t BoardRx_buffer[sizeof(BoardPacket_t)]; 
uint8_t send_str2[28];
uint8_t VofaRx_buffer[128];

/* 串口接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief 串口初始化函数
 * 
 * 初始化串口并注册设备反馈数据解析回调函数
 */
// void SerialInit()
// {
//     // 实例串口
//     auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
//     auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
//      auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
//     // 设置缓冲区
//     HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
//     HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};
//     HAL::UART::Data uart6_rx_buffer{BoardRx_buffer, sizeof(BoardPacket_t)}; // 24字节

//     // 注册串口接收回调函数
//     uart1.receive_dma_idle(uart1_rx_buffer);
//     uart3.receive_dma_idle(uart3_rx_buffer);
//     uart1.register_rx_callback([](const HAL::UART::Data &data) 
//     {
//         if(data.size == 82 && data.buffer != nullptr)
//         {
//             HI12.DataUpdate(data.buffer);
//         }
//     });
//     uart3.register_rx_callback([](const HAL::UART::Data &data) 
//     {
//         if(data.size == 18 && data.buffer != nullptr)
//         {
//             DT7.parseData(data.buffer);
//         }
//     });
// }

/* VOFA 发送函数 ---------------------------------------------------------------------------------------------*/
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    const uint8_t sendSize = sizeof(float); // 单浮点数4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; 

#if ENABLE_VOFA_DEBUG
    auto &switch_uart = HAL::UART::get_uart_bus_instance().get_device(SWITCH_UART_DEV);
    HAL::UART::Data tx_data{send_str2, sizeof(send_str2)};
    switch_uart.transmit_dma(tx_data);
#endif
}

void SerialInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    auto &switch_uart = HAL::UART::get_uart_bus_instance().get_device(SWITCH_UART_DEV); // UART6
    
    HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
    HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};
    

    uart1.receive_dma_idle(uart1_rx_buffer);
    uart3.receive_dma_idle(uart3_rx_buffer);
   

    uart1.register_rx_callback([](const HAL::UART::Data &data) 
     {
         if(data.size == 82 && data.buffer != nullptr)
       {
            HI12.DataUpdate(data.buffer);
        }
    });
    uart3.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 18 && data.buffer != nullptr)
        {
            DT7.parseData(data.buffer);
        }
    });
  // 核心切换逻辑：根据宏配置决定 UART6(SWITCH_UART_DEV) 的行为
#if ENABLE_VOFA_DEBUG
    HAL::UART::Data vofa_rx_data{VofaRx_buffer, sizeof(VofaRx_buffer)};
    switch_uart.receive_dma_idle(vofa_rx_data);
    switch_uart.register_rx_callback([](const HAL::UART::Data &data) 
    {
        // 若VOFA有下发命令(如调参)在此处理，否则留空
    });
#else
    HAL::UART::Data board_rx_data{BoardRx_buffer, sizeof(BoardRx_buffer)};
    switch_uart.receive_dma_idle(board_rx_data);
    switch_uart.register_rx_callback([](const HAL::UART::Data &data) 
    {
        // 板间通讯解析逻辑放在这里
    });
#endif
    }

   

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 串口接收任务函数
 * 
 * 任务主循环，任务为空
 * 
 * @param argument 任务参数指针
 */
extern "C" {
void Serial(void const * argument)
{
    SerialInit();
    for(;;)
    { 
          vofa_send(HI12.GetGyroRad(2) ,cmd.yaw_vel,0.0f,0.0f,0.0f,0.0f);
       
        osDelay(5);
    }
}

}
