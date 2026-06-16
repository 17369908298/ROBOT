 #include "CommunicationTask.hpp"
 #include "../User/Task/MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp" // 为了使用 DT7Rx_buffer

BoardPacket_t TxPacket;

void BoardCommunicationTX()
{
// 只有在未开启 VOFA 调试时，才允许执行板间通讯发送
#if !ENABLE_VOFA_DEBUG
    // 1. 获取硬件实例 (UART6)
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    
    // 2. 获取数据：电机角度
    float angle = MotorJ4310.getAngleRad(2); 

    // 3. 填充结构体
    TxPacket.header = 0x5A;
    memcpy(TxPacket.dt7_raw, DT7Rx_buffer, 18);
    TxPacket.motor_angle = angle;

    // 4. 计算和校验
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t *)&TxPacket;
    for (size_t i = 0; i < sizeof(BoardPacket_t) - 1; i++) {
        sum += ptr[i];
    }
    TxPacket.checksum = sum;

    // 5. 调用发送
    HAL::UART::Data uart6_tx_buffer{(uint8_t*)&TxPacket, sizeof(BoardPacket_t)};
    uart6.transmit_dma(uart6_tx_buffer);
#endif
}




extern "C" {
void Communication(void const * argument)
{
    osDelay(500);
    for(;;)
    {   
        BoardCommunicationTX();
        osDelay(5); 
    }
}
}






