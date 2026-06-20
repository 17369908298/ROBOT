
#include "CommunicationTask.hpp"

BoardCommunication Cboard;

// 实例化 48 字节的接收缓冲区
uint8_t BoardRx[BOARD_PACKET_SIZE * 2];





extern "C" {
void Communication(void const * argument)
{
   
    
    for(;;)
    {
        // 接收端只需要默默接收即可，解析在中断回调中自动完成
        osDelay(5); 
    }
}
}