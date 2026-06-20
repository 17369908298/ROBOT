#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include <cstdint>
#include <cstring>

// 协议总长度：1(帧头) + 18(DT7) + 4(电机角度) + 1(校验和) = 24字节
#define BOARD_PACKET_SIZE 24

// 【核心修复1】：开辟双倍缓冲区，必定能容纳至少一帧完整的错位包
extern uint8_t BoardRx[BOARD_PACKET_SIZE * 2];

class BoardCommunication
{
public:
    BoardCommunication() : yaw_angle(0.0f) 
    {
        memset(dt7_raw_data, 0, 18);
    }

    // 【核心修复2】：滑动窗口扫描解析
    void ParseProtocol(const uint8_t* buffer, uint16_t size) 
    {
        // 扫包：从头开始找 0x5A，并且保证后面有足够的长度
        for(int i = 0; i <= size - BOARD_PACKET_SIZE; i++)
        {
            if(buffer[i] == 0x5A) // 找到了潜在的包头
            {
                uint8_t sum = 0;
                // 计算除了校验位之外的 23 个字节的和
                for(int j = 0; j < BOARD_PACKET_SIZE - 1; j++) {
                    sum += buffer[i + j];
                }

                // 比对校验位
                if(sum == buffer[i + BOARD_PACKET_SIZE - 1]) 
                {
                    // 校验成功！提取数据
                    memcpy(dt7_raw_data, &buffer[i + 1], 18);
                    memcpy(&yaw_angle, &buffer[i + 19], sizeof(float));
                    
                    //可选：如果你需要直接把遥控器数据喂给底层的 DT7 解析器
                    extern BSP::REMOTE_CONTROL::RemoteController DT7;
                    DT7.parseData(dt7_raw_data);

                    return; // 成功解析一包，直接退出
                }
            }
        }
    }

    float GetYawAngle() const { return yaw_angle; }
    const uint8_t* GetRemoteDT7Data() const { return dt7_raw_data; }

private:
    float yaw_angle;     
    uint8_t dt7_raw_data[18];   
};

extern BoardCommunication Cboard;

#endif