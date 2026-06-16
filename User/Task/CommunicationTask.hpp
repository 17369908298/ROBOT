#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/SerialTask.hpp"
#include "../User/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#pragma once
#include <cstdint>
#include <cstring>



// 强制 1 字节对齐，防止 float 产生空隙
#pragma pack(1)
typedef struct {
    uint8_t header;        // 帧头 0x5A
    uint8_t dt7_raw[18];   // 遥控器 18 字节
    float motor_angle;     // 电机角度 4 字节
    uint8_t checksum;      // 校验和 1 字节
} BoardPacket_t;
#pragma pack()

#endif 
