#ifndef SerialTask_h
#define SerialTask_h 

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/IMU/HI12_imu.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include "../User/core/BSP/SimpleKey/SimpleKey.hpp"

// ========================================================
// 1: 目标串口(UART6)挂载为 VOFA+ 调试打印
// 0: 目标串口(UART6)恢复为 原有板间通讯 (BoardCommunicationTX)
#define ENABLE_VOFA_DEBUG  0
#define SWITCH_UART_DEV HAL::UART::UartDeviceId::HAL_Uart6

extern BSP::IMU::HI12_float HI12;
extern uint8_t HI12RX_buffer[82];
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern uint8_t DT7Rx_buffer[18];

extern bool is_change;
extern bool alphabet[28];

extern uint8_t BoardRx_buffer[24]; 
extern uint8_t send_str2[28];
extern uint8_t VofaRx_buffer[128];
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);

#endif
