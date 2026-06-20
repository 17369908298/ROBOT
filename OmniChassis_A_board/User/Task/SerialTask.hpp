#ifndef SerialTask_h
#define SerialTask_h 

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/IMU/HI12_imu.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include "../User/core/BSP/SimpleKey/SimpleKey.hpp"

extern BSP::IMU::HI12_float HI12;
extern uint8_t HI12RX_buffer[82];
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern uint8_t DT7Rx_buffer[18];

extern bool alphabet[28];






#endif
