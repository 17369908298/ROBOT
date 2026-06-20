#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include <cstdint>

static constexpr uint8_t BOARD_PACKET_HEADER = 0x5A;
static constexpr uint16_t BOARD_HI12_RAW_SIZE = 82U;

#pragma pack(push, 1)
typedef struct
{
    uint8_t header;
    uint8_t dt7_raw[18];
    uint8_t hi12_raw[BOARD_HI12_RAW_SIZE];
    float yaw_angle;
    uint8_t checksum;
} BoardPacket_t;
#pragma pack(pop)

void SendBoardPacket();

class BoardCommunication
{
public:
    BoardCommunication() = default;

    void SetYawAngle(float yaw_angle) { yaw_angle_ = yaw_angle; }
    float GetYawAngle() const { return yaw_angle_; }

private:
    float yaw_angle_{0.0f};
};

extern BoardCommunication Cboard;

extern "C"
{
void Communication(void const *argument);
}

#endif
