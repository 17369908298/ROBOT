#include "vision_communication.hpp"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <cmath>
#include <cstring>

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace BSP::Vision
{

static_assert(sizeof(float) == 4, "Vision protocol requires 32-bit float");

VisionCommunicator::VisionCommunicator()
    : rx_size_(0), pitch_angle_(0.0f), yaw_angle_(0.0f),
      pitch_velocity_(0.0f), yaw_velocity_(0.0f),
      pitch_acceleration_(0.0f), yaw_acceleration_(0.0f), mode_(0),
      has_valid_frame_(false), last_rx_tick_(0)
{
    memset(tx_buffer_, 0, sizeof(tx_buffer_));
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

uint16_t VisionCommunicator::Crc16(const uint8_t* data, uint8_t size)
{
    uint16_t crc = 0xffff;
    for (uint8_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? (crc >> 1) ^ 0x8408U : crc >> 1;
    }
    return crc;
}

void VisionCommunicator::SendToVision(const float quaternion[4], float yaw, float yaw_vel,
                                      float pitch, float pitch_vel, float bullet_speed,
                                      uint16_t bullet_count, uint8_t mode)
{
    // CDC 在传输完成前仍引用上次传入的缓冲区，忙时不能覆盖它。
    const auto* cdc = static_cast<const USBD_CDC_HandleTypeDef*>(hUsbDeviceHS.pClassData);
    if (cdc == nullptr || cdc->TxState != 0)
        return;

    tx_buffer_[0] = 'S';
    tx_buffer_[1] = 'P';
    tx_buffer_[2] = mode;
    memcpy(tx_buffer_ + 3, quaternion, 16); // w, x, y, z; STM32H7 为小端 IEEE-754
    memcpy(tx_buffer_ + 19, &yaw, 4);
    memcpy(tx_buffer_ + 23, &yaw_vel, 4);
    memcpy(tx_buffer_ + 27, &pitch, 4);
    memcpy(tx_buffer_ + 31, &pitch_vel, 4);
    memcpy(tx_buffer_ + 35, &bullet_speed, 4);
    tx_buffer_[39] = static_cast<uint8_t>(bullet_count);
    tx_buffer_[40] = static_cast<uint8_t>(bullet_count >> 8);
    const uint16_t crc = Crc16(tx_buffer_, 41);
    tx_buffer_[41] = static_cast<uint8_t>(crc);
    tx_buffer_[42] = static_cast<uint8_t>(crc >> 8);
    CDC_Transmit_HS(tx_buffer_, TX_FRAME_SIZE);
}

void VisionCommunicator::AcceptFrame()
{
    float yaw, yaw_velocity, yaw_acceleration;
    float pitch, pitch_velocity, pitch_acceleration;
    memcpy(&yaw, rx_buffer_ + 3, 4);
    memcpy(&yaw_velocity, rx_buffer_ + 7, 4);
    memcpy(&yaw_acceleration, rx_buffer_ + 11, 4);
    memcpy(&pitch, rx_buffer_ + 15, 4);
    memcpy(&pitch_velocity, rx_buffer_ + 19, 4);
    memcpy(&pitch_acceleration, rx_buffer_ + 23, 4);
    // 延续旧协议的宽松物理范围校验，拒绝 CRC 正确但目标明显失常的帧。
    if (!std::isfinite(yaw) || !std::isfinite(pitch) ||
        !std::isfinite(yaw_velocity) || !std::isfinite(yaw_acceleration) ||
        !std::isfinite(pitch_velocity) || !std::isfinite(pitch_acceleration) ||
        std::fabs(yaw) > 6.28318530718f || std::fabs(pitch) > 1.57079632679f)
        return;

    yaw_angle_ = yaw;
    yaw_velocity_ = yaw_velocity;
    yaw_acceleration_ = yaw_acceleration;
    pitch_angle_ = pitch;
    pitch_velocity_ = pitch_velocity;
    pitch_acceleration_ = pitch_acceleration;
    mode_ = rx_buffer_[2];
    last_rx_tick_ = HAL_GetTick();
    has_valid_frame_ = true;
}

void VisionCommunicator::ParseRxData(const uint8_t* data, uint32_t size)
{
    for (uint32_t i = 0; i < size; ++i)
    {
        rx_buffer_[rx_size_++] = data[i];
        while (rx_size_ >= 1 && rx_buffer_[0] != 'S')
            memmove(rx_buffer_, rx_buffer_ + 1, --rx_size_);
        while (rx_size_ >= 2 && rx_buffer_[1] != 'P')
        {
            memmove(rx_buffer_, rx_buffer_ + 1, --rx_size_);
            while (rx_size_ >= 1 && rx_buffer_[0] != 'S')
                memmove(rx_buffer_, rx_buffer_ + 1, --rx_size_);
        }
        if (rx_size_ == RX_FRAME_SIZE)
        {
            const uint16_t received_crc = static_cast<uint16_t>(rx_buffer_[27]) |
                                          (static_cast<uint16_t>(rx_buffer_[28]) << 8);
            if (Crc16(rx_buffer_, 27) == received_crc)
            {
                AcceptFrame();
                rx_size_ = 0;
            }
            else
            {
                memmove(rx_buffer_, rx_buffer_ + 1, --rx_size_);
            }
        }
    }
}

bool VisionCommunicator::IsDataFresh() const
{
    return has_valid_frame_ && (HAL_GetTick() - last_rx_tick_) < DATA_TIMEOUT_MS;
}

} // namespace BSP::Vision
