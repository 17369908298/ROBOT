#ifndef VISION_COMMUNICATION_HPP
#define VISION_COMMUNICATION_HPP

#include <cstdint>

namespace BSP::Vision
{

class VisionCommunicator
{
public:
    VisionCommunicator();

    // 电控 → 视觉：角度为 IMU 绝对角 (rad)，角速度为 rad/s。
    void SendToVision(const float quaternion[4], float yaw, float yaw_vel,
                      float pitch, float pitch_vel, float bullet_speed,
                      uint16_t bullet_count, uint8_t mode);

    // USB CDC 是字节流，允许一次输入半帧或多帧。
    void ParseRxData(const uint8_t* data, uint32_t size);

    float GetPitchAngle() const { return pitch_angle_; } // rad
    float GetYawAngle() const { return yaw_angle_; }     // rad
    float GetPitchVelocity() const { return pitch_velocity_; }         // rad/s
    float GetYawVelocity() const { return yaw_velocity_; }             // rad/s
    float GetPitchAcceleration() const { return pitch_acceleration_; } // rad/s²
    float GetYawAcceleration() const { return yaw_acceleration_; }     // rad/s²
    uint8_t GetMode() const { return mode_; } // 原样保存视觉发送的模式字节
    // 只表示收到过有效帧；是否启用视觉控制由本地输入与 IsDataFresh 决定。
    bool IsVisionReady() const { return has_valid_frame_; }
    bool IsDataFresh() const;
    static constexpr uint32_t DATA_TIMEOUT_MS = 1000;

private:
    static constexpr uint8_t TX_FRAME_SIZE = 43;
    static constexpr uint8_t RX_FRAME_SIZE = 29;
    static uint16_t Crc16(const uint8_t* data, uint8_t size);
    void AcceptFrame();

    uint8_t tx_buffer_[TX_FRAME_SIZE];
    uint8_t rx_buffer_[RX_FRAME_SIZE];
    uint8_t rx_size_;
    float pitch_angle_;
    float yaw_angle_;
    float pitch_velocity_;
    float yaw_velocity_;
    float pitch_acceleration_;
    float yaw_acceleration_;
    uint8_t mode_;
    bool has_valid_frame_;
    uint32_t last_rx_tick_;
};

} // namespace BSP::Vision

#endif // VISION_COMMUNICATION_HPP
