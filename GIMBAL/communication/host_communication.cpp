#include "host_communication.hpp"
#include "robot_data.hpp"
#include "wire_protocol.hpp"
#include "RtosTask/robot_config.hpp"
#include "usbd_cdc_if.h"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cstring>
#include <cmath>
namespace Communication
{
namespace
{

uint8_t ring[1024]{}, frame[48]{}, tx[48]{};
volatile uint16_t head = 0, tail = 0;
volatile bool overflow = false;
size_t collected = 0;
uint32_t last_vis_tx = 0, last_nav_tx = 0;
// 主机下行帧为 17/29 字节，上行导航和视觉帧为 48/43 字节。
float ReadFloat(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}
void PutFloat(uint8_t *p, float f)
{
    memcpy(p, &f, 4);
}
bool FiniteFloats(const uint8_t *p, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        if (!std::isfinite(ReadFloat(p + 4 * i)))
            return false;
    return true;
}
void Consume()
{
    // 按帧头和固定长度滑动重同步；CRC、模式值及浮点字段验证通过后才发布。
    while (collected)
    {
        size_t expected = 0;
        uint16_t poly = 0;
        if (frame[0] == 0xa5)
        {
            if (collected < 2)
                return;
            if (frame[1] == 2)
            {
                expected = 17;
                poly = 0xa001;
            }
        }
        else if (frame[0] == 0x53)
        {
            if (collected < 2)
                return;
            if (frame[1] == 0x50)
            {
                expected = 29;
                poly = 0x8408;
            }
        }
        if (expected && collected < expected)
            return;
        if (expected && ReadU16(frame + expected - 2) == Crc16Reflected(frame, expected - 2, poly))
        {
            if (frame[0] == 0xa5)
            {
                if (frame[14] <= 1 && FiniteFloats(frame + 2, 3))
                {
                    App::Chassis::HostMotionRequest r{};
                    r.vx_mps = ReadFloat(frame + 2);
                    r.vy_mps = ReadFloat(frame + 6);
                    r.wz_radps = ReadFloat(frame + 10);
                    r.gimbal_mode = frame[14];
                    r.timestamp = HAL_GetTick();
                    sentry_data.host.publish(r);
                }
            }
            else if (FiniteFloats(frame + 3, 6))
            {
                App::Vision::VisionControlRequest r{};
                r.control_mode = frame[2] <= 2 ? frame[2] : 0;
                r.vis_ready = r.control_mode >= 1;
                r.fire = r.control_mode == 2;
                r.yaw_angle_rad = ReadFloat(frame + 3);
                r.yaw_speed_radps = ReadFloat(frame + 7);
                r.yaw_accel_radps2 = ReadFloat(frame + 11);
                r.pitch_angle_rad = ReadFloat(frame + 15);
                r.pitch_speed_radps = ReadFloat(frame + 19);
                r.pitch_accel_radps2 = ReadFloat(frame + 23);
                r.timestamp = HAL_GetTick();
                sentry_data.vision.publish(r);
            }
            collected -= expected;
            memmove(frame, frame + expected, collected);
        }
        else
        {
            --collected;
            memmove(frame, frame + 1, collected);
        }
    }
}
bool SendVision(uint32_t now)
{
    const auto imu = sentry_data.main_imu.read();
    if (!imu.is_online(100))
        return false;
    for (float f : imu.quaternion)
        if (!std::isfinite(f))
            return false;
    if (!std::isfinite(imu.angle_deg[1]) || !std::isfinite(imu.angle_deg[2]) ||
        !std::isfinite(imu.gyro_dps[0]) || !std::isfinite(imu.gyro_dps[2]))
        return false;
    // 上行视觉帧发送弧度制角速度，四元数沿用 IMU 原始值。
    memset(tx, 0, 43);
    tx[0] = 0x53;
    tx[1] = 0x50;
    memcpy(tx + 3, imu.quaternion, 16);
    PutFloat(tx + 19, Sentry::Utils::normalize_angle(Sentry::Utils::deg_to_rad(imu.angle_deg[2])));
    PutFloat(tx + 23, Sentry::Utils::deg_to_rad(imu.gyro_dps[2]));
    PutFloat(tx + 27, Sentry::Utils::deg_to_rad(imu.angle_deg[1]));
    PutFloat(tx + 31, Sentry::Utils::deg_to_rad(imu.gyro_dps[0]));
    PutFloat(tx + 35, 23.0f);
    WriteU16(tx + 41, Crc16Reflected(tx, 41, 0x8408));
    if (CDC_Transmit_FS(tx, 43) != USBD_OK)
        return false;
    last_vis_tx = now;
    return true;
}
bool SendNavigation(uint32_t now)
{
    const auto o = sentry_data.odometry.read();
    const auto yaw = sentry_data.base_fb.read();
    if (!o.is_valid() || !o.is_online(250) || !yaw.is_online(100) ||
        !std::isfinite(yaw.output_angle_rad) || !std::isfinite(yaw.output_speed_radps))
        return false;
    // 导航帧使用机体平面速度、航向和位置；云台 yaw 按配置零位折算。
    const float values[] = {
        o.vx_body_mps,
        o.vy_body_mps,
        o.yaw_rad,
        o.x_m,
        o.y_m,
        Sentry::Utils::normalize_angle(nav_cfg::gimbal_yaw_sign *
                                       (yaw.output_angle_rad - nav_cfg::gimbal_yaw_zero_rad)),
        nav_cfg::gimbal_yaw_sign * yaw.output_speed_radps};
    for (float v : values)
        if (!std::isfinite(v))
            return false;
    memset(tx, 0, 48);
    tx[0] = 0xa5;
    tx[1] = 1;
    for (size_t i = 0; i < 7; ++i)
        PutFloat(tx + 2 + 4 * i, values[i]);
    auto r = sentry_data.radar.read();
    if (r.online(500) && r.target.version == 1 && r.target.msg_type == 1 &&
        r.target.confidence <= 100)
        memcpy(tx + 30, &r.target, 16);
    else
    {
        tx[30] = 1;
        tx[31] = 1;
    }
    WriteU16(tx + 46, Crc16Reflected(tx, 46, 0xa001));
    if (CDC_Transmit_FS(tx, 48) != USBD_OK)
        return false;
    last_nav_tx = now;
    return true;
}
} // namespace
void HostReceiveFromISR(const uint8_t *bytes, uint32_t size)
{
    // 中断仅搬运字节；协议解析留在线程上下文执行。
    for (uint32_t i = 0; i < size; ++i)
    {
        const uint16_t next = uint16_t((head + 1) % sizeof(ring));
        if (next == tail)
        {
            overflow = true;
            break;
        }
        ring[head] = bytes[i];
        __DMB();
        head = next;
    }
}
void HostUpdate()
{
    if (overflow)
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        tail = head;
        overflow = false;
        collected = 0;
        __set_PRIMASK(mask);
    }
    while (tail != head)
    {
        const uint8_t b = ring[tail];
        tail = uint16_t((tail + 1) % sizeof(ring));
        if (collected == sizeof(frame))
        {
            --collected;
            memmove(frame, frame + 1, collected);
        }
        frame[collected++] = b;
        Consume();
    }

    const uint32_t now = HAL_GetTick();

    extern bool HostTxReady();
    if (!HostTxReady())
        return;
    if (now - last_nav_tx >= 10 && SendNavigation(now))
        return;
    if (now - last_vis_tx >= 2)
        SendVision(now);
}
} // namespace Communication
extern "C" USBD_HandleTypeDef hUsbDeviceFS;
namespace Communication
{
bool HostTxReady()
{
    auto *c = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && c && c->TxState == 0;
}
} // namespace Communication

extern "C" void Sentry_UsbRx(const uint8_t *data, uint32_t size)
{
    Communication::HostReceiveFromISR(data, size);
}
