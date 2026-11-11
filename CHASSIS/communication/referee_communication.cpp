#include "referee_communication.hpp"
#include "robot_data.hpp"
#include "wire_protocol.hpp"
#include <cstring>
namespace Communication
{
namespace
{
uint8_t frame[256]{};
size_t size = 0;
Sentry::RefereeData referee;
// 校验帧头 CRC8 和整帧 CRC16 后，仅发布交接文档要求的命令 ID。
void Parse()
{
    while (size)
    {
        if (frame[0] != 0xa5)
        {
            --size;
            memmove(frame, frame + 1, size);
            continue;
        }
        if (size < 5)
            return;
        const size_t length = ReadU16(frame + 1), total = length + 9;
        if (Crc8(frame, 4) != frame[4] || total > sizeof(frame))
        {
            --size;
            memmove(frame, frame + 1, size);
            continue;
        }
        if (size < total)
            return;
        if (ReadU16(frame + total - 2) != Crc16Reflected(frame, total - 2, 0x8408))
        {
            --size;
            memmove(frame, frame + 1, size);
            continue;
        }
        const auto id = ReadU16(frame + 5);
        const auto *data = frame + 7;
        const uint32_t now = HAL_GetTick();
        ++referee.rx_frame_count;
        referee.last_cmd_id = id;
        referee.last_cmd_len = uint16_t(length);
        bool changed = false;
        // 0x0201 为机器人状态，0x0202 为功率与热量数据。
        if (id == 0x0201 && length == sizeof(referee.status))
        {
            memcpy(&referee.status, data, length);
            ++referee.robot_status_count;
            changed = true;
        }
        if (id == 0x0202 && length == sizeof(referee.power))
        {
            memcpy(&referee.power, data, length);
            ++referee.power_heat_count;
            changed = true;
        }
        if (changed)
        {
            referee.timestamp = now;
            referee.is_online = true;
            sentry_data.referee.publish(referee);
        }
        // 0x0301/0x02A0 携带雷达单目标数据。
        if (id == 0x0301 && length == 22 && ReadU16(data) == 0x02a0)
        {
            Sentry::RefereeRadarTarget radar{};
            memcpy(&radar.target, data + 6, 16);
            if (radar.target.version == 1 && radar.target.msg_type == 1 &&
                radar.target.confidence <= 100)
            {
                radar.sender_id = ReadU16(data + 2);
                radar.receiver_id = ReadU16(data + 4);
                radar.timestamp = now;
                radar.rx_count = sentry_data.radar.read().rx_count + 1;
                sentry_data.radar.publish(radar);
            }
        }
        size -= total;
        memmove(frame, frame + total, size);
    }
}
} // namespace
void RefereeResetRx()
{
    size = 0;
}
void RefereeReceive(const uint8_t *bytes, uint16_t count)
{
    for (uint16_t i = 0; i < count; ++i)
    {
        if (size == sizeof(frame))
            size = 0;
        frame[size++] = bytes[i];
        Parse();
    }
}
} // namespace Communication
