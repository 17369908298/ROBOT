#include "boards_communication.hpp"
#include "robot_data.hpp"
#include "wire_protocol.hpp"
#include <cstring>
#include <cmath>
namespace Communication
{
namespace
{
// 板间帧：A5、消息 ID、小端长度、数据、CRC-16/CCITT-FALSE，经 COBS 编码并以 0 结束。
uint8_t rx[96]{}, tx[96]{};
size_t rx_size = 0;
bool discard = false;
uint32_t last_rx_source[8]{}, last_tx_source[8]{}, last_tx_tick[8]{};
uint8_t next_slot = 0;
constexpr uint16_t ids[] = {0x000a, 0x000f, 0x000c, 0x0012, 0x000e, 0x000d, 0x0010, 0x0011};
constexpr uint16_t periods[] = {10, 20, 20, 20, 50, 10, 50, 20};
#ifdef SENTRY_GIMBAL
UART_HandleTypeDef *const uart = &huart3;
#else
UART_HandleTypeDef *const uart = &huart6;
#endif
template <class T>
bool Import(Sentry::Snapshot<T> &channel, const uint8_t *bytes, size_t n, uint8_t slot)
{
    if (n != sizeof(T))
        return false;
    T sample{};
    memcpy(&sample, bytes, n);
    // 拒绝零时间戳和重复源帧，再以本板时钟标记接收时间。
    const uint32_t source = sample.timestamp;
    if (source == 0 || source == last_rx_source[slot])
        return false;

    last_rx_source[slot] = source;
    sample.timestamp = HAL_GetTick();
    channel.publish(sample);
    return true;
}
void Decode()
{
    uint8_t raw[80]{};
    const size_t n = CobsDecode(rx, rx_size, raw, sizeof(raw));
    if (n < 6 || raw[0] != 0xa5 || n != size_t(raw[3]) + 6 ||
        ReadU16(raw + n - 2) != Crc16Ccitt(raw, n - 2))
        return;
    const uint16_t id = ReadU16(raw + 1);
    const auto *p = raw + 4;
    const size_t length = raw[3];
#ifdef SENTRY_GIMBAL
    if (id == 0x000d)
        Import(sentry_data.odometry, p, length, 5);
    else if (id == 0x0010)
        Import(sentry_data.radar, p, length, 6);
    else if (id == 0x0011)
        Import(sentry_data.referee, p, length, 7);
#else
    if (id == 0x000a)
        Import(sentry_data.remote, p, length, 0);
    else if (id == 0x000f)
        Import(sentry_data.heading, p, length, 1);
    else if (id == 0x000c)
        Import(sentry_data.yaw_joint, p, length, 2);
    else if (id == 0x0012)
        Import(sentry_data.motion_gate, p, length, 3);
    else if (id == 0x000e)
        Import(sentry_data.host, p, length, 4);
#endif
}
// 仅在源时间戳更新且达到该通道发送间隔后组帧发送。
template <class T> bool Send(Sentry::Snapshot<T> &channel, uint8_t slot, uint32_t now)
{
    const auto sample = channel.read();
    const uint32_t source = sample.timestamp;
    if (source == 0 || source == last_tx_source[slot] ||
        uint32_t(now - last_tx_tick[slot]) < periods[slot])
        return false;
    uint8_t raw[80]{};
    static_assert(sizeof(T) <= 74);
    raw[0] = 0xa5;
    WriteU16(raw + 1, ids[slot]);
    raw[3] = sizeof(T);
    memcpy(raw + 4, &sample, sizeof(T));
    WriteU16(raw + 4 + sizeof(T), Crc16Ccitt(raw, 4 + sizeof(T)));
    const auto n = CobsEncode(raw, 6 + sizeof(T), tx, sizeof(tx) - 1);
    if (!n)
        return false;
    tx[n] = 0;
    if (HAL_UART_Transmit_DMA(uart, tx, uint16_t(n + 1)) != HAL_OK)
        return false;
    last_tx_source[slot] = source;
    last_tx_tick[slot] = now;
    return true;
}
} // namespace
void BoardResetRx()
{
    rx_size = 0;
    discard = false;
}
void BoardReceive(const uint8_t *data, uint16_t size)
{
    if (!data)
        return;
    for (uint16_t i = 0; i < size; ++i)
    {
        if (data[i] == 0)
        {
            if (rx_size && !discard)
                Decode();
            rx_size = 0;
            discard = false;
        }
        else if (!discard)
        {
            if (rx_size < sizeof(rx))
                rx[rx_size++] = data[i];
            else
            {
                rx_size = 0;
                discard = true;
            }
        }
    }
}
void BoardTransmit()
{
    if (uart->gState != HAL_UART_STATE_READY)
        return;
    const uint32_t now = HAL_GetTick();
#ifdef SENTRY_GIMBAL
    for (uint8_t i = 0; i < 5; ++i)
    {
        uint8_t slot = next_slot;
        next_slot = uint8_t((next_slot + 1) % 5);
        bool sent = false;
        switch (slot)
        {
        case 0:
            sent = Send(sentry_data.remote, slot, now);
            break;
        case 1:
            sent = Send(sentry_data.heading, slot, now);
            break;
        case 2:
            sent = Send(sentry_data.yaw_joint, slot, now);
            break;
        case 3:
            sent = Send(sentry_data.motion_gate, slot, now);
            break;
        case 4:
            sent = Send(sentry_data.host, slot, now);
            break;
        }
        if (sent)
            return;
    }
#else
    for (uint8_t i = 0; i < 3; ++i)
    {
        uint8_t slot = uint8_t(5 + next_slot);
        next_slot = uint8_t((next_slot + 1) % 3);
        bool sent = false;
        switch (slot)
        {
        case 5:
            sent = Send(sentry_data.odometry, slot, now);
            break;
        case 6:
            sent = Send(sentry_data.radar, slot, now);
            break;
        case 7:
            sent = Send(sentry_data.referee, slot, now);
            break;
        }
        if (sent)
            return;
    }
#endif
}
} // namespace Communication
