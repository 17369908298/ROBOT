#pragma once
#include <cstddef>
#include <cstdint>
namespace Communication
{

/** @brief 计算 CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021）。 */
constexpr uint16_t Crc16Ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xffff;
    while (size--)
    {
        crc ^= uint16_t(*data++) << 8;
        for (int i = 0; i < 8; ++i)
            crc = crc & 0x8000 ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
    return crc;
}
/** @brief 计算反射 CRC-16；poly 按具体协议传入。 */
constexpr uint16_t Crc16Reflected(const uint8_t *data, size_t size, uint16_t poly)
{
    uint16_t crc = 0xffff;
    while (size--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i)
            crc = crc & 1 ? uint16_t((crc >> 1) ^ poly) : uint16_t(crc >> 1);
    }
    return crc;
}
/** @brief 计算初值 0xFF、多项式 0x8C 的反射 CRC-8。 */
constexpr uint8_t Crc8(const uint8_t *data, size_t size)
{
    uint8_t crc = 0xff;
    while (size--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i)
            crc = crc & 1 ? uint8_t((crc >> 1) ^ 0x8c) : uint8_t(crc >> 1);
    }
    return crc;
}
/** @brief 将含零字节数据编码为 COBS；容量不足时返回 0。 */
constexpr size_t CobsEncode(const uint8_t *src, size_t size, uint8_t *dst, size_t cap)
{
    if (!dst || cap < 2)
        return 0;
    size_t out = 1, code_pos = 0;
    uint8_t code = 1;
    for (size_t i = 0; i < size; ++i)
    {
        if (src[i] == 0)
        {
            dst[code_pos] = code;
            code_pos = out++;
            code = 1;
            if (out > cap)
                return 0;
        }
        else
        {
            if (out >= cap)
                return 0;
            dst[out++] = src[i];
            if (++code == 0xff)
            {
                dst[code_pos] = code;
                code_pos = out++;
                code = 1;
                if (out > cap)
                    return 0;
            }
        }
    }
    dst[code_pos] = code;
    return out;
}
/** @brief 解码 COBS 数据；数据非法或输出容量不足时返回 0。 */
constexpr size_t CobsDecode(const uint8_t *src, size_t size, uint8_t *dst, size_t cap)
{
    size_t in = 0, out = 0;
    while (in < size)
    {
        uint8_t code = src[in++];
        if (code == 0 || in + code - 1 > size)
            return 0;
        for (uint8_t k = 1; k < code; ++k)
        {
            if (out >= cap || src[in] == 0)
                return 0;
            dst[out++] = src[in++];
        }
        if (code != 0xff && in < size)
        {
            if (out >= cap)
                return 0;
            dst[out++] = 0;
        }
    }
    return out;
}

constexpr uint16_t ReadU16(const uint8_t *p)
{
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
constexpr void WriteU16(uint8_t *p, uint16_t v)
{
    p[0] = uint8_t(v);
    p[1] = uint8_t(v >> 8);
}
} // namespace Communication
