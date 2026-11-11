#pragma once
#include <cstdint>
namespace Communication
{
/** @brief 将裁判系统 UART 字节流解析为状态、热量和雷达数据。 */
void RefereeReceive(const uint8_t *, uint16_t);
/** @brief UART 接收错误后清空当前裁判系统帧。 */
void RefereeResetRx();
} // namespace Communication
