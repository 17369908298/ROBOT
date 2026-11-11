#pragma once
#include "usart.h"
#include <cstdint>
namespace Communication
{
/** @brief 接收并解析板间 COBS 字节流，可处理跨回调的半帧。 */
void BoardReceive(const uint8_t *data, uint16_t size);
/** @brief UART 接收错误后清空当前板间帧。 */
void BoardResetRx();
/** @brief 按轮询顺序发送一帧有更新且到期的数据。 */
void BoardTransmit();
} // namespace Communication
