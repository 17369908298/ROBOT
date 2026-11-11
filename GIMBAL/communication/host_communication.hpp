#pragma once
#include <cstdint>
namespace Communication
{
/** @brief USB CDC 接收中断入口，仅将字节写入环形缓冲区。 */
void HostReceiveFromISR(const uint8_t *bytes, uint32_t size);
/** @brief 在线程上下文解析主机指令并发送导航和视觉数据。 */
void HostUpdate();
} // namespace Communication
