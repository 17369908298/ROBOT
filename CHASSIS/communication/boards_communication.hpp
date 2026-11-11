#pragma once
#include "usart.h"
#include <cstdint>
namespace Communication
{
void BoardReceive(const uint8_t *data, uint16_t size);
void BoardResetRx();
void BoardTransmit();
} // namespace Communication
