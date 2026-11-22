#pragma once

#include "usart.h"
#include <cstdint>

namespace HAL::UART { class IUartDevice; }

namespace Communication
{
struct BoardLinkStats
{
    uint32_t tx_frames = 0;
    uint32_t tx_missed_slots = 0;
    uint32_t rx_frames = 0;
    uint32_t rx_missed_slots = 0;
    uint32_t rx_max_interval_ms = 0;
    uint32_t rx_crc_errors = 0;
    uint32_t rx_framing_errors = 0;
    uint32_t rx_overruns = 0;
    uint32_t rx_old_frames = 0;
    uint32_t rx_start_failures = 0;
    uint32_t uart_errors = 0;
    uint32_t peer_restarts = 0;
};

extern volatile BoardLinkStats board_link_stats;
void BoardRun(HAL::UART::IUartDevice &device);
bool BoardRxEventFromISR(UART_HandleTypeDef *uart);
bool BoardRxErrorFromISR(UART_HandleTypeDef *uart);
#ifdef SENTRY_GIMBAL
void BoardStoreRemote(const uint8_t *bytes, uint32_t timestamp);
#endif
} // namespace Communication
