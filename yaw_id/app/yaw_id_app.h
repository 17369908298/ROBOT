#pragma once
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
void YawId_CreateTasks(void);
void YawId_TimerIrq(void);
void YawId_FaultStop(void);
extern volatile float yaw_id_probe_channels[24];
extern volatile unsigned yaw_id_state, yaw_id_fault, yaw_id_tx_drops, yaw_id_can_errors;
extern volatile unsigned yaw_id_init_stage, yaw_id_vofa_tx_frames, yaw_id_vofa_status_frames;
#ifdef __cplusplus
}
#endif
