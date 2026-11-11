#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
/** @brief 底盘 CAN 电机命令发送任务入口。 */
void can_send_task(void *argument);
#ifdef __cplusplus
}
#endif
