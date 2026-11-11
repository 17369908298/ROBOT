#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
/** @brief 以 1 ms 周期更新底盘控制，里程计保持 10 ms 周期。 */
void control_task(void *argument);
#ifdef __cplusplus
}
#endif
