#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
/** @brief 以 1 ms 周期更新云台和发射控制，并按原周期输出 VOFA。 */
void control_task(void *argument);
#ifdef __cplusplus
}
#endif
