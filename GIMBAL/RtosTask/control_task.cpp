#include "control_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_control.hpp"
#include "shooter_control.hpp"
#include "usart.h"
#include <cstring>

GimbalCtrl gimbal;
ShooterCtrl shooter;
Sentry::Snapshot<App::Onmi::GimbalMotionStatus> App::Onmi::gimbalMotionStatus;

// VOFA：UART8/115200/JustFloat，50 Hz；0 拨盘、1 摩擦轮、2 云台、3 完整数据。
volatile uint32_t vofa_profile = 0;
volatile bool vofa_enable = true;

void vofa_sendN(const float *data, uint8_t count)
{
    static uint8_t tx[29 * 4 + 4]{};
    if (!data || count == 0 || count > 29 || huart8.gState != HAL_UART_STATE_READY)
        return;
    memcpy(tx, data, count * 4);
    const uint8_t tail[] = {0, 0, 0x80, 0x7f};
    memcpy(tx + count * 4, tail, 4);
    HAL_UART_Transmit_DMA(&huart8, tx, uint16_t(count * 4 + 4));
}

extern "C" void control_task(void *)
{
    // 控制器初始化与主循环同属本任务；CAN 发送任务只消费已发布的命令。
    gimbal.Init();
    shooter.Init();
    TickType_t tick = xTaskGetTickCount();
    uint32_t vofa_tick = 0;
    for (;;)
    {
        const uint32_t now = HAL_GetTick();
        gimbal.Update();
        shooter.Update();
        if (vofa_enable && now - vofa_tick >= 20)
        {
            float values[29]{};
            unsigned n = vofa_profile < 2 ? shooter.GetVofaData(values, vofa_profile == 1)
                                          : gimbal.GetVofaData(values, vofa_profile == 3);
            vofa_sendN(values, uint8_t(n));
            vofa_tick = now;
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(1));
    }
}
