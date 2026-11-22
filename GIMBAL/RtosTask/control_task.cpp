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
Sentry::Snapshot<App::Onmi::VisionTrackingStatus> App::Onmi::visionTrackingStatus;

// VOFA：UART8/115200/JustFloat；0、2、4 档 200 Hz，其余 50 Hz。
// profile：0 拨盘、1 摩擦轮、2 云台、3 完整数据、4 位置跟踪门控。
volatile uint32_t vofa_profile = 2;
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
        const uint32_t profile = vofa_profile;
        // 云台档 12 通道，含帧尾共 52 字节，在 115200/8N1 下约需 4.51 ms。
        const uint32_t vofa_period_ms = (profile == 0U || profile == 2U || profile == 4U) ? 5U : 20U;
        if (vofa_enable && now - vofa_tick >= vofa_period_ms)
        {
            float values[29]{};
            unsigned n;
            if (profile == 4U)
                n = gimbal.GetTrackingVofaData(values);
            else
                n = profile < 2U ? shooter.GetVofaData(values, profile == 1U)
                                 : gimbal.GetVofaData(values, profile == 3U);
            vofa_sendN(values, uint8_t(n));
            vofa_tick = now;
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(1));
    }
}
