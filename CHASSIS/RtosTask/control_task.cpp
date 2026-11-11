#include "control_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_control.hpp"
#include "communication/robot_data.hpp"

ChassisCmd chassis_cmd;
ChassisCtrl chassis_ctrl;
ChassisOdom chassis_odom;

extern "C" void control_task(void *)
{
    // 控制计算固定 1 ms；里程计仍按 10 ms 更新。
    chassis_cmd.Init();
    const bool ready = chassis_ctrl.Init();
    chassis_odom.Init();
    TickType_t tick = xTaskGetTickCount();
    uint32_t odom_tick = 0;
    for (;;)
    {
        const auto now = HAL_GetTick();
        chassis_cmd.Update();
        if (ready)
            chassis_ctrl.Update();
        else
            chassis_ctrl.Stop();
        if (now - odom_tick >= 10)
        {
            chassis_odom.Update();
            odom_tick = now;
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(1));
    }
}
