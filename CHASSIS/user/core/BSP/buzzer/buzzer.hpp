#ifndef BUZZER_HPP
#define BUZZER_HPP

#include "tim.h"
#include <stdint.h>

namespace BSP::BUZZER
{
    /**
     * @brief 更新电机在线状态；任一已检测到的电机离线时输出4 kHz蜂鸣，恢复后静音。
     * @param motor_id 电机报警编号，范围1-8，与MotorBase::isConnected第二参数一致。
     * @param online true表示在线，false表示离线。
     */
    inline void setMotorOnline(uint8_t motor_id, bool online)
    {
        if (motor_id < 1 || motor_id > 8)
        {
            return;
        }

        static bool pwm_started = false;
        static uint8_t offline_mask = 0;

        if (!pwm_started)
        {
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            pwm_started = true;
        }

        const uint8_t previous_offline_mask = offline_mask;
        const uint8_t motor_mask = static_cast<uint8_t>(1U << (motor_id - 1U));
        if (online)
        {
            offline_mask &= static_cast<uint8_t>(~motor_mask);
        }
        else
        {
            offline_mask |= motor_mask;
        }

        if (offline_mask != previous_offline_mask)
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,
                                  offline_mask == 0 ? 0 : 125);
        }
    }
}

#endif
