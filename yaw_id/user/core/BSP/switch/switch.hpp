#pragma once
#include "stm32h7xx_hal.h"

namespace BSP::Switch
{
// 单个微动开关；GPIO 时钟、输入模式和上下拉由外部配置。
class MicroSwitch
{
  public:
    // 固定高电平按下、低电平松开；debounce_ms：按下/松开的消抖时间(ms)。
    MicroSwitch(GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_ms = 20)
        : port_(port), pin_(pin), debounce_ms_(debounce_ms), stable_level_(GPIO_PIN_RESET),
          candidate_level_(stable_level_), last_change_ms_(0), has_sample_(false)
    {
    }

    // 周期调用，不阻塞；启动默认未按下，首次采样也需满足消抖时间。
    void update()
    {
        const GPIO_PinState level = getRawLevel();
        const uint32_t now_ms = HAL_GetTick();

        if (!has_sample_ || level != candidate_level_)
        {
            candidate_level_ = level;
            last_change_ms_ = now_ms;
            has_sample_ = true;
        }

        // 无符号差值可处理 HAL_GetTick() 的正常回绕。
        if (static_cast<uint32_t>(now_ms - last_change_ms_) >= debounce_ms_)
        {
            stable_level_ = candidate_level_;
        }
    }

    // 实时原始电平，不经过消抖。
    GPIO_PinState getRawLevel() const
    {
        return HAL_GPIO_ReadPin(port_, pin_);
    }

    // 消抖后的电平，仅由 update() 更新。
    GPIO_PinState getLevel() const
    {
        return stable_level_;
    }

    bool getPress() const
    {
        return stable_level_ == GPIO_PIN_SET;
    }

  private:
    GPIO_TypeDef *port_;
    uint16_t pin_;
    uint32_t debounce_ms_;
    GPIO_PinState stable_level_;
    GPIO_PinState candidate_level_;
    uint32_t last_change_ms_;
    bool has_sample_;
};
} // namespace BSP::Switch
