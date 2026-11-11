#include "single_shot_heat_limiter.hpp"

namespace ALG
{
namespace FireControl
{
namespace
{
// 浮点参数的最小有效正值，防止无效配置参与热量判断。
constexpr float kMinPositive = 0.001f;
// 预留裁判系统通信和热量刷新延迟。
constexpr float kShotHeatSafetyReserve = 15.0f;
}

SingleShotHeatConfig::SingleShotHeatConfig()
    : projectile_heat(100.0f),
      default_heat_limit(20000.0f),
      default_cooling_rate(2000.0f),
      initial_heat(0.0f)
{
}

SingleShotHeatLimiter::SingleShotHeatLimiter(const SingleShotHeatConfig &config)
    : config_(config)
{
    Init();
}

void SingleShotHeatLimiter::Init()
{
    // 初始化为离线模式，启动时先使用默认热量配置。
    referee_online_ = false;
    data_received_ = false;
    configuration_valid_ = IsConfigurationValid();

    estimated_heat_ = config_.initial_heat;
    heat_limit_ = config_.default_heat_limit;
    cooling_rate_ = config_.default_cooling_rate;
    remaining_heat_ = 0.0f;
    last_update_tick_ms_ = 0U;
    RefreshRemainingHeat();
}

void SingleShotHeatLimiter::UpdateJudgeData(bool referee_online,
                                             bool data_received,
                                             uint16_t cooling_rate,
                                             uint16_t heat_limit,
                                             uint16_t current_heat,
                                             uint32_t now_tick_ms)
{
    configuration_valid_ = IsConfigurationValid();
    data_received_ = data_received;

    // 无论在线还是掉线，都先推进本地冷却计时。
    // 在线时随后会用裁判系统热量覆盖；掉线时则保留这次本地估算结果。
    AdvanceCooling(now_tick_ms);

    // 裁判在线且数据基本自洽时，才使用这一帧裁判数据。
    const bool online_sample = configuration_valid_ &&
                               data_received &&
                               referee_online &&
                               heat_limit > 0U &&
                               current_heat <= heat_limit;
    referee_online_ = online_sample;

    if (online_sample)
    {
        // 在线时裁判系统是热量数据的最终依据。
        heat_limit_ = static_cast<float>(heat_limit);
        cooling_rate_ = static_cast<float>(cooling_rate);
        estimated_heat_ = static_cast<float>(current_heat);
    }
    else
    {
        // 掉线时不使用无效的实时热量，只保留上一次有效的限制参数。
        // 如果从未收到过有效参数，则使用配置中的默认值。
        if (heat_limit_ <= kMinPositive)
        {
            heat_limit_ = config_.default_heat_limit;
        }
        if (cooling_rate_ < 0.0f)
        {
            cooling_rate_ = config_.default_cooling_rate;
        }
    }

    RefreshRemainingHeat();
}

bool SingleShotHeatLimiter::CanStartShot(bool bypass_heat_limit) const
{
    // V 键解除限制时允许发射，但配置本身仍必须有效。
    if (!configuration_valid_ || bypass_heat_limit)
    {
        return configuration_valid_;
    }

    // 预测打出下一发后的热量不能超过裁判系统热量上限。
    // 发射后本地估算热量最多达到热量上限减 20。
    // 例如热量上限为 200 时，发射后不得超过 180。
    const float safe_heat_limit = heat_limit_ - kShotHeatSafetyReserve;
    return safe_heat_limit > 0.0f &&
           estimated_heat_ + config_.projectile_heat <= safe_heat_limit;
}

bool SingleShotHeatLimiter::NotifyShotFired(bool bypass_heat_limit,
                                            uint32_t now_tick_ms)
{
    // 记账前重新计算冷却，避免因为控制周期间隔造成估算偏大。
    AdvanceCooling(now_tick_ms);
    RefreshRemainingHeat();

    // 正常模式下再次确认准入；绕过模式下允许超限发射。
    if (!CanStartShot(bypass_heat_limit))
    {
        return false;
    }

    // 一次单发只在这里增加一次热量，不能在每个控制周期重复增加。
    estimated_heat_ += config_.projectile_heat;
    RefreshRemainingHeat();
    return true;
}

bool SingleShotHeatLimiter::IsRefereeOnline() const
{
    return referee_online_;
}

bool SingleShotHeatLimiter::IsDataReceived() const
{
    return data_received_;
}

float SingleShotHeatLimiter::GetEstimatedHeat() const
{
    return estimated_heat_;
}

float SingleShotHeatLimiter::GetHeatLimit() const
{
    return heat_limit_;
}

float SingleShotHeatLimiter::GetRemainingHeat() const
{
    return remaining_heat_;
}

float SingleShotHeatLimiter::GetCoolingRate() const
{
    return cooling_rate_;
}

void SingleShotHeatLimiter::AdvanceCooling(uint32_t now_tick_ms)
{
    // 第一次调用只建立时间基准，不进行冷却计算。
    if (last_update_tick_ms_ == 0U)
    {
        last_update_tick_ms_ = now_tick_ms;
        return;
    }

    const uint32_t elapsed_ms = static_cast<uint32_t>(now_tick_ms - last_update_tick_ms_);
    // 冷却量 = 冷却速度 × 经过时间，时间单位从 ms 换算为 s。
    estimated_heat_ -= cooling_rate_ * (static_cast<float>(elapsed_ms) * 0.001f);
    if (estimated_heat_ < 0.0f)
    {
        estimated_heat_ = 0.0f;
    }
    last_update_tick_ms_ = now_tick_ms;
}

void SingleShotHeatLimiter::RefreshRemainingHeat()
{
    // 剩余热量不能为负，便于上层显示和判断。
    remaining_heat_ = heat_limit_ - estimated_heat_;
    if (remaining_heat_ < 0.0f)
    {
        remaining_heat_ = 0.0f;
    }
}

bool SingleShotHeatLimiter::IsConfigurationValid() const
{
    return config_.projectile_heat > kMinPositive &&
           config_.default_heat_limit > kMinPositive &&
           config_.default_cooling_rate >= 0.0f &&
           config_.initial_heat >= 0.0f;
}

} // namespace FireControl
} // namespace ALG
