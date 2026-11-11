#ifndef SINGLE_SHOT_HEAT_LIMITER_HPP
#define SINGLE_SHOT_HEAT_LIMITER_HPP

#include <stdint.h>

namespace ALG
{
namespace FireControl
{

struct SingleShotHeatConfig
{
    float projectile_heat;        // 单发增加的热量，需按当前赛季规则标定。
    float default_heat_limit;    // 没有有效裁判数据时使用的热量上限。
    float default_cooling_rate;  // 没有有效裁判数据时使用的冷却速度，单位：热量/s。
    float initial_heat;           // 控制器启动时的本地估算热量。

    SingleShotHeatConfig();
};

/**
 * @brief 英雄机器人单发拨弹的热量限制器。
 *
 * 裁判系统在线时，直接使用裁判系统上报的热量；裁判系统掉线时，
 * 从最近一次估算热量开始按冷却速度递减，并在每次接受单发后增加一次单发热量。
 * bypass_heat_limit 用于实现 V 键切换热量限制的功能。
 */
class SingleShotHeatLimiter
{
public:
    explicit SingleShotHeatLimiter(
        const SingleShotHeatConfig &config = SingleShotHeatConfig());

    /// @brief 初始化状态和默认热量参数。
    void Init();

    /// @brief 更新裁判系统热量、上限和冷却速度数据。
    void UpdateJudgeData(bool referee_online,
                         bool data_received,
                         uint16_t cooling_rate,
                         uint16_t heat_limit,
                         uint16_t current_heat,
                         uint32_t now_tick_ms);

    /**
     * @brief 判断下一发是否允许接受。
     * @param bypass_heat_limit true 时跳过热量比较，但仍检查配置是否有效。
     */
    bool CanStartShot(bool bypass_heat_limit) const;

    /**
     * @brief 记录一发已经被接受的单发。
     * @details 该函数应在单发真正进入 FEEDER_SINGLE_SHOT 时调用一次。
     */
    bool NotifyShotFired(bool bypass_heat_limit, uint32_t now_tick_ms);

    /// @brief 获取当前判断使用的裁判在线状态。
    bool IsRefereeOnline() const;
    /// @brief 获取是否曾经收到过裁判 CAN 数据。
    bool IsDataReceived() const;
    /// @brief 获取本地估算热量。
    float GetEstimatedHeat() const;
    /// @brief 获取当前热量上限。
    float GetHeatLimit() const;
    /// @brief 获取剩余热量。
    float GetRemainingHeat() const;
    /// @brief 获取当前使用的冷却速度。
    float GetCoolingRate() const;

private:
    void AdvanceCooling(uint32_t now_tick_ms);
    void RefreshRemainingHeat();
    bool IsConfigurationValid() const;

private:
    SingleShotHeatConfig config_; // 单发热量限制配置。
    bool referee_online_;         // 当前帧是否认为裁判系统在线。
    bool data_received_;          // 是否至少收到过一帧裁判数据。
    bool configuration_valid_;    // 配置参数是否有效。

    float estimated_heat_;       // 当前用于判断的估算热量。
    float heat_limit_;           // 当前热量上限，在线时来自裁判系统。
    float cooling_rate_;         // 当前冷却速度，单位：热量/s。
    float remaining_heat_;       // heat_limit_ - estimated_heat_。
    uint32_t last_update_tick_ms_; // 上次本地冷却估算的时间戳。
};

} // namespace FireControl
} // namespace ALG

#endif // SINGLE_SHOT_HEAT_LIMITER_HPP
