#ifndef FRICTION_SHOT_DETECTOR_HPP
#define FRICTION_SHOT_DETECTOR_HPP

#include <stdint.h>

namespace ALG
{
namespace FireControl
{

class FrictionShotDetector
{
public:
    void Reset();
    void Update(float current_a, bool enabled, uint32_t now_ms);
    bool ConsumeShotConfirmed();
    bool IsEnabled() const { return enabled_; }
    uint32_t GetEnabledElapsedMs(uint32_t now_ms) const;

private:
    static constexpr float TRIGGER_CURRENT_A = 8.0f;
    static constexpr float RELEASE_CURRENT_A = 5.5f;
    static constexpr uint32_t STARTUP_IGNORE_MS = 250U;

    bool enabled_ = false;
    bool armed_ = true;
    bool pending_ = false;
    uint32_t enabled_since_ms_ = 0U;
};

} // namespace FireControl
} // namespace ALG

#endif
