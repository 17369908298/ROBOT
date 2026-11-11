#include "friction_shot_detector.hpp"
#include <math.h>

namespace ALG
{
namespace FireControl
{

void FrictionShotDetector::Reset()
{
    enabled_ = false;
    armed_ = true;
    pending_ = false;
    enabled_since_ms_ = 0U;
}

void FrictionShotDetector::Update(float current_a, bool enabled, uint32_t now_ms)
{
    if (!enabled)
    {
        Reset();
        return;
    }
    if (!enabled_)
    {
        enabled_since_ms_ = now_ms;
        armed_ = true;
        pending_ = false;
    }
    enabled_ = true;

    if (static_cast<uint32_t>(now_ms - enabled_since_ms_) < STARTUP_IGNORE_MS)
    {
        return;
    }

    const float current = fabsf(current_a);
    if (armed_ && current >= TRIGGER_CURRENT_A)
    {
        pending_ = true;
        armed_ = false;
    }
    else if (!armed_ && current <= RELEASE_CURRENT_A)
    {
        armed_ = true;
    }
}

bool FrictionShotDetector::ConsumeShotConfirmed()
{
    const bool confirmed = pending_;
    pending_ = false;
    return confirmed;
}

uint32_t FrictionShotDetector::GetEnabledElapsedMs(uint32_t now_ms) const
{
    if (!enabled_)
    {
        return 0U;
    }
    return static_cast<uint32_t>(now_ms - enabled_since_ms_);
}

} // namespace FireControl
} // namespace ALG
