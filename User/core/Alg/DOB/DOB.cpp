#include "DOB.hpp"

#include <cmath>

namespace
{
float SafeFinite(float value)
{
    return std::isfinite(value) ? value : 0.0f;
}
}  // namespace

namespace ALG::DOB
{
StandardDOB::StandardDOB(float J_, float wc_, float dt_)
    : J(J_), wc(wc_), dt(dt_), filter_state(0.0f)
{
}

void StandardDOB::Reset(float init_speed)
{
    filter_state = wc * J * SafeFinite(init_speed);
}

float StandardDOB::Update(float v_real, float t_actual)
{
    v_real = SafeFinite(v_real);
    t_actual = SafeFinite(t_actual);

    if (!std::isfinite(filter_state))
    {
        Reset(v_real);
    }

    const float p_in = wc * J * v_real + t_actual;

    filter_state += wc * dt * (p_in - filter_state);

    return wc * J * v_real - filter_state;
}
}  // namespace ALG::DOB
