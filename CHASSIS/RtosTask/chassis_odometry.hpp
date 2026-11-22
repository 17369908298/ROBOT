#pragma once

#include "communication/robot_data.hpp"
#include "user/core/Alg/Omni/kinematics_solvers.hpp"
#include <cstdint>

/** @brief 根据轮速和云台航向计算底盘里程计。 */
class ChassisOdom
{
  public:
    ChassisOdom();

    bool Init();
    void Update();
    void Stop();

  private:
    static constexpr std::uint8_t WHEEL_COUNT = 4U;

    struct Inputs
    {
        std::uint32_t nowMs{0U};
        std::uint32_t dtMs{0U};
        float wheelRpm[WHEEL_COUNT]{};
        std::uint8_t validWheels{0U};
    };

    Inputs readInputs();

    Sentry::Snapshot<Sentry::MotorFeedback> *wheel_sub_[WHEEL_COUNT]{};

    ALG::Omni::OmniSolver kinematics_;
    App::Chassis::ChassisOdometry state_{};
    std::uint32_t last_update_ms_{0U};
    bool ready_{false};
    bool stationary_lock_{true};
    bool gimbal_motion_lock_{false};

    static float wrap_angle(float value);
    bool apply_stationary_gate(ALG::Omni::ChassisOdometry &velocity);
    void publish(std::uint32_t now, std::uint8_t flags, std::uint8_t valid_wheel_count);
};
