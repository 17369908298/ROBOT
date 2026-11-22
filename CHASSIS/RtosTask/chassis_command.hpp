#pragma once

#include "communication/robot_data.hpp"
#include "user/core/Alg/Omni/time_pid.hpp"
#include <cstdint>

/** @brief 仲裁遥控与主机导航请求，结合云台反馈生成底盘运动指令。 */
class ChassisCmd
{
  public:
    ChassisCmd();

    bool Init();
    void Update();
    void Stop();

  private:
    static constexpr float DT_SEC = 0.001f;

    enum class Selection : std::uint8_t
    {
        Stop = 0U,
        Manual,
        Auto,
    };

    struct Inputs
    {
        std::uint32_t nowMs{0U};
        Sentry::RemoteData remote{};
        App::Chassis::HostMotionRequest host{};
        App::Chassis::GimbalMotionGate gate{};
    };

    Inputs readInputs();

    ALG::Omni::TimePid follow_pid_{};

    static bool remote_connected(const Sentry::RemoteData &remote);
    static bool remote_allows_enable(const Sentry::RemoteData &remote);
    static bool auto_selected(const Sentry::RemoteData &remote);
    static bool follow_selected(const Sentry::RemoteData &remote);
    static bool shooter_owns_scroll(const Sentry::RemoteData &remote);
    static Selection select(const Sentry::RemoteData &remote);
    static App::Chassis::ChassisMotionCommand make_stop(std::uint32_t now_ms, std::uint8_t flags);
};
