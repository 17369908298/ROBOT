#pragma once

#include "RtosTask/robot_config.hpp"

#include "communication/robot_data.hpp"
#include "user/core/Alg/Omni/time_pid.hpp"
#include "user/core/Alg/Omni/shooter_heat_limiter.hpp"
#include "user/core/Alg/Omni/control_math.hpp"

#include <cstdint>

namespace ShooterRuntime = AppConfig::OnmiShooter;

/** @brief 摩擦轮与拨盘的工作模式。 */
enum class OnmiShooterMode : std::uint8_t
{
    STOP = 0U,
    CONTINUOUS,
    VISION,
    COUNT,
};

/** @brief 自瞄发射流程：等待摩擦轮就绪后再拨弹。 */
enum class OnmiVisionShooterState : std::uint8_t
{
    IDLE = 0U,
    WAIT_READY,
    HOLD,
    FIRE,
};

/** @brief 拨盘堵转恢复流程状态。 */
enum class OnmiDialRecoveryState : std::uint8_t
{
    NORMAL = 0U,
    REVERSE,
    SETTLE,
    REINIT,
    WAIT_FEEDBACK,
    BACKOFF,
};

/** @brief 管理摩擦轮、拨盘、热量限制和堵转恢复。 */
class ShooterCtrl
{
  public:
    ShooterCtrl();
    /** @brief 按既有 VOFA 通道顺序导出拨盘或摩擦轮数据。 */
    unsigned GetVofaData(float *values, bool friction) const;

    /** @brief 装载 shooter 配置并初始化控制状态。 */
    bool Init();
    /** @brief 根据遥控器、视觉和裁判数据更新发射控制。 */
    void Update();
    /** @brief 停止发射输出并复位控制器。 */
    void Stop();

  private:
    struct Inputs
    {
        std::uint32_t nowMs{0U};
        Sentry::RemoteData remote{};
        App::Vision::VisionControlRequest vision{};
        Sentry::RefereeData referee{};
        Sentry::MotorFeedback left{};
        Sentry::MotorFeedback right{};
        Sentry::MotorFeedback third{};
        Sentry::MotorFeedback dial{};
    };

    Inputs readInputs();

    ALG::Omni::TimePid fric_left_pid_{};
    ALG::Omni::TimePid fric_right_pid_{};
    ALG::Omni::TimePid fric_third_pid_{};
    ALG::Omni::TimePid dial_angle_pid_{};
    ALG::Omni::TimePid dial_speed_pid_{};
    ALG::Omni::ShooterHeatLimiter heat_limiter_{};
    ALG::Omni::ShooterHeatLimiter::Result heat_result_{};

    OnmiShooterMode mode_{OnmiShooterMode::STOP};
    OnmiVisionShooterState vis_state_{OnmiVisionShooterState::IDLE};
    float dial_ref_deg_{0.0f};
    bool ref_synced_{false};
    bool last_dial_fb_online_{false};
    bool last_scroll_active_{false};
    std::uint32_t scroll_active_since_ms_{0U};
    std::uint32_t last_vis_lock_ms_{0U};
    bool base_fire_gate_open_{false};
    std::uint32_t base_fire_gate_clear_since_ms_{0U};
    OnmiDialRecoveryState dial_recovery_state_{OnmiDialRecoveryState::NORMAL};
    std::uint32_t dial_recovery_state_since_ms_{0U};
    std::uint32_t dial_stall_since_ms_{0U};
    float dial_stall_start_angle_deg_{0.0f};
    std::uint32_t dial_last_fb_ms_{0U};
    float dial_last_angle_deg_{0.0f};
    std::int8_t dial_normal_direction_{0};
    std::uint8_t dial_recovery_retry_count_{0U};

    static bool is_finite(float value);
    static std::uint8_t dt7_switch(const Sentry::RemoteData &remote, std::uint8_t index);
    static bool remote_connected(const Sentry::RemoteData &remote);
    static bool remote_allows_output(const Sentry::RemoteData &remote);
    static OnmiShooterMode select_mode(const Sentry::RemoteData &remote);
    bool right_scroll_active(const Sentry::RemoteData &remote) const;
    static float angle_per_tick_deg(float fire_rate_hz);
    static bool fb_online(const Sentry::MotorFeedback &feedback);
    static bool friction_wheel_stopped(const Sentry::MotorFeedback &feedback);
    static bool friction_ready(const Sentry::MotorFeedback &left,
                               const Sentry::MotorFeedback &right,
                               const Sentry::MotorFeedback &third);
    static std::int16_t clamp_fric_current(float value);
    static std::int16_t clamp_dial_current(float value);
    static float requested_fire_rate_hz(OnmiShooterMode mode);
    static float commanded_friction_speed_rpm(bool run_friction);
    static float referee_shooter_heat(const Sentry::RefereeData &referee);
    static bool vis_control_active(const App::Vision::VisionControlRequest &vision);
    static bool vis_lock_detected(const App::Vision::VisionControlRequest &vision);
    static bool vis_fire_requested(const App::Vision::VisionControlRequest &vision);
    static bool vis_dial_should_fire(OnmiVisionShooterState state);
    bool update_base_fire_gate(std::uint32_t now_ms);

    OnmiVisionShooterState update_vis_state(const App::Vision::VisionControlRequest &vision,
                                            bool friction_is_ready, std::uint32_t now_ms);
    void reset_vis_state();
    bool update_dial_recovery(const Sentry::MotorFeedback &dial, float dial_current_cmd,
                              bool motion_requested, std::uint32_t now_ms);
    void publish_dial_override(std::int16_t current, bool enable, bool reinitialize = false);
    ALG::Omni::ShooterHeatLimiter::Result update_heat_control(
        bool run_friction, float requested_fire_rate_hz, const Sentry::MotorFeedback &fric_left,
        const Sentry::MotorFeedback &fric_right, const Sentry::RefereeData &referee,
        std::uint32_t now_ms);
    void sync_dial_ref(float dial_angle_deg);
    void update_dial_ref(OnmiShooterMode mode, const Sentry::RemoteData &remote,
                         float dial_angle_deg, bool allow_fire, float fire_rate_hz,
                         std::uint32_t now_ms);
    void publish_control(bool enable, bool enable_dial, const Sentry::MotorFeedback &fric_left,
                         const Sentry::MotorFeedback &fric_right,
                         const Sentry::MotorFeedback &fric_third,
                         const Sentry::MotorFeedback &dial);
    void publish_stop(const Sentry::MotorFeedback &fric_left,
                      const Sentry::MotorFeedback &fric_right,
                      const Sentry::MotorFeedback &fric_third);
    void publish_zero_control();
    void reset_pids();
};
