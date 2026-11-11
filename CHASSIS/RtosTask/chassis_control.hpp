#pragma once
#include "user/core/Alg/Omni/fsm.hpp"
#include "user/core/Alg/Omni/chassis_ff.hpp"
#include "user/core/Alg/Omni/chassis_kinematics.hpp"
#include "user/core/Alg/Omni/kinematics_solvers.hpp"
#include "user/core/Alg/Omni/time_pid.hpp"
#include "user/core/Alg/Omni/power_control.hpp"
#include "user/core/Alg/Omni/speed_planner.hpp"
#include "RtosTask/robot_config.hpp"
#include "communication/robot_data.hpp"
#include <cstdint>

/** @brief 底盘速度控制模式。 */
enum class OnmiChassisMode : std::uint8_t
{
    STOP = 0U,
    FOLLOW,
    NO_FOLLOW,
    AUTO,
    COUNT,
};

/** @brief 根据底盘指令和轮速反馈计算四轮电流。 */
class ChassisCtrl
{
  public:
    ChassisCtrl();

    /** @brief 初始化运动学、功率控制和轮速环。 */
    bool Init();
    /** @brief 执行一轮底盘闭环控制。 */
    void Update();
    /** @brief 清零四轮输出并复位速度规划。 */
    void Stop();

    OnmiChassisMode mode() const { return mode_fsm_.state(); }

  private:
    static constexpr std::uint8_t WHEEL_COUNT = 4U;
    static constexpr float DT_SEC = 0.001f;

    enum WheelIndex : std::uint8_t
    {
        FL = 0U,
        FR = 1U,
        BL = 2U,
        BR = 3U,
    };

    struct Inputs
    {
        std::uint32_t nowMs{0U};
        Sentry::RemoteData remote{};
        App::Chassis::ChassisMotionCommand command{};
        Sentry::MotorFeedback feedback[WHEEL_COUNT]{};
        bool anyOffline{false};
    };

    Inputs readInputs();

    struct MotorLoop
    {
        const char *name{nullptr};
        Sentry::Snapshot<Sentry::MotorFeedback> *fb{nullptr};
        Sentry::Snapshot<Sentry::MotorCommand> *cmd{nullptr};
        ALG::Omni::TimePid pid{};
        bool fb_online{false};
        bool offline{true};
    };

    static constexpr ALG::Omni::TimePid::Config WHEEL_PID_DEFAULT = {
        chassis_cfg::wheel_pid_kp,         chassis_cfg::wheel_pid_ki,
        chassis_cfg::wheel_pid_kd,         chassis_cfg::wheel_pid_output_max,
        chassis_cfg::wheel_pid_output_min, chassis_cfg::wheel_pid_integral_max,
    };

    ALG::Omni::PowerControl<WHEEL_COUNT> power_control_{};
    ALG::Omni::PowerControlStrategy power_strategy_{50.0f};
    bool power_output_allowed_{true};
    bool power_model_fault_{false};
    MotorLoop wheel_[WHEEL_COUNT]{};
    ALG::Omni::FSM<OnmiChassisMode> mode_fsm_{};
    ALG::Omni::OmniSolver kinematics_;

    ALG::Omni::Omni_ID dynamics_;

    ALG::Omni::SCurveVelocityPlanning planner_vx_{};
    ALG::Omni::SCurveVelocityPlanning planner_vy_{};
    ALG::Omni::SCurveVelocityPlanning planner_wz_{};

    float wheel_sign_[WHEEL_COUNT] = {
        chassis_cfg::wheel_sign_fl,
        chassis_cfg::wheel_sign_fr,
        chassis_cfg::wheel_sign_bl,
        chassis_cfg::wheel_sign_br,
    };

    float ref_wheel_radps_[WHEEL_COUNT]{};

    float last_ff_current_raw_[WHEEL_COUNT]{};
    std::int16_t last_wheel_current_[WHEEL_COUNT]{};
    std::uint32_t wheel_offline_since_ms_[WHEEL_COUNT]{};
    bool last_output_allowed_{false};

    static ALG::Omni::OmniSolver::Config make_kinematics_config();
    static ALG::Omni::Omni_ID make_dynamics();
    static ALG::Omni::SCurveVelocityPlanning::Config make_linear_planner_config();
    static ALG::Omni::SCurveVelocityPlanning::Config make_angular_planner_config();
    static bool force_ff_config_valid();
    static bool power_control_config_valid();
    static float torque_to_raw_unclamped(float torque_nm);
    static bool remote_connected(const Sentry::RemoteData &remote);
    static bool remote_allows_enable(const Sentry::RemoteData &remote);
    static OnmiChassisMode select_mode(const Sentry::RemoteData &remote);
    static bool command_matches_mode(OnmiChassisMode mode,
                                     const App::Chassis::ChassisMotionCommand &command);
    static bool system_safe_for_output(const Sentry::RemoteData &remote, OnmiChassisMode mode,
                                       const App::Chassis::ChassisMotionCommand &command);
    static const char *mode_name(OnmiChassisMode mode);
    static std::uint32_t fb_age_ms(std::uint32_t now, const Sentry::MotorFeedback &feedback);
    static std::int16_t clamp_current(float value);
    static const char *wheel_name(std::uint8_t index);

    void reset_wheel_control(std::uint8_t index);
    void update_power_budget(std::uint32_t now);
    bool update_fb_offline(const Sentry::MotorFeedback &feedback, std::uint32_t now,
                           std::uint32_t &offline_since_ms);
    void run_motion_command(OnmiChassisMode mode, const App::Chassis::ChassisMotionCommand &command,
                            const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT]);

    void reset_control();
    void stop_all();
};

/** @brief 汇总遥控器和云台通信，生成底盘运动指令。 */
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
        App::Chassis::HeadingFeedback heading{};
        App::Chassis::GimbalYawJointFeedback yawJoint{};
    };

    Inputs readInputs();

    ALG::Omni::TimePid follow_pid_{};
    bool last_remote_online_{false};
    bool last_heading_online_{false};
    bool last_yaw_joint_online_{false};
    bool last_host_request_online_{false};

    static bool is_finite(float value);
    static bool remote_connected(const Sentry::RemoteData &remote);
    static bool remote_allows_enable(const Sentry::RemoteData &remote);
    static bool auto_selected(const Sentry::RemoteData &remote);
    static bool follow_selected(const Sentry::RemoteData &remote);
    static bool shooter_owns_scroll(const Sentry::RemoteData &remote);
    static Selection select(const Sentry::RemoteData &remote);
    static App::Chassis::ChassisMotionCommand make_stop(std::uint32_t now_ms, std::uint8_t flags);
    void publish(App::Chassis::ChassisMotionCommand command);
};

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

    enum WheelIndex : std::uint8_t
    {
        FL = 0U,
        FR = 1U,
        BL = 2U,
        BR = 3U,
    };

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

    static ALG::Omni::OmniSolver::Config make_kinematics_config();
    static bool finite(float value);
    static float wrap_angle(float value);
    bool apply_stationary_gate(ALG::Omni::ChassisOdometry &velocity);
    void publish(std::uint32_t now, std::uint8_t flags, std::uint8_t valid_wheel_count);
};
