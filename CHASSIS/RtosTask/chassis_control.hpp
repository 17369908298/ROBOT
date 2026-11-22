#pragma once

#include "RtosTask/robot_config.hpp"
#include "communication/robot_data.hpp"
#include "user/core/Alg/Omni/chassis_kinematics.hpp"
#include "user/core/Alg/Omni/fsm.hpp"
#include "user/core/Alg/Omni/kinematics_solvers.hpp"
#include "user/core/Alg/Omni/power_control.hpp"
#include "user/core/Alg/Omni/speed_planner.hpp"
#include "user/core/Alg/Omni/time_pid.hpp"
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

    bool Init();
    void Update();
    void Stop();

    /**
     * @brief 获取当前底盘控制模式。
     * @return 底盘模式状态机的当前状态。
     */
    OnmiChassisMode mode() const { return mode_fsm_.state(); }

  private:
    static constexpr std::uint8_t WHEEL_COUNT = 4U;
    static constexpr float DT_SEC = 0.001f;

    struct Inputs
    {
        std::uint32_t nowMs{0U};
        Sentry::RemoteData remote{};
        App::Chassis::ChassisMotionCommand command{};
        Sentry::MotorFeedback feedback[WHEEL_COUNT]{};
    };

    struct FeedforwardTorques
    {
        bool enabled{false};
        float inertial_nm[WHEEL_COUNT]{};
        float speed_nm[WHEEL_COUNT]{};
    };

    struct MotorLoop
    {
        Sentry::Snapshot<Sentry::MotorFeedback> *fb{nullptr};
        Sentry::Snapshot<Sentry::MotorCommand> *cmd{nullptr};
        ALG::Omni::TimePid pid{};
        bool fb_online{false};
        bool offline{true};
    };

    static constexpr ALG::Omni::TimePid::Config WHEEL_PID_DEFAULT = {
        chassis_cfg::chassis.wheel.speed_pid.kp,
        chassis_cfg::chassis.wheel.speed_pid.ki,
        chassis_cfg::chassis.wheel.speed_pid.kd,
        chassis_cfg::chassis.wheel.speed_pid.output_max,
        chassis_cfg::chassis.wheel.speed_pid.output_min,
        chassis_cfg::chassis.wheel.speed_pid.integral_max,
    };

    ALG::Omni::PowerControl<WHEEL_COUNT> power_control_{};
    ALG::Omni::PowerControlStrategy power_strategy_{50.0f};
    bool power_output_allowed_{true};
    MotorLoop wheel_[WHEEL_COUNT]{};
    ALG::Omni::FSM<OnmiChassisMode> mode_fsm_{};
    ALG::Omni::OmniSolver kinematics_;
    ALG::Omni::Omni_ID dynamics_;

    ALG::Omni::SCurveVelocityPlanning planner_vx_{};
    ALG::Omni::SCurveVelocityPlanning planner_vy_{};
    ALG::Omni::SCurveVelocityPlanning planner_wz_{};

    static ALG::Omni::Omni_ID make_dynamics();
    static ALG::Omni::SCurveVelocityPlanning::Config make_linear_planner_config();
    static ALG::Omni::SCurveVelocityPlanning::Config make_angular_planner_config();
    static bool force_ff_config_valid();
    static bool power_control_config_valid();
    static float torque_to_raw_unclamped(float torque_nm);
    static bool remote_allows_enable(const Sentry::RemoteData &remote);
    static OnmiChassisMode select_mode(const Sentry::RemoteData &remote);
    static bool command_matches_mode(OnmiChassisMode mode,
                                     const App::Chassis::ChassisMotionCommand &command);
    static bool system_safe_for_output(const Sentry::RemoteData &remote, OnmiChassisMode mode,
                                       const App::Chassis::ChassisMotionCommand &command);
    static std::int16_t clamp_current(float value);
    static bool feedback_hard_offline(const Sentry::MotorFeedback &feedback, std::uint32_t now);
    static void apply_manual_translation(float &vx, float &vy);

    Inputs readInputs();
    void update_power_budget(std::uint32_t now);
    void run_motion_command(OnmiChassisMode mode, const App::Chassis::ChassisMotionCommand &command,
                            const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT]);
    void calculate_wheel_targets(float &vx, float &vy, float &wz, float (&wheel_radps)[WHEEL_COUNT]);
    FeedforwardTorques calculate_feedforward(float vx, float vy, float wz,
                                            const float (&wheel_radps)[WHEEL_COUNT]);
    void run_wheel_speed_control(const float (&wheel_radps)[WHEEL_COUNT], const FeedforwardTorques &ff,
                                 const Sentry::MotorFeedback (&feedback)[WHEEL_COUNT]);
    void reset_wheel_control(std::uint8_t index);
    void reset_control();
    void stop_all();
};
