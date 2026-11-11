#pragma once
#include "user/core/Alg/Omni/dual_yaw_control.hpp"
#include "user/core/Alg/Omni/fsm.hpp"
#include "user/core/Alg/Omni/time_pid.hpp"
#include "user/core/Alg/Omni/roll_control.hpp"
#include "user/core/Alg/Omni/torque_current_converter.hpp"
#include "user/core/Alg/Omni/speed_planner.hpp"
#include "user/core/Alg/Omni/vision_base_angle_hold.hpp"
#include "user/core/Alg/Omni/yaw_auto_lqr_eso_ctrl.hpp"
#include "RtosTask/robot_config.hpp"
#include "communication/robot_data.hpp"
#include "feeder_fsm/transform_fsm.hpp"
#include "user/core/Alg/Omni/control_math.hpp"
#include <cstdint>
#include <cmath>

namespace App::Onmi
{

/** @brief 发布给底盘的云台转速和目标转速。 */
struct GimbalMotionStatus
{
    float bigYawSpeed{}, bigYawTargetSpeed{};
    std::uint32_t timestamp{};
};

extern Sentry::Snapshot<GimbalMotionStatus> gimbalMotionStatus;

} // namespace App::Onmi

namespace cfg = AppConfig::OnmiGimbal;

/** @brief 云台控制模式，由遥控器、视觉状态和安全条件共同决定。 */
enum class OnmiGimbalMode : std::uint8_t
{
    STOP = 0U,
    ARMING,
    MANUAL,
    VISION,
    SENTRY,
    TRANSFORM,

    COUNT,
};

// 云台内部角度使用 rad、角速度使用 rad/s、力矩使用 N*m。
// angle_rad 为 IMU 角度反馈，motor.angle_rad 为编码器反馈。
/** @brief 管理云台各轴、自瞄与变形控制。 */
class GimbalCtrl
{
  public:
    GimbalCtrl();
    /** @brief 按既有 VOFA 通道顺序导出控制数据，并返回通道数。 */
    unsigned GetVofaData(float *values, bool full) const;

    /** @brief 按 robot_config.hpp 中的配置初始化控制器。 */
    bool Init();
    /** @brief 执行一轮云台状态更新和控制计算。 */
    void Update();
    /** @brief 请求停止输出并复位可复位的运行状态。 */
    void Stop();

    OnmiGimbalMode mode() const { return state_.fsm.state(); }

  private:
    static constexpr float DT_SEC = 0.001f;
    static constexpr float SAFETY_YAW_MECH_LIMIT_RAD = 0.481799388f;
    static constexpr float SAFETY_PITCH_MIN_RAD = -0.436332313f;
    static constexpr float SAFETY_PITCH_MAX_RAD = 0.436332313f;
    static constexpr std::uint32_t SAFETY_REMOTE_TIMEOUT_MS = 300U;
    static constexpr std::uint32_t SAFETY_MAIN_IMU_TIMEOUT_MS = 200U;
    static constexpr std::uint32_t SAFETY_BASE_IMU_TIMEOUT_MS = 200U;
    static constexpr std::uint32_t SAFETY_MOTOR_FB_TIMEOUT_MS = 300U;
    static constexpr std::uint32_t SAFETY_YAW_FB_TIMEOUT_MS = 300U;
    static constexpr std::uint32_t SAFETY_VIS_TIMEOUT_MS = 100U;
    static constexpr std::uint32_t SAFETY_HOST_TIMEOUT_MS = 300U;
    static constexpr std::uint32_t SAFETY_CHASSIS_LOCK_TIMEOUT_MS = 150U;
    static_assert(cfg::gimbal.sentry.yaw.min_rad >= -SAFETY_YAW_MECH_LIMIT_RAD &&
                      cfg::gimbal.sentry.yaw.max_rad <= SAFETY_YAW_MECH_LIMIT_RAD,
                  "SENTRY yaw sweep must stay inside the fixed mechanical limit");

    struct Inputs
    {
        std::uint32_t nowMs{};
        Sentry::RemoteData remote{};
        App::Chassis::HostMotionRequest host{};
        App::Vision::VisionControlRequest vision{};
        Sentry::IMUData mainImu{};
        Sentry::IMUData bigImu{};
        Sentry::MotorFeedback bigFb{};
        Sentry::MotorFeedback smallFb{};
        Sentry::MotorFeedback pitchFb{};
        Sentry::MotorFeedback rollFb{};
        Sentry::DMMotorStatus bigStatus{};
        Sentry::DMMotorStatus pitchStatus{};
        Sentry::DMMotorStatus rollStatus{};
    };

    Inputs readInputs();
    /** @brief 更新反馈健康状态，并仅在数据有效时刷新轴角度。 */
    void updateFeedback(const Inputs &in);
    enum class VisionBigYawState : std::uint8_t
    {
        ACQUIRE = 0U,
        TRACK,
    };

    /** @brief 保存单个轴的电机角度、转速和反馈状态。 */
    struct MotorState
    {
        float angle_rad{}, speed_radps{};
        bool online{}, status_ok{};
    };
    /** @brief 统一换算电机反馈；无效转速写为 NaN 供安全检查使用。 */
    void update_motor_feedback(MotorState &motor, const Sentry::MotorFeedback &feedback);

    /** @brief 变形对齐时单轴使用的角度环、规划器和参考角。 */
    struct TransformAxis
    {
        ALG::Omni::TimePid angle_pid{};
        ALG::Omni::SCurveVelocityPlanning planner{};
        float ref_rad{};
    };

    /** @brief 单轴变形路径，封装起点、位移、已走路径和当前参考角。 */
    struct AxisPath
    {
        float start_rad{}, delta_rad{}, path_rad{}, ref_rad{};

        void reset(float angle_rad, float target_rad);
        bool advance(ALG::Omni::SCurveVelocityPlanning &planner, float max_speed, float max_accel,
                     float max_jerk, float dt_sec);
    };

    /** @brief 同步保存底盘、yaw、pitch 三轴的变形或恢复路径。 */
    struct AlignPath
    {
        AxisPath base{};
        AxisPath yaw{};
        AxisPath pitch{};
        float roll_hold_rad{};
        bool done{};
    };

    // loop 保存可复位的运行量；PID 和规划器对象保留原配置。
    /** @brief 大 yaw 轴状态与速度环。 */
    struct Base
    {
        ALG::Omni::TimePid speed_pid{};
        ALG::Omni::CutoffLowPassFilter speed_lpf{cfg::gimbal.base.feedback.speed_lpf_cutoff_hz};
        ALG::Omni::BigYawSpeedDisturbanceObserver disturbance{};
        float angle_rad{};
        MotorState motor{};
        struct Loop
        {
            float raw_ref_radps{}, ref_radps{}, speed_radps{}, torque_nm{}, dist_torque_nm{};
        } loop{};
    } base_{};
    /** @brief 小 yaw 轴状态、角度/速度环和力矩换算器。 */
    struct Yaw
    {
        ALG::Omni::TimePid angle_pid{};
        ALG::Omni::TimePid speed_pid{};
        ALG::Omni::TimePid sentry_angle_pid{};
        ALG::Omni::TimePid sentry_speed_pid{};
        Sentry::AngleLimitGuard limit{
            -SAFETY_YAW_MECH_LIMIT_RAD,
            SAFETY_YAW_MECH_LIMIT_RAD,
        };
        ALG::Omni::YawAutoLqrEsoController auto_ctrl{};
        ALG::Omni::TorqueCurrentConverter torque_converter{};
        float ref_rad{}, aim_rad{}, offset_rad{}, ref_offset_rad{}, angle_rad{};
        MotorState motor{};
        struct Loop
        {
            float raw_ref_radps{}, ref_radps{}, speed_radps{}, torque_nm{}, auto_torque_nm{},
                manual_torque_nm{};
            std::int16_t cmd_raw{0};
            bool ref_ready{};
        } loop{};
    } yaw_{};
    /** @brief pitch 轴反馈、限位和重力补偿状态。 */
    struct Pitch
    {
        ALG::Omni::TimePid angle_pid{};
        ALG::Omni::TimePid sentry_angle_pid{};
        Sentry::AngleLimitGuard limit{
            SAFETY_PITCH_MIN_RAD,
            SAFETY_PITCH_MAX_RAD,
        };
        ALG::Omni::AngleMath::CosineCache gravity_cosine{};
        float ref_rad{}, angle_rad{}, speed_radps{};
        MotorState motor{};
        struct Loop
        {
            float ref_radps{}, gravity_nm{};
        } loop{};
    } pitch_{};
    /** @brief roll 轴反馈和阻抗控制状态。 */
    struct Roll
    {
        ALG::Omni::RollImpedanceController ctrl{};
        float ref_rad{}, planned_rad{}, ref_radps{};
        std::uint32_t fb_ms{};
        MotorState motor{};
    } roll_{};
    /** @brief 手动模式下大 yaw 速度规划状态。 */
    struct Manual
    {
        ALG::Omni::SCurveVelocityPlanning planner{};
        bool ready{};
        float q_rad{}, raw_ref_radps{};
    } manual_{};
    /** @brief 自瞄的锁定、防抖、趋势估计和大 yaw 规划状态。 */
    struct Vision
    {
        ALG::Omni::SCurveVelocityPlanning planner{};
        ALG::Omni::VisionBigYawAngleHold angle_hold{};
        VisionBigYawState state{VisionBigYawState::ACQUIRE};
        struct Base
        {
            bool ready{}, centering{};
            std::uint32_t acquire_ms{}, hold_ms{};
            float init_rad{}, raw_ref_radps{};
        } base{};
        struct Trend
        {
            float angle_rad{}, speed_radps{}, accel_radps2{}, base_speed_radps{};
        } trend{};
        struct Q
        {
            float center_speed_radps{}, error_rad{}, center_weight{}, request_rad{},
                predicted_rad{};
        } q{};
        struct Ref
        {
            float speed_radps{}, accel_radps2{};
        } ref{};
        struct Mode
        {
            bool active{}, rising{};
            std::uint32_t rising_ms{};
            bool falling{};
            std::uint32_t falling_ms{};
        } mode{};
        struct Lock
        {
            bool active{};
            std::uint32_t loss_ms{};
            bool loss_hold{};
        } lock{};
        struct Alignment
        {
            float offset_rad{};
            bool valid{};
        } alignment{};
    } vision_{};
    /** @brief 双 yaw 相对速度滤波及反作用力矩状态。 */
    struct Dual
    {
        ALG::Omni::DualYawDetail::TimeConstantAlphaCache alpha_cache{};
        ALG::Omni::DualYawReactionFeedforward reaction_ff{};
        struct State
        {
            float raw_speed_radps{}, speed_radps{}, reaction_nm{};
        } state{};
    } dual_{};
    /** @brief 变形状态机、各轴规划器及进入/恢复路径。 */
    struct Tf
    {
        TransformAxis base{};
        struct Sync
        {
            float base_rad{}, yaw_rad{}, pitch_rad{};
        } sync{};
        AlignPath align{};
        AlignPath recovery{};
        TransformAxis yaw{};
        TransformAxis pitch{};
        App::Onmi::TransformFSM fsm{};
        ALG::Omni::TimePid yaw_speed_pid{};
        std::uint16_t stable_count{};
        std::uint32_t since_ms{};
        bool sync_done{}, from_stop{}, external_pending{};
    } tf_{};
    /** @brief 模式切换时的角度平滑状态。 */
    struct Blend
    {
        bool active{};
        std::uint32_t since_ms{};
        float yaw_rad{}, pitch_rad{};
    } blend_{};
    /** @brief 自主巡航启动阶段状态。 */
    struct Sweep
    {
        std::uint32_t since_ms{};
        bool ready{};
    } sentry_{};
    /** @brief 模式机、在线状态与外部同步时间戳。 */
    struct State
    {
        App::Onmi::GimbalModeRequest request{};
        ALG::Omni::FSM<OnmiGimbalMode> fsm{};
        bool chassis_locked{};
        bool stopped{true};
        bool remote_online{}, main_imu_online{}, base_imu_online{}, motors_ready{}, fb_stale{};
        std::uint8_t s1{}, s2{};
        std::uint32_t heading_ms{}, joint_ms{};
    } state_{};

    static float read_yaw_rad(const Sentry::IMUData &imu);
    static float read_pitch_rad(const Sentry::IMUData &imu);
    static float read_motor_rad(const Sentry::MotorFeedback &feedback);
    static float angle_error_rad(float ref_rad, float fb_rad);
    static float remote_channel_normalized(const Sentry::RemoteData &remote, std::uint8_t index);
    static std::uint8_t dt7_switch(const Sentry::RemoteData &remote, std::uint8_t index);
    static bool remote_connected(const Sentry::RemoteData &remote);
    static bool remote_allows_enable(const Sentry::RemoteData &remote);
    static bool switch_selected(std::uint8_t s1, std::uint8_t s2, std::uint8_t expected_s1,
                                std::uint8_t expected_s2);
    static bool active_control_mode(OnmiGimbalMode mode);
    static bool vis_lock_active(const App::Vision::VisionControlRequest &vision);
    bool update_vis_mode_debounce(bool raw_mode_active, std::uint32_t now_ms);
    static OnmiGimbalMode select_mode(const Sentry::RemoteData &remote, bool main_imu_online,
                                      bool base_imu_online, bool motors_ready, bool fb_stale,
                                      bool vis_ref_locked, bool tf_requested,
                                      bool tf_control_active, bool normal_roll_ready);
    static bool system_safe_for_external_output(const Sentry::RemoteData &remote,
                                                bool main_imu_online, bool base_imu_online,
                                                bool motors_ready, bool fb_stale);
    static bool system_safe_for_tf_output(const Sentry::RemoteData &remote, bool motors_ready,
                                          bool fb_stale);
    static float apply_deadband(float value, float deadband);
    static float clamp_speed_radps(float speed_radps, float limit_radps);
    static float slew_limit(float current, float target, float max_delta);
    static float wrap_angle_rad(float angle_rad);

    float current_yaw_offset_rad() const;
    float encoder_rel_yaw_rad() const;
    bool set_yaw_ref_from_global(float desired_ref_rad);
    bool set_yaw_ref_from_offset(float ref_offset_rad);
    bool set_sentry_yaw_ref_from_offset(float ref_offset_rad);
    float measured_yaw_torque_nm(const Sentry::MotorFeedback &yaw_fb) const;
    void reset_vis_base_planner();
    void record_vis_base_state_switch(VisionBigYawState next);
    void update_vis_yaw_alignment();
    void begin_vis_init_lock_centering();
    void update_vis_base_planner(float dt_sec);

    bool set_ref(OnmiGimbalMode mode, const Sentry::RemoteData &remote,
                 const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu,
                 const App::Vision::VisionControlRequest &vision);
    bool set_idle_ref();
    bool set_manual_ref(const Sentry::RemoteData &remote, const Sentry::IMUData &main_imu,
                        const Sentry::IMUData &base_imu);
    bool set_vis_ref(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu,
                     const App::Vision::VisionControlRequest &vision);
    bool set_sentry_ref(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu);
    void begin_mode_switch_blend(OnmiGimbalMode current_mode, std::uint32_t now_ms);
    void update_mode_switch_blend(std::uint32_t now_ms);
    void set_roll_mode_ref(OnmiGimbalMode mode);
    void update_tf_fsm(bool tf_requested, bool normal_recovery_requested);
    void publish_motion_gate();
    bool update_tf_control(const Sentry::MotorFeedback &base_fb,
                           const Sentry::MotorFeedback &yaw_fb,
                           const Sentry::MotorFeedback &pitch_fb);
    void capture_tf_refs();
    void update_tf_alignment_references();
    void capture_normal_sync_refs();
    void capture_normal_recovery_refs();
    void update_normal_recovery_references();
    bool tf_axes_reached() const;
    bool tf_roll_reached(float ref_rad) const;
    bool normal_roll_ref_reached() const;
    void reset_tf_ctrls();
    bool set_refs_from_imus(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu);
    bool sync_refs_from_imus(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu);
    bool update_control(OnmiGimbalMode mode, const Sentry::IMUData &main_imu,
                        const Sentry::IMUData &base_imu, const Sentry::MotorFeedback &yaw_fb);
    void reset_pids(bool reset_roll = true);
    void configure_gimbal_ctrls();
    /** @brief 清零本轮速度、力矩和命令，不改动控制器配置。 */
    void reset_control_outputs();

    void reset_roll_reference(float angle_rad);

    void configure_roll_ctrl();

    void update_roll_control(bool update_reference = true, float hold_ref_rad = 0.0f);
    void publish_heading(const Sentry::IMUData &base_imu);
    void publish_yaw_joint(const Sentry::MotorFeedback &base_fb);
    void publish_control(bool enable);
    void publish_stop();
};

namespace OnmiGimbalDetail
{

inline constexpr std::uint8_t DT7_CH_LEFT_Y = 0U;
inline constexpr std::uint8_t DT7_CH_LEFT_X = 1U;
inline constexpr std::uint8_t DT7_CH_RIGHT_Y = 2U;
inline constexpr std::uint8_t DT7_CH_RIGHT_X = 3U;
inline constexpr std::uint8_t DT7_CH_S1 = 4U;
inline constexpr std::uint8_t DT7_CH_LEFT_SCROLL = 5U;
inline constexpr std::uint8_t DT7_CH_RIGHT_SCROLL = 6U;
inline constexpr std::uint8_t DT7_CH_S2 = 7U;

inline float clamp_by_abs_limit(float value, float limit)
{
    const float abs_limit = std::fabs(limit);
    return Sentry::Utils::clamp(value, -abs_limit, abs_limit);
}

inline float shortest_setpoint_rad(float ref_rad, float fb_rad)
{
    return fb_rad + Sentry::Utils::normalize_angle_error(ref_rad - fb_rad);
}

} // namespace OnmiGimbalDetail
