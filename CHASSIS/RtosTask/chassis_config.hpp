#pragma once
#include <cstdint>

namespace AppConfig::OnmiChassis
{

// 底盘各模块的可调参数。长度用 m、角度用 rad、转矩用 N·m；时间和速度单位见字段后缀。
struct ChassisConfig
{
    // 遥控输入死区及模式拨杆配置。
    struct Input
    {
        float analog_deadband{0.05f};

        // 遥控器 S1/S2 档位：1 为上档、2 为下档、3 为中档。
        struct Switches
        {
            std::uint8_t disable_s1{2U};
            std::uint8_t disable_s2{2U};
            std::uint8_t follow_s1{3U};
            std::uint8_t follow_s2{1U};
            std::uint8_t auto_s1{3U};
            std::uint8_t auto_s2{3U};
            std::uint8_t navigation_s1{auto_s1};
            std::uint8_t navigation_s2{auto_s2};
            std::uint8_t shooter_scroll_s1{1U};
        } switches{};
    } input{};

    // 底盘平移、旋转及轮端转速上限。
    struct Limits
    {
        float max_planar_speed_mps{2.0f};
        float max_angular_speed_radps{5.0f};
        float max_wheel_speed_rpm{420.0f};
    } limits{};

    // 全向轮几何参数；FL/FR/BL/BR 分别为左前、右前、左后、右后。
    struct Geometry
    {
        float wheel_radius_m{0.06f};
        float chassis_radius_m{0.275f};

        // 各轮安装位置相对底盘中心的方位角，单位 rad。
        struct Azimuth
        {
            float fl_rad{0.78539816339f};
            float fr_rad{-0.78539816339f};
            float bl_rad{2.35619449019f};
            float br_rad{-2.35619449019f};
        } azimuth{};

        // 各轮驱动方向角，单位 rad；与运动学模型的坐标系一致。
        struct Direction
        {
            float fl_rad{2.35619449019f};
            float fr_rad{0.78539816339f};
            float bl_rad{-2.35619449019f};
            float br_rad{-0.78539816339f};
        } direction{};
    } geometry{};

    // 遥控、运动指令、主机请求及反馈的有效期，单位 ms。
    struct Timeouts
    {
        std::uint32_t remote_connection_ms{300U};
        std::uint32_t motion_command_ms{50U};
        std::uint32_t host_request_ms{300U};
        std::uint32_t gimbal_motion_gate_ms{100U};
        std::uint32_t motor_fb_soft_ms{100U}; // 软超时后反馈不再视为在线。
        std::uint32_t motor_fb_hard_ms{250U}; // 硬超时触发底盘停机保护。
    } timeouts{};

    // 控制门控及电机在线状态的日志开关与输出间隔。
    struct Diagnostics
    {
        std::uint32_t gate_interval_ms{1000U};
        bool motor_online_enabled{true};
        std::uint32_t motor_online_interval_ms{1000U};
    } diagnostics{};

    // 轮端反馈方向、速度环及电机电流/转矩换算。
    struct Wheel
    {
        // 各轮电机正方向映射；取 +1 或 -1。
        struct Sign
        {
            float fl{-1.0f};
            float fr{-1.0f};
            float bl{-1.0f};
            float br{-1.0f};
        } sign{};

        // 轮端速度环 PID：输入 rad/s，输出驱动原始计数；积分限幅为 rad。
        struct SpeedPid
        {
            float kp{1800.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max{16384.0f};
            float output_min{-16384.0f};
            float integral_max{104.71976f};
        } speed_pid{};

        // C620 电流与命令换算；转矩常数对应减速后的轮端输出。
        struct Motor
        {
            float current_max_a{20.0f};
            float command_max_raw{16384.0f};
            float torque_constant_nm_per_a{0.24628237f};
        } motor{};
    } wheel{};

    // 底盘动力学前馈、轮端摩擦补偿及参考轨迹约束。
    struct ForceFeedforward
    {
        bool enabled{true};
        float mass_kg{16.0f};
        float yaw_inertia_kgm2{0.43f};
        float wheel_equivalent_inertia_kgm2{0.002f};
        float wheel_coulomb_friction_nm{0.0f};
        float wheel_viscous_friction_nm_per_radps{0.0f};
        float wheel_friction_smooth_radps{0.15f}; // 库仑摩擦平滑过渡的速度尺度。
        float inertial_fade_error_radps{2.0f}; // 轮速误差增大时惯性前馈衰减的尺度。

        // 平移参考的加速、减速和加加速度上限。
        struct Linear
        {
            float max_accel_mps2{5.0f};
            float max_decel_mps2{8.0f};
            float max_jerk_mps3{20.0f};
        } linear{};

        // 旋转参考的加速、减速和加加速度上限。
        struct Angular
        {
            float max_accel_radps2{8.0f};
            float max_decel_radps2{10.0f};
            float max_jerk_radps3{50.0f};
        } angular{};
    } force_ff{};

    // 电机功率模型、裁判功率限制及低缓冲能量降额。
    struct Power
    {
        bool enabled{true};

        // 单电机六项模型：1、I、w（或 |w|）、I*w、I²、w²；I 用 A，w 用转子 rad/s。
        struct Model
        {
            float coeffs[6]{
                0.614597983f, 0.005554523f, 0.004789858f, 0.017675351f, 0.114952711f, 0.000008669f,
            };
            bool absolute_speed_loss{true}; // 仅一次速度损耗项取绝对值。
        } model{};
        float fixed_w{0.0f}; // 额外固定功耗；为 0 时由模型常数项估计。
        float fallback_w{50.0f}; // 裁判数据失效时使用的功率上限。
        float margin_w{5.0f}; // 功率限制预留裕量。
        std::uint32_t referee_timeout_ms{500U};
        float buffer_low_j{20.0f}; // 进入低缓冲降额的能量阈值。
        float buffer_recover_j{30.0f}; // 退出低缓冲降额的能量阈值。
        float low_buffer_scale{0.8f}; // 低缓冲状态的功率缩放比例。
    } power{};

    // 手动云台坐标系换算、反馈有效期和运动补偿。
    struct Manual
    {
        std::uint32_t gimbal_yaw_timeout_ms{100U};
        std::uint32_t odometry_timeout_ms{100U};
        float gimbal_yaw_zero_rad{4.688f};
        float gimbal_yaw_sign{1.0f};
        float gimbal_angle_comp_rad{0.0f};
        float gimbal_phase_comp_rad{0.0f};
        float dynamic_k_base{0.00f}; // 动态平移补偿系数；为 0 时不启用该补偿。
    } manual{};

    // 底盘跟随云台的航向误差死区、折返范围及角度环。
    struct FollowYaw
    {
        float deadband_rad{0.03f};
        float half_range_rad{1.57079632679f};

        // 航向角度环 PID：输入 rad，输出 rad/s；积分限幅为 rad·s，输出默认限幅沿用底盘角速度上限。
        struct AnglePid
        {
            float kp{3.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max_radps{Limits{}.max_angular_speed_radps};
            float output_min_radps{-Limits{}.max_angular_speed_radps};
            float integral_max_rad_s{0.0f};
        } angle_pid{};
    } follow_yaw{};
};

inline constexpr ChassisConfig chassis{};

static_assert(chassis.timeouts.motor_fb_soft_ms < chassis.timeouts.motor_fb_hard_ms,
              "chassis motor soft timeout must be below hard timeout");
} // namespace AppConfig::OnmiChassis

namespace AppConfig::NavHostComm
{

// 导航通信使用与手动控制一致的云台 yaw 零点、方向及反馈有效期。
struct NavHostCommConfig
{
    std::uint32_t gimbal_yaw_timeout_ms{OnmiChassis::chassis.manual.gimbal_yaw_timeout_ms};
    float gimbal_yaw_zero_rad{OnmiChassis::chassis.manual.gimbal_yaw_zero_rad};
    float gimbal_yaw_sign{OnmiChassis::chassis.manual.gimbal_yaw_sign};
};

inline constexpr NavHostCommConfig nav_host_comm{};

} // namespace AppConfig::NavHostComm

namespace AppConfig::OnmiChassisOdometry
{

// 底盘里程计的反馈校验、速度死区及静止滞回参数。
struct OdometryConfig
{
    // 轮端反馈和外部航向的有效期。
    struct Timeouts
    {
        std::uint32_t motor_fb_ms{100U};
        std::uint32_t heading_ms{100U};
    } timeouts{};

    // 允许积分里程的更新周期范围，单位 ms。
    struct Update
    {
        std::uint32_t min_dt_ms{1U};
        std::uint32_t max_dt_ms{100U};
    } update{};

    // 反馈合理性检查：最大线速度、角速度及相邻航向跳变量。
    struct Limits
    {
        float max_linear_speed_mps{3.0f};
        float max_angular_speed_radps{12.0f};
        float max_heading_jump_rad{1.0f};
    } limits{};

    // 里程计输出速度的零值死区。
    struct Deadband
    {
        float linear_mps{0.01f};
        float angular_radps{0.01f};
    } deadband{};

    // 静止判定使用滞回：低于进入阈值判静止，高于退出阈值恢复运动。
    struct Stationary
    {
        float enter_linear_mps{0.03f};
        float exit_linear_mps{0.06f};
        float enter_angular_radps{0.03f};
        float exit_angular_radps{0.06f};
    } stationary{};
};

inline constexpr OdometryConfig odometry{};

static_assert(odometry.update.min_dt_ms <= odometry.update.max_dt_ms,
              "odometry update dt limits must be ordered");
static_assert(odometry.stationary.enter_linear_mps < odometry.stationary.exit_linear_mps &&
                  odometry.stationary.enter_angular_radps < odometry.stationary.exit_angular_radps,
              "stationary gate must have hysteresis");
} // namespace AppConfig::OnmiChassisOdometry

namespace chassis_cfg = AppConfig::OnmiChassis;
namespace odom_cfg = AppConfig::OnmiChassisOdometry;
namespace nav_cfg = AppConfig::NavHostComm;
