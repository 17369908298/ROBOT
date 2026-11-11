#pragma once
#include <cstdint>

namespace AppConfig::OnmiGimbal
{

// 云台各模块的可调参数。角度用 rad、角速度用 rad/s、转矩用 N·m；时间单位见字段后缀。
struct GimbalConfig
{

    // 遥控输入死区、方向、模式拨杆与 IMU 轴映射。
    struct Input
    {
        float analog_deadband{0.05f};
        float yaw_stick_sign{-1.0f};

        // 遥控器 S1/S2 档位：1 为上档、2 为下档、3 为中档。
        struct Switches
        {
            std::uint8_t disable_s1{2U};
            std::uint8_t disable_s2{2U};
            std::uint8_t auto_s1{3U};
            std::uint8_t auto_s2{3U};
            std::uint8_t tf_s1{1U};
            std::uint8_t tf_s2{1U};
        } switches{};
        std::uint8_t base_imu_yaw_axis{2U};
    } input{};

    // 大 yaw 轴的编码器、限幅、电机换算和控制器参数。
    struct Yaw
    {

        // 编码器零点，用于将反馈值换算为 yaw 角度。
        struct Encoder
        {
            float zero_rad{5.235f};
        } encoder{};

        // yaw 手动参考角速度、反馈速度和参考加速度上限。
        struct Limits
        {
            float rate_radps{5.617993878f};
            float speed_limit_radps{100.0f};
            float ref_accel_limit_radps2{3000.0f};
        } limits{};

        // yaw 电机转矩换算；raw 表示驱动命令的原始计数值。
        struct Motor
        {
            float torque_constant_nm_per_a{0.741f};
            float current_max_a{3.0f};
            float command_max_raw{16384.0f};
            float command_limit_raw{16384.0f};

            constexpr float max_torque_nm() const
            {
                return torque_constant_nm_per_a * current_max_a *
                       ((command_limit_raw > 0.0f && command_limit_raw < command_max_raw)
                            ? command_limit_raw / command_max_raw
                            : 1.0f);
            }
        } motor{};

        // yaw 角度环 PID；积分分离和清零阈值单位为 rad。
        struct AnglePid
        {
            float integral_separation_rad{0.07f};
            float integral_reset_rad{0.15f};
            float kp{13.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max{30.0f};
            float output_min{-30.0f};
            float integral_max{1.0f};
        } angle_pid{};

        // yaw 速度环 PID，输出范围由电机最大可用转矩确定。
        struct SpeedPid
        {
            float kp{0.225f};
            float ki{0.0f};
            float kd{0.0f};
            float integral_max{0.0f};
        } speed_pid{};

        // yaw 自动控制模型、前馈、扰动观测和转矩平滑参数。
        struct Auto
        {
            float j_kg_m2{0.009706f};
            float b_nms_rad{0.00749f};
            float k_theta{7.20298f};
            float k_omega{0.582f};
            float k_i{2.5f};
            float theta_integral_limit_rad_s{5.0f};
            float ff_gain{0.8f};
            float tau_coulomb_nm{0.02226f};
            float coulomb_smooth_rad_s{0.201333942f};
            float eso_bandwidth_rad_s{60.0f};
            float eso_comp_gain{0.3f};
            float eso_comp_limit_nm{1.0f};
            float eso_omega_gate_rad_s{10.0f};
            float eso_alpha_gate_rad_s2{50.0f};
            float tau_bias_ki{0.3f};
            float tau_bias_limit_nm{0.8f};
            float tau_meas_lpf_alpha{0.2f};
            float theta_deadband_rad{0.003f};
            float omega_deadband_rad_s{0.04f};
            float reference_speed_deadband_rad_s{0.01f};
            float torque_slew_rate_nm_s{1000.0f};
            bool eso_enable{true};
            bool eso_comp_enable{true};
            bool torque_slew_enable{true};
        } auto_control{};
    } yaw{};

    // IMU 航向融合的角度修正量。
    struct Integration
    {
        float heading_offset_rad{0.0f};
    } integration{};

    // 底盘大 yaw 轴的手动跟随、速度环和扰动补偿参数。
    struct Base
    {

        // 手动跟随控制；q 表示底盘与云台的相对航向误差。
        struct Manual
        {
            float q_deadband_rad{0.08f};
            float q_kp_radps_per_rad{3.0f};
            float speed_limit_radps{4.0f};
            float accel_limit_radps2{50.0f};
            float jerk_limit_radps3{500.0f};
        } manual{};

        // 底盘角速度反馈的低通滤波参数。
        struct Feedback
        {
            float speed_lpf_cutoff_hz{40.0f};
        } feedback{};

        // 底盘电机 MIT 模式的阻尼、速度和转矩限制。
        struct Mit
        {
            float kd{0.0f};
            float speed_limit_radps{100.0f};
            float torque_limit_nm{12.0f};
            float torque_offset_nm{0.0f};
        } mit{};

        // 底盘速度环 PID，输出限幅单位为 N·m。
        struct SpeedPid
        {
            float kp{3.20f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max{10.0f};
            float output_min{-10.0f};
            float integral_max{0.0f};
        } speed_pid{};

        // 底盘 yaw 扰动观测器的名义模型和补偿限制。
        struct Observer
        {
            float nominal_inertia_kgm2{0.08f};
            float nominal_damping_nms_rad{0.05f};
            float bandwidth_rad_s{60.0f};
            float comp_gain{0.1f};
            float comp_limit_nm{1.0f};
        } observer{};
    } base{};

    // pitch 轴角速度限制、MIT 控制、角度环及重力补偿。
    struct Pitch
    {

        // pitch 手动输入的参考角速度上限。
        struct Limits
        {
            float rate_radps{5.617993878f};
        } limits{};

        // pitch 电机 MIT 模式的阻尼、速度和转矩限制。
        struct Mit
        {
            float kd{1.2f};
            float speed_limit_radps{29.0f};
            float torque_limit_nm{7.0f};
            float torque_offset_nm{0.0f};
        } mit{};

        // pitch 角度环 PID，输出为参考角速度 rad/s。
        struct AnglePid
        {
            float kp{22.0f};
            float ki{2.0f};
            float kd{0.0f};
            float output_max_radps{26.179938780f};
            float output_min_radps{-26.179938780f};
            float integral_max_rad_s{1.745329252f};
            float integral_separation_rad{0.087266463f};
            float integral_reset_rad{0.261799388f};
        } angle_pid{};

        // pitch 重力前馈系数及补偿相位。
        struct Gravity
        {
            float k_nm{1.1f};
            float offset_rad{0.0f};
        } gravity{};
    } pitch{};

    // 视觉跟踪、底盘运动补偿、目标捕获与丢失保持参数。
    struct Vision
    {

        // 手动与视觉模式切换后的等待时间。
        struct Mode
        {
            // 手动/视觉模式切换后的死区时间，单位：毫秒。
            std::uint32_t switch_deadtime_ms{80U};
        } mode{};

        // 视觉模式底盘跟随限幅及运动趋势估计参数。
        struct Base
        {
            // 底盘跟随角速度上限，单位：rad/s。
            float speed_limit_radps{3.0f};
            // 底盘跟随角加速度上限，单位：rad/s²。
            float accel_limit_radps2{10.0f};
            // 底盘跟随角加加速度（jerk）上限，单位：rad/s³。
            float jerk_limit_radps3{100.0f};

            // 目标居中保持的增益、运动补偿和积分抑制参数。
            struct Hold
            {
                // 目标中心误差到保持角速度的比例增益，单位：(rad/s)/rad。
                float kp_radps_per_rad{5.0f};
                // 居中保持角速度上限，单位：rad/s。
                float speed_limit_radps{2.0f};
                // 居中保持角加速度上限，单位：rad/s²。
                float accel_limit_radps2{15.0f};
                // 运动趋势低通滤波截止频率，单位：Hz。
                float motion_trend_cutoff_hz{0.5f};
                // 启用运动补偿的底盘速度起始阈值，单位：rad/s。
                float motion_speed_start_radps{0.08f};
                // 运动补偿达到满权重的底盘速度，单位：rad/s。
                float motion_speed_full_radps{0.40f};
                // 运动补偿权重上升时间，单位：秒。
                float motion_rise_time_sec{0.03f};
                // 运动补偿权重释放时间，单位：秒。
                float motion_release_time_sec{0.20f};
                // 保持锚点跟随目标中心变化的时间常数，单位：秒。
                float anchor_follow_time_sec{0.02f};
                // 目标中心滤波时间常数，单位：秒。
                float center_filter_time_sec{0.50f};
                // 积分增益，单位：(rad/s)/(rad·s)。
                float ki_radps_per_rad_sec{0.25f};
                // 积分项输出限幅，单位：rad/s。
                float integral_limit_radps{0.25f};
                // 积分项释放时间，单位：秒。
                float integral_release_time_sec{0.20f};
                // 开始抑制积分的底盘速度阈值，单位：rad/s。
                float integral_speed_start_radps{0.03f};
                // 积分抑制达到满权重的底盘速度，单位：rad/s。
                float integral_speed_full_radps{0.12f};
            } hold{};

            // 常规底盘运动趋势估计器参数。
            struct Trend
            {
                // 常规运动趋势估计器带宽，单位：rad/s。
                float bandwidth_rad_s{3.0f};
                // 常规运动趋势估计器阻尼比，无量纲。
                float damping{1.0f};
                // 常规运动趋势估计器的角加速度限幅，单位：rad/s²。
                float accel_limit_radps2{8.0f};
            } trend{};

            // 快速底盘运动趋势估计器带宽。
            struct FastTrend
            {
                // 快速运动趋势估计器带宽，单位：rad/s。
                float bandwidth_rad_s{3.0f};
            } fast_trend{};
        } base{};

        // 目标 q 轴居中、软限幅及短时预测参数。
        struct Q
        {
            // q 轴角度偏差到居中角速度的比例增益，单位：(rad/s)/rad。
            float centering_kp_radps_per_rad{2.5f};
            // 软限幅开始生效的 q 轴偏差，单位：rad。
            float soft_start_rad{0.35f};
            // 软限幅达到满权重的 q 轴偏差，单位：rad。
            float soft_full_rad{0.42f};
            // q 轴目标居中位置，单位：rad。
            float center_rad{0.0f};
            // q 轴短时预测时长，单位：秒。
            float predict_time_sec{0.030f};
        } q{};

        // 目标捕获判定阈值，以及条件持续时间。
        struct Acquire
        {
            // 目标捕获时允许的 q 轴角度误差，单位：rad。
            float q_tol_rad{0.08f};
            // 目标捕获时允许的底盘角速度，单位：rad/s。
            float base_speed_tol_radps{0.25f};
            // 目标捕获时允许的 yaw 轴角速度，单位：rad/s。
            float yaw_speed_tol_radps{0.30f};
            // 满足捕获条件需持续的时间，单位：毫秒。
            std::uint32_t acquire_hold_ms{80U};
        } acquire{};

        // 目标丢失后保持锁定状态的宽限时间。
        struct Lock
        {
            // 目标丢失后继续保持锁定状态的宽限时间，单位：毫秒。
            std::uint32_t grace_ms{1000U};
        } lock{};
    } vision{};

    // 双 yaw 轴相对运动滤波与反作用转矩补偿参数。
    struct DualYaw
    {

        // 双 yaw 轴相对角速度滤波时间常数。
        struct Filter
        {
            float relative_speed_tau_sec{0.025f};
        } filter{};

        // 双 yaw 轴反作用转矩补偿增益、滤波及输出限制。
        struct Reaction
        {
            float comp_gain{1.9f};
            float filter_cutoff_hz{70.0f};
            float torque_limit_nm{1.0f};
            float torque_slew_rate_nmps{2000.0f};
            float coupled_inertia_kgm2{0.007f};
            float inertia_gain{-1.0f};
            float rel_speed_filter_cutoff_hz{30.0f};
            float rel_accel_filter_cutoff_hz{15.0f};
        } reaction{};
    } dual_yaw{};

    // 巡逻模式开关、底盘巡航速度和双轴扫描参数。
    struct Sentry
    {
        bool enabled{true}; // 是否启用巡逻模式。
        float base_cruise_speed_radps{1.7f}; // 底盘巡航角速度。

        // 巡逻 yaw 正弦扫描的中心、幅值、频率和角度范围。
        struct Yaw
        {
            float center_rad{0.0f};
            float amplitude_rad{0.209439510f};
            float frequency_hz{3.0f};
            float min_rad{-0.351799388f};
            float max_rad{0.351799388f};

            // 巡逻轴的编码器角度环 PID。
            struct EncoderAnglePid
            {
                float kp{12.0f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max{30.0f};
                float output_min{-30.0f};
                float integral_max{1.0f};
                float integral_separation_rad{0.07f};
                float integral_reset_rad{0.15f};
            } encoder_angle_pid{};

            // yaw 编码器速度环 PID，输出为转矩。
            struct EncoderSpeedPid
            {
                float kp{0.185f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max{1.223f};
                float output_min{-1.223f};
                float integral_max{0.0f};
            } encoder_speed_pid{};
        } yaw{};

        // 巡逻 pitch 正弦扫描参数及反馈源选择。
        struct Pitch
        {
            float center_rad{0.0f};
            float amplitude_rad{0.087266463f};
            float frequency_hz{3.0f};
            float min_rad{-0.30453293f};
            float max_rad{0.30453293f};
            bool use_imu_fb{true}; // true 使用 IMU，false 使用电机编码器反馈。

            // pitch 编码器角度环 PID，输出为参考角速度。
            struct EncoderAnglePid
            {
                float kp{15.0f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max_radps{26.179938780f};
                float output_min_radps{-26.179938780f};
                float integral_max_rad_s{1.745329252f};
                float integral_separation_rad{0.087266463f};
                float integral_reset_rad{0.261799388f};
            } encoder_angle_pid{};
        } pitch{};
    } sentry{};

    // roll 轴目标角、阻抗控制、驱动限制和轨迹规划参数。
    struct Roll
    {

        // 常规姿态与变形姿态的 roll 目标角。
        struct Reference
        {
            float normal_rad{2.98f};
            float transform_rad{4.869f};
        } reference{};

        // roll 阻抗控制的刚度、阻尼和变形比例。
        struct Impedance
        {
            float stiffness_nm_per_rad{32.0f};
            float damping_nms_rad{2.3f};
            float transform_ratio{0.6f};
        } impedance{};

        // roll 电机 MIT 模式速度和转矩限制。
        struct Mit
        {
            float speed_limit_radps{30.0f};
            float torque_limit_nm{12.5f};
            float torque_offset_nm{0.0f};
        } mit{};

        // roll S 曲线轨迹的速度、加速度和加加速度上限。
        struct Trajectory
        {
            float max_speed_radps{10.0f};
            float max_accel_radps2{35.0f};
            float max_jerk_radps3{400.0f};
        } trajectory{};

        // roll 重力前馈系数及补偿相位。
        struct Gravity
        {
            float k_nm{1.3f};
            float offset_rad{-1.457349925f};
        } gravity{};
    } roll{};

    // 变形动作目标、到位判定、超时保护及各轴控制参数。
    struct Transform
    {

        // 变形时底盘、yaw 和 pitch 轴的目标角。
        struct Reference
        {
            float base_rad{4.6881f};
            float yaw_rad{5.275f};
            float pitch_rad{5.480f};
        } reference{};

        // 判定各轴到位所需的角度误差和速度阈值。
        struct Arrival
        {
            float angle_tolerance_rad{0.087453293f};
            float speed_tolerance_radps{0.187266463f};
        } arrival{};
        std::uint16_t stable_cycles{10U};
        std::uint32_t phase_timeout_ms{3000U};

        // 变形时底盘 yaw 角度环和 MIT 阻尼。
        struct Base
        {

            // 底盘变形角度环 PID。
            struct AnglePid
            {
                float kp{5.0f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max{10.0f};
                float output_min{-10.0f};
                float integral_max{5.0f};
                float integral_separation_rad{0.07f};
                float integral_reset_rad{0.15f};
            } angle_pid{};
            float mit_kd{0.9f};
        } base{};

        // 变形时 yaw 轴角度环和速度环。
        struct Yaw
        {

            // yaw 变形角度环 PID。
            struct AnglePid
            {
                float kp{8.0f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max{30.0f};
                float output_min{-30.0f};
                float integral_max{1.0f};
                float integral_separation_rad{0.07f};
                float integral_reset_rad{0.15f};
            } angle_pid{};

            // yaw 变形速度环 PID，输出为转矩。
            struct SpeedPid
            {
                float kp{0.15f};
                float ki{0.0f};
                float kd{0.0f};
                float output_max{2.223f};
                float output_min{-2.223f};
                float integral_max{0.0f};
            } speed_pid{};
        } yaw{};

        // 变形时 pitch 角度环和 MIT 阻尼。
        struct Pitch
        {

            // pitch 变形角度环 PID，输出为参考角速度。
            struct AnglePid
            {
                float kp{20.0f};
                float ki{1.0f};
                float kd{0.0f};
                float output_max_radps{26.179938780f};
                float output_min_radps{-26.179938780f};
                float integral_max_rad_s{1.745329252f};
                float integral_separation_rad{0.087266463f};
                float integral_reset_rad{0.261799388f};
            } angle_pid{};
            float mit_kd{0.9f};
        } pitch{};
    } transform{};

    // 控制模式切换时参考值的平滑过渡时间。
    struct Mode
    {
        std::uint32_t blend_ms{200U};
    } mode{};
};

inline constexpr GimbalConfig gimbal{};

static_assert(gimbal.sentry.yaw.min_rad < gimbal.sentry.yaw.max_rad, "SENTRY yaw limits must be ordered");
static_assert(gimbal.sentry.pitch.min_rad < gimbal.sentry.pitch.max_rad, "SENTRY pitch limits must be ordered");
static_assert(gimbal.transform.stable_cycles > 0U && gimbal.transform.phase_timeout_ms > 0U,
              "TRANSFORM confirmation and timeout must be non-zero");
} // namespace AppConfig::OnmiGimbal
