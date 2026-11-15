#pragma once
#include <cstdint>

namespace AppConfig::OnmiShooter
{

// 发射机构各模块的可调参数。拨盘角度用 deg、转速用 rpm；时间单位见字段后缀，raw 为驱动原始计数。
struct ShooterConfig
{
    float dt_sec{0.001f}; // 发射控制周期，单位 s。

    // 遥控滚轮触发、释放滞回及模式拨杆。
    struct Input
    {
        float scroll_deadband{0.6f};
        float scroll_release_threshold_scale{0.75f}; // 释放阈值 = 触发阈值 × 此比例，避免临界抖动。

        // 遥控器 S1/S2 档位：1 为上档、2 为下档、3 为中档。
        struct Switches
        {
            std::uint8_t stop_s1{2U};
            std::uint8_t stop_s2{2U};
            std::uint8_t vis_s1{3U};
            std::uint8_t vis_s2{3U};
            std::uint8_t continuous_s1{1U};
            std::uint8_t continuous_s2{3U};
        } switches{};
    } input{};

    // 三摩擦轮的目标转速、就绪/停转阈值和速度环。
    struct Friction
    {
        float left_ref_rpm{8000.0f};
        float right_ref_rpm{-8000.0f};
        float third_ref_rpm{8000.0f};
        float ready_error_rpm{1500.0f}; // 沿目标方向达到目标转速减此裕量后判就绪。
        float stop_error_rpm{30.0f}; // 绝对转速不超过此值时判停转。

        // 摩擦轮速度环 PID：输入 rpm，输出电流命令原始计数；积分限幅为 rpm·s。
        struct SpeedPid
        {
            float kp{12.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max{10000.0f};
            float output_min{-10000.0f};
            float integral_max{100.0f};
        } speed_pid{};
    } friction{};

    // 拨盘传动换算、角度/速度串级控制和堵转恢复。
    struct Dial
    {
        float direction{1.0f}; // 参考角增量的方向系数；实际更新时减去该增量。
        float reduction_ratio{10.0f}; // 拨盘每槽角度到控制轴角度的减速比。
        float gear_count{9.0f}; // 拨盘每圈的弹槽数。
        float external_reduction{1.0f}; // 编码器轴角度/速度到控制轴的外置减速比。

        // 拨盘角度环 PID：输入控制轴 deg，输出控制轴 rpm；积分限幅为 deg·s。
        struct AnglePid
        {
            float kp{4.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max_rpm{8000.0f};
            float output_min_rpm{-8000.0f};
            float integral_max_deg_s{0.0f};
        } angle_pid{};

        // 拨盘速度环 PID：输入控制轴 rpm，输出电流命令原始计数；积分限幅为 rpm·s。
        struct SpeedPid
        {
            float kp{1.0f};
            float ki{0.0f};
            float kd{0.0f};
            float output_max{2048.0f};
            float output_min{-2048.0f};
            float integral_max{2500.0f};
        } speed_pid{};

        // 堵转检测与反转、等待、重初始化及重试参数；角度/速度均对应控制轴。
        struct Recovery
        {
            bool enabled{false};
            float stall_error_deg{120.0f};
            float stall_speed_rpm{4.0f};
            float stall_current_a{1.0f};
            float stall_progress_deg{3.0f}; // 正向进展达到此角度后重新开始堵转计时。
            std::uint32_t stall_hold_ms{200U};
            std::uint32_t reverse_time_ms{40U};
            std::uint32_t settle_time_ms{15U};
            std::int16_t reverse_current_raw{900}; // 反转电流命令幅值，使用驱动原始计数。
            std::uint32_t reinit_timeout_ms{1000U};
            std::uint32_t retry_interval_ms{200U};
            std::uint8_t max_retry{3U};
        } recovery{};
    } dial{};

    // 手动/视觉射速及手动滚轮持续触发的确认时间。
    struct Fire
    {
        float manual_rate_hz{10.0f};
        float auto_rate_hz{15.0f};
        std::uint32_t continuous_hold_ms{1U};
    } fire{};

    // 热量限制、裁判数据选择及本地热量估计。
    struct Heat
    {
        bool enabled{false};
        bool use_referee{true};
        std::uint32_t referee_timeout_ms{500U};
        float limit{100.0f}; // 裁判数据不可用时的热量上限。
        float cooling_rate{12.0f}; // 备用冷却速率，单位为热量/s。
        float per_shot{10.0f}; // 每发弹丸增加的热量。
        float free_margin_ratio{0.50f}; // 可用热量余量比例高于此值时允许全射速。
        float stop_margin_ratio{0.10f}; // 可用热量余量比例低于此值时停止发射。

        // 利用左右摩擦轮电流突增检测发弹；电流窗口按采样点求和，不乘 dt。
        struct Detection
        {
            std::uint8_t window_size{20U}; // 滑动窗口的采样点数。
            std::uint32_t shot_cooldown_ms{80U}; // 两次发弹检测的最小间隔。
            std::uint32_t startup_ignore_ms{1500U}; // 摩擦轮启动后的检测屏蔽时间。
            float integral_threshold{8.0f}; // 扣除噪声后电流窗口和的触发阈值。
            float noise_threshold_a{1.5f}; // 左右轮绝对电流之和的噪声门限。
            float ref_start_threshold_rpm{1000.0f}; // 开始发弹检测所需的参考转速。
            float wheel_ready_speed_rpm{3000.0f}; // 两摩擦轮均超过此转速才累计电流。
        } detection{};
    } heat{};

    // 电机反馈和遥控连接的有效期。
    struct Timeouts
    {
        std::uint32_t motor_fb_ms{150U};
        std::uint32_t remote_connection_ms{400U};
    } timeouts{};

    // 视觉请求有效期、丢失锁定保持时间和发射许可开关。
    struct Vision
    {
        std::uint32_t request_timeout_ms{150U};
        std::uint32_t lock_hold_ms{600U};
        bool enable_fire{true};
    } vision{};

    // 大 yaw 转速过高时禁止拨弹；降速后持续满足条件才重新放行。
    struct BaseFireGate
    {
        float block_speed_radps{1.20f};
        float release_speed_radps{1.00f};
        std::uint32_t release_hold_ms{60U};
        std::uint32_t status_timeout_ms{100U};
    } base_fire_gate{};
};

inline constexpr ShooterConfig shooter{};

static_assert(shooter.fire.continuous_hold_ms > 0U, "shooter hold gates must be non-zero");
static_assert(shooter.base_fire_gate.release_speed_radps < shooter.base_fire_gate.block_speed_radps,
              "big-yaw fire release speed must be below block speed");
static_assert(shooter.heat.free_margin_ratio >= shooter.heat.stop_margin_ratio,
              "heat free margin must not be below stop margin");
} // namespace AppConfig::OnmiShooter
