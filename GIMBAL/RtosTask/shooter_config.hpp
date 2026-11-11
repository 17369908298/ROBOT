#pragma once
#include <cstdint>

namespace AppConfig
{
namespace OnmiShooter
{
inline constexpr float dt_sec = 0.001f;
inline constexpr float scroll_deadband = 0.6f;
inline constexpr float scroll_release_threshold_scale = 0.75f;
inline constexpr float dial_direction = 1.0f;
inline constexpr float dial_reduction_ratio = 10.0f;
inline constexpr float dial_gear_count = 9.0f;
inline constexpr float dial_external_reduction = 1.0f;
inline constexpr float fric_left_ref_rpm = 8000.0f;
inline constexpr float fric_right_ref_rpm = -8000.0f;
inline constexpr float fric_third_ref_rpm = 8000.0f;
inline constexpr float fric_ready_error_rpm = 1500.0f;
inline constexpr float fric_stop_error_rpm = 30.0f;
inline constexpr float fric_pid_kp = 12.0f;
inline constexpr float fric_pid_ki = 0.0f;
inline constexpr float fric_pid_kd = 0.0f;
inline constexpr float fric_pid_output_max = 10000.0f;
inline constexpr float fric_pid_output_min = -10000.0f;
inline constexpr float fric_pid_integral_max = 100.0f;
inline constexpr float dial_angle_pid_kp = 20.0f;
inline constexpr float dial_angle_pid_ki = 0.0f;
inline constexpr float dial_angle_pid_kd = 0.0f;
inline constexpr float dial_angle_pid_output_max = 8000.0f;
inline constexpr float dial_angle_pid_output_min = -8000.0f;
inline constexpr float dial_angle_pid_integral_max = 0.0f;
inline constexpr float dial_speed_pid_kp = 1.0f;
inline constexpr float dial_speed_pid_ki = 0.0f;
inline constexpr float dial_speed_pid_kd = 0.0f;
inline constexpr float dial_speed_pid_output_max = 2048.0f;
inline constexpr float dial_speed_pid_output_min = -2048.0f;
inline constexpr float dial_speed_pid_integral_max = 2500.0f;
inline constexpr float manual_fire_rate_hz = 10.0f;
inline constexpr float auto_fire_rate_hz = 10.0f;
inline constexpr std::uint32_t continuous_hold_threshold_ms = 10U;
inline constexpr bool heat_limit_enable = false;
inline constexpr bool heat_use_referee = true;
inline constexpr std::uint32_t heat_referee_timeout_ms = 500U;
inline constexpr float heat_limit = 100.0f;
inline constexpr float heat_cooling_rate = 12.0f;
inline constexpr float heat_per_shot = 10.0f;
inline constexpr float heat_free_margin_ratio = 0.50f;
inline constexpr float heat_stop_margin_ratio = 0.10f;
inline constexpr std::uint8_t heat_window_size = 20U;
inline constexpr std::uint32_t heat_shot_cooldown_ms = 80U;
inline constexpr std::uint32_t heat_startup_ignore_ms = 1500U;
inline constexpr float heat_integral_threshold = 8.0f;
inline constexpr float heat_noise_threshold = 1.5f;
inline constexpr float heat_ref_start_threshold = 1000.0f;
inline constexpr float heat_wheel_ready_speed = 3000.0f;
inline constexpr std::uint32_t motor_fb_timeout_ms = 150U;
inline constexpr std::uint32_t remote_connection_timeout_ms = 400U;
inline constexpr std::uint32_t vis_request_timeout_ms = 150U;
inline constexpr std::uint32_t vis_lock_hold_ms = 600U;
inline constexpr float base_fire_block_speed_radps = 1.20f;
inline constexpr float base_fire_release_speed_radps = 1.00f;
inline constexpr std::uint32_t base_fire_release_hold_ms = 60U;
inline constexpr std::uint32_t base_fire_status_timeout_ms = 100U;
inline constexpr bool dial_stall_recovery_enable = false;
inline constexpr float dial_stall_error_deg = 120.0f;
inline constexpr float dial_stall_speed_rpm = 4.0f;
inline constexpr float dial_stall_current_a = 1.0f;
inline constexpr float dial_stall_progress_deg = 3.0f;
inline constexpr std::uint32_t dial_stall_hold_ms = 200U;
inline constexpr std::uint32_t dial_reverse_time_ms = 40U;
inline constexpr std::uint32_t dial_settle_time_ms = 15U;
inline constexpr std::int16_t dial_reverse_current = 900;
inline constexpr std::uint32_t dial_reinit_timeout_ms = 1000U;
inline constexpr std::uint32_t dial_retry_interval_ms = 200U;
inline constexpr std::uint8_t dial_recovery_max_retry = 3U;
inline constexpr std::uint8_t stop_s1 = 2U;
inline constexpr std::uint8_t stop_s2 = 2U;
inline constexpr std::uint8_t vis_s1 = 3U;
inline constexpr std::uint8_t vis_s2 = 3U;
inline constexpr std::uint8_t continuous_s1 = 1U;
inline constexpr std::uint8_t continuous_s2 = 3U;
inline constexpr bool enable_vis_fire = true;
static_assert(continuous_hold_threshold_ms > 0U, "shooter hold gates must be non-zero");
static_assert(base_fire_release_speed_radps < base_fire_block_speed_radps,
              "big-yaw fire release speed must be below block speed");
static_assert(heat_free_margin_ratio >= heat_stop_margin_ratio,
              "heat free margin must not be below stop margin");
} // namespace OnmiShooter
} // namespace AppConfig
