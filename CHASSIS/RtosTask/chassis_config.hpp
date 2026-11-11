#pragma once
#include <cstdint>
// 控制参数沿用 Omni 原始值；变量后缀标示 rad、rad/s、rpm、A 或原始命令量。

namespace AppConfig
{
namespace OnmiChassis
{
inline constexpr float max_planar_speed = 2.0f;
inline constexpr float max_vw = 5.0f;
inline constexpr float wheel_radius = 0.06f;
inline constexpr float chassis_radius = 0.275f;
inline constexpr float max_wheel_speed_rpm = 420.0f;
inline constexpr float sbus_analog_deadband = 0.05f;
inline constexpr std::uint32_t remote_connection_timeout_ms = 300U;
inline constexpr std::uint32_t motion_command_timeout_ms = 50U;
inline constexpr std::uint32_t host_request_timeout_ms = 300U;
inline constexpr std::uint32_t gimbal_motion_gate_timeout_ms = 100U;
inline constexpr std::uint32_t motor_fb_soft_timeout_ms = 100U;
inline constexpr std::uint32_t motor_fb_hard_timeout_ms = 250U;
inline constexpr std::uint32_t gate_log_interval_ms = 1000U;
inline constexpr bool motor_online_log_enabled = true;
inline constexpr std::uint32_t motor_online_log_interval_ms = 1000U;
inline constexpr float wheel_azimuth_fl = 0.78539816339f;
inline constexpr float wheel_azimuth_fr = -0.78539816339f;
inline constexpr float wheel_azimuth_bl = 2.35619449019f;
inline constexpr float wheel_azimuth_br = -2.35619449019f;
inline constexpr float wheel_direction_fl = 2.35619449019f;
inline constexpr float wheel_direction_fr = 0.78539816339f;
inline constexpr float wheel_direction_bl = -2.35619449019f;
inline constexpr float wheel_direction_br = -0.78539816339f;
inline constexpr float wheel_sign_fl = -1.0f;
inline constexpr float wheel_sign_fr = -1.0f;
inline constexpr float wheel_sign_bl = -1.0f;
inline constexpr float wheel_sign_br = -1.0f;
inline constexpr float wheel_pid_kp = 1800.0f;
inline constexpr float wheel_pid_ki = 0.0f;
inline constexpr float wheel_pid_kd = 0.0f;
inline constexpr float wheel_pid_output_max = 16384.0f;
inline constexpr float wheel_pid_output_min = -16384.0f;
inline constexpr float wheel_pid_integral_max = 104.71976f;
inline constexpr bool force_ff_enabled = true;
inline constexpr float chassis_mass_kg = 16.0f;
inline constexpr float chassis_yaw_inertia_kgm2 = 0.43f;
inline constexpr float wheel_equivalent_inertia_kgm2 = 0.002f;
inline constexpr float wheel_coulomb_friction_nm = 0.0f;
inline constexpr float wheel_viscous_friction_nm_per_radps = 0.0f;
inline constexpr float wheel_friction_smooth_radps = 0.15f;
inline constexpr float force_ff_inertial_fade_error_radps = 2.0f;
inline constexpr float force_ff_linear_max_accel_mps2 = 5.0f;
inline constexpr float force_ff_linear_max_decel_mps2 = 8.0f;
inline constexpr float force_ff_linear_max_jerk_mps3 = 20.0f;
inline constexpr float force_ff_angular_max_accel_radps2 = 8.0f;
inline constexpr float force_ff_angular_max_decel_radps2 = 10.0f;
inline constexpr float force_ff_angular_max_jerk_radps3 = 50.0f;
inline constexpr float c620_current_max_a = 20.0f;
inline constexpr float c620_command_max_raw = 16384.0f;
inline constexpr float wheel_output_torque_constant_nm_per_a = 0.24628237f;
inline constexpr bool power_control_enabled = true;
inline constexpr float power_model_coeffs[6] = {
    0.614597983f, 0.005554523f, 0.004789858f, 0.017675351f, 0.114952711f, 0.000008669f,
};
inline constexpr bool power_model_absolute_speed_loss = true;
inline constexpr float power_fixed_w = 0.0f;
inline constexpr float power_fallback_w = 50.0f;
inline constexpr float power_margin_w = 5.0f;
inline constexpr std::uint32_t power_referee_timeout_ms = 500U;
inline constexpr float power_buffer_low_j = 20.0f;
inline constexpr float power_buffer_recover_j = 30.0f;
inline constexpr float power_low_buffer_scale = 0.8f;
inline constexpr std::uint8_t disable_s1 = 2U;
inline constexpr std::uint8_t disable_s2 = 2U;
inline constexpr std::uint8_t follow_s1_a = 3U;
inline constexpr std::uint8_t follow_s2_a = 1U;
inline constexpr std::uint8_t auto_s1 = 3U;
inline constexpr std::uint8_t auto_s2 = 3U;
inline constexpr std::uint8_t navigation_s1 = auto_s1;
inline constexpr std::uint8_t navigation_s2 = auto_s2;
inline constexpr std::uint8_t shooter_scroll_s1 = 1U;
inline constexpr std::uint32_t manual_gimbal_yaw_timeout_ms = 100U;
inline constexpr std::uint32_t manual_odometry_timeout_ms = 100U;
inline constexpr float manual_gimbal_yaw_zero_rad = 4.688f;
inline constexpr float manual_gimbal_yaw_sign = 1.0f;
inline constexpr float manual_gimbal_angle_comp_rad = 0.0f;
inline constexpr float manual_gimbal_phase_comp_rad = 0.0f;
inline constexpr float manual_dynamic_k_base = 0.00f;
inline constexpr float follow_yaw_pid_kp = 3.0f;
inline constexpr float follow_yaw_pid_ki = 0.0f;
inline constexpr float follow_yaw_pid_kd = 0.0f;
inline constexpr float follow_yaw_pid_output_max = max_vw;
inline constexpr float follow_yaw_pid_output_min = -max_vw;
inline constexpr float follow_yaw_pid_integral_max = 0.0f;
inline constexpr float follow_yaw_deadband_rad = 0.03f;
inline constexpr float follow_yaw_half_range_rad = 1.57079632679f;
} // namespace OnmiChassis
namespace NavHostComm
{
inline constexpr std::uint32_t gimbal_yaw_timeout_ms = OnmiChassis::manual_gimbal_yaw_timeout_ms;
inline constexpr float gimbal_yaw_zero_rad = OnmiChassis::manual_gimbal_yaw_zero_rad;
inline constexpr float gimbal_yaw_sign = OnmiChassis::manual_gimbal_yaw_sign;
} // namespace NavHostComm
namespace OnmiChassisOdometry
{
inline constexpr std::uint32_t motor_fb_timeout_ms = 100U;
inline constexpr std::uint32_t heading_timeout_ms = 100U;
inline constexpr std::uint32_t min_update_dt_ms = 1U;
inline constexpr std::uint32_t max_update_dt_ms = 100U;
inline constexpr float max_linear_speed_mps = 3.0f;
inline constexpr float max_angular_speed_radps = 12.0f;
inline constexpr float max_heading_jump_rad = 1.0f;
inline constexpr float linear_deadband_mps = 0.01f;
inline constexpr float angular_deadband_radps = 0.01f;
inline constexpr float stationary_enter_linear_mps = 0.03f;
inline constexpr float stationary_exit_linear_mps = 0.06f;
inline constexpr float stationary_enter_angular_radps = 0.03f;
inline constexpr float stationary_exit_angular_radps = 0.06f;
static_assert(OnmiChassis::motor_fb_soft_timeout_ms < OnmiChassis::motor_fb_hard_timeout_ms,
              "chassis motor soft timeout must be below hard timeout");
static_assert(min_update_dt_ms <= max_update_dt_ms, "odometry update dt limits must be ordered");
static_assert(stationary_enter_linear_mps < stationary_exit_linear_mps &&
                  stationary_enter_angular_radps < stationary_exit_angular_radps,
              "stationary gate must have hysteresis");
} // namespace OnmiChassisOdometry
} // namespace AppConfig

namespace chassis_cfg = AppConfig::OnmiChassis;
namespace odom_cfg = AppConfig::OnmiChassisOdometry;
namespace nav_cfg = AppConfig::NavHostComm;
