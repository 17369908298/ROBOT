#pragma once
#include <cstdint>
#include "app/omni_reference_config.hpp"
#include "user/core/Alg/Omni/roll_control.hpp"

// All excitation units: rad, rad/s, N*m, seconds. Rebuild after editing.
namespace YawId::Config {
inline constexpr unsigned control_hz = 1000, telemetry_divider = 5;
inline constexpr unsigned vofa_baud = 115200;
inline constexpr unsigned vofa_channels = 10, vofa_schema = 5;
inline constexpr std::uint16_t yaw_feedback_id = 0x206;
static_assert(yaw_feedback_id >= 0x205 && yaw_feedback_id <= 0x208,
              "This current group 0x1FE supports GM6020 IDs 1..4 / feedback 0x205..0x208");
inline constexpr float torque_constant = .741f, current_max_a = 3.0f;
inline constexpr float torque_limit_nm = 2.2f, torque_slew_nm_s = 160.0f;
inline constexpr float soft_kp = 1.0f, soft_kd = .35f, soft_limit_nm = 1.8f;
inline constexpr float hard_angle_rad = .45f, hard_speed_rad_s = 30.0f;
inline constexpr float governor_start_rad = .34f, governor_stop_rad = .42f;
inline constexpr float governor_prediction_s = .10f;
inline constexpr float imu_yaw_sign = 1.0f;
inline constexpr unsigned imu_yaw_axis = 2;
inline constexpr unsigned sensor_timeout_ms = 50, remote_timeout_ms = 100;
inline constexpr unsigned watchdog_us = 8000, roll_feedback_timeout_ms = 300;
inline constexpr float startup_settle_s = .3f, preparing_timeout_s = 20.0f;

enum class Mode : unsigned { Sine=1, Chirp=2, Prbs=3, SteppedSine=4 };
inline constexpr Mode mode = Mode::SteppedSine;
// Step dwell is counted in cycles, giving low frequencies enough real time.
inline constexpr float step_hz[] = {.2f, .5f, 1.f, 2.f, 3.f, 4.f, 6.f};
inline constexpr float step_amplitude_scale[] = {.5f, .75f, 1.f};
inline constexpr float step_cycles = 8.f, step_fade_cycles = 1.f;
inline constexpr float amplitude_low_nm = .25f, amplitude_high_nm = 2.1f;
inline constexpr float amplitude_full_hz = 5.f;
inline constexpr float sine_hz = 1.0f, sine_amplitude_nm = .3f;
inline constexpr float duration_s = 180.f, ramp_s = 3.f;
inline constexpr float chirp_start_hz = .2f, chirp_end_hz = 6.f, chirp_exponent = 2.f;
inline constexpr float prbs_amplitude_nm = .3f, prbs_switch_s = .1f;
inline constexpr std::uint32_t prbs_seed = 0x12345678U;

// Local frozen snapshot of SENTRY/GIMBAL/RtosTask/gimbal_config.hpp.
inline constexpr const auto& omni = AppConfig::OnmiGimbal::gimbal;
inline constexpr bool roll_enabled = true, pitch_enabled = true;
inline constexpr bool wait_roll_settled = true;
inline constexpr std::uint16_t roll_rx_id = 0x07, roll_tx_id = 0x06;
inline constexpr std::uint16_t pitch_rx_id = 0x05, pitch_tx_id = 0x04;
inline constexpr std::uint16_t big_yaw_tx_id = 0x02;
inline constexpr float roll_target_rad = omni.roll.reference.normal_rad;
inline constexpr ALG::Omni::RollImpedanceController::Config roll_controller = {
    omni.roll.impedance.stiffness_nm_per_rad, omni.roll.impedance.damping_nms_rad,
    omni.roll.impedance.transform_ratio, omni.roll.mit.speed_limit_radps,
    omni.roll.mit.torque_limit_nm, omni.roll.gravity.k_nm, omni.roll.gravity.offset_rad,
    omni.roll.mit.torque_offset_nm, omni.roll.trajectory.max_speed_radps,
    omni.roll.trajectory.max_accel_radps2, omni.roll.trajectory.max_jerk_radps3};
// Pitch is a manual setup axis; no automatic pitch sweep during yaw capture.
inline constexpr float pitch_min_rad = -.187f, pitch_max_rad = .678f;
inline constexpr float pitch_manual_rate = .5f, pitch_kp = 20.f, pitch_kd = 1.6f;
inline constexpr float pitch_speed_limit = 10.f, pitch_torque_limit = 4.f;
inline constexpr float pitch_gravity_nm = 1.0f, pitch_gravity_offset_rad = 0.0f;
static_assert(control_hz==1000 && telemetry_divider>0, "TIM6 control period is 1 ms");
static_assert(vofa_channels==10 && vofa_schema==5, "Compact VOFA schema/layout must agree");
static_assert((vofa_channels*4+4)*10*control_hz/telemetry_divider < vofa_baud,
              "VOFA data rate exceeds UART capacity");
static_assert(imu_yaw_axis<3 && (imu_yaw_sign==1 || imu_yaw_sign==-1), "Invalid IMU axis/sign");
static_assert(torque_constant>0 && current_max_a>0 && torque_limit_nm<=torque_constant*current_max_a,
              "Invalid current/torque conversion or torque limit");
static_assert(0<governor_start_rad && governor_start_rad<governor_stop_rad &&
              governor_stop_rad<hard_angle_rad, "Invalid travel limits");
static_assert(step_fade_cycles>0 && step_cycles>2*step_fade_cycles && prbs_seed!=0 && prbs_switch_s>0,
              "Invalid excitation settings");
}
