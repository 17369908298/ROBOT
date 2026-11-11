#include "RtosTask/gimbal_control.hpp"
#include "communication/robot_data.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using OnmiGimbalDetail::clamp_by_abs_limit;
using OnmiGimbalDetail::DT7_CH_LEFT_SCROLL;
using OnmiGimbalDetail::DT7_CH_LEFT_X;
using OnmiGimbalDetail::DT7_CH_LEFT_Y;
using OnmiGimbalDetail::DT7_CH_RIGHT_SCROLL;
using OnmiGimbalDetail::DT7_CH_RIGHT_X;
using OnmiGimbalDetail::DT7_CH_RIGHT_Y;
using OnmiGimbalDetail::DT7_CH_S1;
using OnmiGimbalDetail::DT7_CH_S2;
using OnmiGimbalDetail::shortest_setpoint_rad;

bool GimbalCtrl::update_control(OnmiGimbalMode mode, const Sentry::IMUData &main_imu,
                                const Sentry::IMUData &base_imu,
                                const Sentry::MotorFeedback &yaw_fb)
{
    // 更新双 yaw 速度，再按当前模式计算参考量和各轴输出。
    if (mode == OnmiGimbalMode::ARMING)
    {
        reset_control_outputs();
        return true;
    }
    if (!active_control_mode(mode))
    {
        reset_control_outputs();
        return false;
    }

    const float main_imu_gyro_z_radps = Sentry::Utils::deg_to_rad(main_imu.gyro_dps[2]);
    const float base_imu_gyro_radps =
        Sentry::Utils::deg_to_rad(base_imu.gyro_dps[cfg::gimbal.input.base_imu_yaw_axis]);
    if (!std::isfinite(yaw_.angle_rad) || !std::isfinite(base_.angle_rad) ||
        !std::isfinite(pitch_.angle_rad) || !std::isfinite(main_imu_gyro_z_radps) ||
        !std::isfinite(base_imu_gyro_radps))
    {
        reset_control_outputs();
        return false;
    }

    yaw_.offset_rad = current_yaw_offset_rad();

    base_.loop.speed_radps = base_.speed_lpf.update(base_imu_gyro_radps, DT_SEC);
    dual_.state.raw_speed_radps = main_imu_gyro_z_radps - base_imu_gyro_radps;
    const float q_dot_alpha = dual_.alpha_cache.get(cfg::gimbal.dual_yaw.filter.relative_speed_tau_sec, DT_SEC);
    dual_.state.speed_radps +=
        q_dot_alpha * (dual_.state.raw_speed_radps - dual_.state.speed_radps);

    if (mode == OnmiGimbalMode::MANUAL)
    {

        const float q_deadband = std::fabs(cfg::gimbal.base.manual.q_deadband_rad);
        const float q_effective = std::copysign(
            std::fmax(std::fabs(yaw_.offset_rad) - q_deadband, 0.0f), yaw_.offset_rad);
        manual_.q_rad = q_effective;
        const float physical_speed_ref = clamp_speed_radps(
            cfg::gimbal.base.manual.q_kp_radps_per_rad * q_effective, cfg::gimbal.base.manual.speed_limit_radps);
        const float signed_speed_ref = physical_speed_ref;
        manual_.raw_ref_radps = signed_speed_ref;

        if (!manual_.ready)
        {
            manual_.planner.reset(base_.loop.speed_radps, 0.0f);
            manual_.ready = true;
        }
        base_.loop.raw_ref_radps = signed_speed_ref;
        base_.loop.ref_radps = manual_.planner.update(signed_speed_ref, DT_SEC);

        base_.loop.torque_nm =
            base_.speed_pid.update(base_.loop.ref_radps, base_.loop.speed_radps, DT_SEC);
    }
    else if (mode == OnmiGimbalMode::SENTRY && cfg::gimbal.sentry.enabled)
    {

        manual_.ready = false;
        manual_.q_rad = 0.0f;
        manual_.raw_ref_radps = 0.0f;
        const float physical_speed_ref =
            clamp_speed_radps(cfg::gimbal.sentry.base_cruise_speed_radps, cfg::gimbal.base.mit.speed_limit_radps);
        base_.loop.raw_ref_radps = physical_speed_ref;
        base_.loop.ref_radps = base_.loop.raw_ref_radps;
        base_.loop.torque_nm =
            base_.speed_pid.update(base_.loop.ref_radps, base_.loop.speed_radps, DT_SEC);
    }
    else if (mode == OnmiGimbalMode::VISION)
    {

        manual_.ready = false;
        manual_.q_rad = 0.0f;
        manual_.raw_ref_radps = 0.0f;
        update_vis_base_planner(DT_SEC);
        const auto hold_output = vision_.angle_hold.update(
            base_.angle_rad, base_.loop.speed_radps,
            base_.angle_rad + vision_.q.request_rad - cfg::gimbal.vision.q.center_rad, base_.loop.ref_radps,
            vision_.q.center_weight, cfg::gimbal.vision.base.speed_limit_radps, DT_SEC);
        base_.loop.ref_radps = hold_output.reference_speed_radps;
        base_.loop.torque_nm =
            base_.speed_pid.update(base_.loop.ref_radps, base_.loop.speed_radps, DT_SEC);
    }
    else
    {

        manual_.ready = false;
        manual_.q_rad = 0.0f;
        manual_.raw_ref_radps = 0.0f;
        base_.loop.raw_ref_radps = 0.0f;
        base_.loop.ref_radps = 0.0f;
        base_.loop.torque_nm = 0.0f;
        dual_.state.reaction_nm = 0.0f;
    }

    const bool sentry_yaw_encoder_fb = mode == OnmiGimbalMode::SENTRY;

    ALG::Omni::TimePid &active_yaw_angle_pid =
        sentry_yaw_encoder_fb ? yaw_.sentry_angle_pid : yaw_.angle_pid;
    ALG::Omni::TimePid &active_yaw_speed_pid =
        sentry_yaw_encoder_fb ? yaw_.sentry_speed_pid : yaw_.speed_pid;
    if (sentry_yaw_encoder_fb &&
        (!std::isfinite(yaw_.motor.angle_rad) || !std::isfinite(yaw_.motor.speed_radps)))
    {
        reset_control_outputs();
        return false;
    }
    const float yaw_fb_rad = sentry_yaw_encoder_fb ? yaw_.motor.angle_rad : yaw_.angle_rad;
    const float yaw_ref_rad = shortest_setpoint_rad(yaw_.ref_rad, yaw_fb_rad);
    const float yaw_fb_rate_radps =
        sentry_yaw_encoder_fb ? yaw_.motor.speed_radps : main_imu_gyro_z_radps;
    if (mode == OnmiGimbalMode::VISION)
    {

        float omega_ref = vision_.ref.speed_radps;
        float alpha_ref = vision_.ref.accel_radps2;

        yaw_.loop.raw_ref_radps = omega_ref;
        yaw_.loop.ref_radps = omega_ref;
        yaw_.loop.speed_radps = main_imu_gyro_z_radps;
        ALG::Omni::YawAutoLqrEsoController::Feedback feedback{};
        feedback.theta_rad = yaw_fb_rad;
        feedback.omega_rad_s = main_imu_gyro_z_radps;
        feedback.tau_meas_nm = measured_yaw_torque_nm(yaw_fb);
        feedback.fb_ok = true;

        ALG::Omni::YawAutoLqrEsoController::Reference reference{};
        reference.theta_rad = yaw_ref_rad;
        reference.omega_rad_s = omega_ref;
        reference.alpha_rad_s2 = alpha_ref;

        const auto output = yaw_.auto_ctrl.update(feedback, reference, DT_SEC);
        yaw_.loop.auto_torque_nm = output.tau_cmd_nm;
        yaw_.loop.torque_nm =
            clamp_by_abs_limit(yaw_.loop.auto_torque_nm, yaw_.torque_converter.torque_limit_Nm());

        yaw_.loop.manual_torque_nm = 0.0f;
        yaw_.loop.cmd_raw = yaw_.torque_converter.torque_to_command(yaw_.loop.torque_nm);
    }
    else
    {
        yaw_.auto_ctrl.reset(yaw_fb_rad, yaw_fb_rate_radps);
        yaw_.loop.torque_nm = 0.0f;
        yaw_.loop.auto_torque_nm = 0.0f;
        const float yaw_angle_output_radps =
            clamp_speed_radps(active_yaw_angle_pid.update(yaw_ref_rad, yaw_fb_rad, DT_SEC),
                              cfg::gimbal.yaw.limits.speed_limit_radps);
        yaw_.loop.raw_ref_radps = yaw_angle_output_radps;
        if (!yaw_.loop.ref_ready)
        {
            yaw_.loop.ref_radps = 0.0f;
            yaw_.loop.ref_ready = true;
        }
        yaw_.loop.ref_radps = slew_limit(yaw_.loop.ref_radps, yaw_.loop.raw_ref_radps,
                                         cfg::gimbal.yaw.limits.ref_accel_limit_radps2 * DT_SEC);
        yaw_.loop.speed_radps = yaw_fb_rate_radps;
        yaw_.loop.manual_torque_nm =
            active_yaw_speed_pid.update(yaw_.loop.ref_radps, yaw_.loop.speed_radps, DT_SEC);
        yaw_.loop.torque_nm = yaw_.loop.manual_torque_nm;
        yaw_.loop.cmd_raw = yaw_.torque_converter.torque_to_command(yaw_.loop.manual_torque_nm);
    }

    if (mode == OnmiGimbalMode::VISION || mode == OnmiGimbalMode::SENTRY || false)
    {

        dual_.state.reaction_nm = dual_.reaction_ff.update_from_yaw_torque_and_rel_speed(
            yaw_.loop.torque_nm,

            dual_.state.raw_speed_radps, DT_SEC);
        base_.loop.torque_nm += dual_.state.reaction_nm;
    }
    else
    {

        dual_.state.reaction_nm = 0.0f;

        dual_.reaction_ff.reset(0.0f);
    }

    if (mode == OnmiGimbalMode::VISION || mode == OnmiGimbalMode::SENTRY)
    {

        const float torque_before_dist = base_.loop.torque_nm;
        base_.loop.dist_torque_nm = base_.disturbance.update(
            base_.loop.speed_radps, torque_before_dist, -dual_.state.reaction_nm, DT_SEC);
        base_.loop.torque_nm += base_.loop.dist_torque_nm;
    }
    else
    {

        base_.disturbance.reset(std::isfinite(base_.loop.speed_radps) ? base_.loop.speed_radps
                                                                      : 0.0f);
        base_.loop.dist_torque_nm = 0.0f;
    }

    const bool sentry_pitch_encoder_fb =
        mode == OnmiGimbalMode::SENTRY && !cfg::gimbal.sentry.pitch.use_imu_fb;
    ALG::Omni::TimePid &active_pitch_angle_pid =
        sentry_pitch_encoder_fb ? pitch_.sentry_angle_pid : pitch_.angle_pid;
    const float pitch_fb_rad = sentry_pitch_encoder_fb ? pitch_.motor.angle_rad : pitch_.angle_rad;
    const float pitch_control_ref_rad = sentry_pitch_encoder_fb
                                            ? shortest_setpoint_rad(pitch_.ref_rad, pitch_fb_rad)
                                            : pitch_.ref_rad;
    const float pitch_reference_speed_radps =
        active_pitch_angle_pid.update(pitch_control_ref_rad, pitch_fb_rad, DT_SEC);
    pitch_.loop.ref_radps =
        clamp_speed_radps(pitch_reference_speed_radps, cfg::gimbal.pitch.mit.speed_limit_radps);
    pitch_.loop.gravity_nm =
        cfg::gimbal.pitch.gravity.k_nm == 0.0f
            ? 0.0f
            : cfg::gimbal.pitch.gravity.k_nm *
                  pitch_.gravity_cosine.get(pitch_.angle_rad + cfg::gimbal.pitch.gravity.offset_rad);

    update_roll_control();

    base_.loop.torque_nm = clamp_by_abs_limit(base_.loop.torque_nm, cfg::gimbal.base.mit.torque_limit_nm);
    return roll_.ctrl.output().valid && std::isfinite(base_.loop.raw_ref_radps) &&
           std::isfinite(yaw_.loop.raw_ref_radps) && std::isfinite(pitch_.loop.ref_radps) &&
           std::isfinite(roll_.ref_radps) && std::isfinite(dual_.state.reaction_nm) &&
           std::isfinite(base_.loop.dist_torque_nm) && std::isfinite(yaw_.loop.torque_nm) &&
           std::isfinite(yaw_.loop.auto_torque_nm) && std::isfinite(yaw_.loop.manual_torque_nm) &&
           std::isfinite(base_.loop.torque_nm) && std::isfinite(pitch_.loop.gravity_nm) &&
           std::isfinite(roll_.ctrl.output().gravity_torque_nm) &&
           std::isfinite(roll_.ctrl.output().comp_torque_nm);
}

float GimbalCtrl::read_yaw_rad(const Sentry::IMUData &imu)
{
    return Sentry::Utils::normalize_angle(Sentry::Utils::deg_to_rad(imu.angle_deg[2]));
}

float GimbalCtrl::read_pitch_rad(const Sentry::IMUData &imu)
{
    return Sentry::Utils::deg_to_rad(imu.angle_deg[1]);
}

float GimbalCtrl::read_motor_rad(const Sentry::MotorFeedback &feedback)
{

    if (std::isfinite(feedback.output_angle_rad))
    {
        return Sentry::Utils::normalize_angle_0_2pi(feedback.output_angle_rad);
    }
    if (std::isfinite(feedback.rotor_angle_rad))
    {
        return Sentry::Utils::normalize_angle_0_2pi(feedback.rotor_angle_rad);
    }
    return 0.0f;
}

float GimbalCtrl::angle_error_rad(float ref_rad, float fb_rad)
{
    return Sentry::Utils::normalize_angle_error(ref_rad - fb_rad);
}

float GimbalCtrl::measured_yaw_torque_nm(const Sentry::MotorFeedback &yaw_fb) const
{
    if (std::isfinite(yaw_fb.motor_current_A))
    {
        return yaw_.torque_converter.current_to_torque_Nm(yaw_fb.motor_current_A);
    }
    if (std::isfinite(yaw_fb.out_torque_Nm))
    {
        return yaw_fb.out_torque_Nm;
    }
    return yaw_.torque_converter.command_to_torque_Nm(yaw_fb.torque_raw);
}

void GimbalCtrl::reset_pids(bool reset_roll)
{
    base_.speed_pid.reset();
    sentry_.ready = false;
    sentry_.since_ms = 0U;
    manual_.planner.reset(0.0f, 0.0f);
    manual_.ready = false;
    reset_vis_base_planner();
    dual_.reaction_ff.reset(0.0f);
    base_.disturbance.reset(std::isfinite(base_.loop.speed_radps) ? base_.loop.speed_radps : 0.0f);
    yaw_.angle_pid.reset();
    yaw_.speed_pid.reset();
    yaw_.sentry_angle_pid.reset();
    yaw_.sentry_speed_pid.reset();
    yaw_.auto_ctrl.reset();
    pitch_.angle_pid.reset();
    pitch_.sentry_angle_pid.reset();
    reset_tf_ctrls();
    tf_.base.planner.reset();
    tf_.yaw.planner.reset();
    tf_.pitch.planner.reset();
    if (reset_roll)
        reset_roll_reference(roll_.motor.angle_rad);
    reset_control_outputs();
}

void GimbalCtrl::set_roll_mode_ref(OnmiGimbalMode mode)
{
    roll_.ref_rad =
        mode == OnmiGimbalMode::TRANSFORM ? cfg::gimbal.roll.reference.transform_rad : cfg::gimbal.roll.reference.normal_rad;
}

void GimbalCtrl::configure_roll_ctrl()
{
    roll_.ctrl.configure({cfg::gimbal.roll.impedance.stiffness_nm_per_rad, cfg::gimbal.roll.impedance.damping_nms_rad,
                          cfg::gimbal.roll.impedance.transform_ratio, cfg::gimbal.roll.mit.speed_limit_radps,
                          cfg::gimbal.roll.mit.torque_limit_nm, cfg::gimbal.roll.gravity.k_nm,
                          cfg::gimbal.roll.gravity.offset_rad, cfg::gimbal.roll.mit.torque_offset_nm,
                          cfg::gimbal.roll.trajectory.max_speed_radps, cfg::gimbal.roll.trajectory.max_accel_radps2,
                          cfg::gimbal.roll.trajectory.max_jerk_radps3});
}

void GimbalCtrl::update_roll_control(bool update_reference, float hold_ref_rad)
{

    const bool normal_owner = tf_.fsm.state() == App::Onmi::TransformState::NORMAL;
    const auto &result =
        roll_.ctrl.update(update_reference ? roll_.ref_rad : hold_ref_rad, roll_.motor.angle_rad,
                          roll_.motor.speed_radps, roll_.fb_ms, DT_SEC, normal_owner);
    roll_.planned_rad = result.reference_angle_rad;
    roll_.ref_radps = result.reference_speed_radps;
}

void GimbalCtrl::reset_roll_reference(float angle_rad)
{
    roll_.planned_rad =
        std::isfinite(angle_rad) ? Sentry::Utils::normalize_angle_0_2pi(angle_rad) : 0.0f;
    roll_.ctrl.reset(roll_.planned_rad);
}

void GimbalCtrl::reset_control_outputs()
{
    base_.loop = {};
    yaw_.loop = {};
    pitch_.loop = {};
    roll_.ref_radps = 0.0f;
    manual_.q_rad = 0.0f;
    manual_.raw_ref_radps = 0.0f;
    base_.speed_lpf.clear();
    dual_.state = {};
    dual_.reaction_ff.reset(0.0f);
    vision_.ref = {};
    manual_.planner.reset(0.0f, 0.0f);
    manual_.ready = false;
    reset_vis_base_planner();
}

void GimbalCtrl::Update()
{
    // 单周期流程：读取输入快照、选择模式、计算控制并发布输出。
    const auto in = readInputs();

    updateFeedback(in);

    update_vis_yaw_alignment();

    const bool vis_locked_raw = vis_lock_active(in.vision);
    const bool vis_locked = update_vis_mode_debounce(vis_locked_raw, in.nowMs);
    const bool auto_switch_selected =
        switch_selected(state_.s1, state_.s2, cfg::gimbal.input.switches.auto_s1, cfg::gimbal.input.switches.auto_s2);
    const bool tf_requested = state_.request.update(
        switch_selected(state_.s1, state_.s2, cfg::gimbal.input.switches.tf_s1, cfg::gimbal.input.switches.tf_s2),
        auto_switch_selected && remote_allows_enable(in.remote),
        in.host.is_online(SAFETY_HOST_TIMEOUT_MS), in.host.gimbal_mode);
    const bool vis_was_active = state_.fsm.state() == OnmiGimbalMode::VISION;
    bool vis_mode_request_locked = vis_locked;
    if (vis_was_active && auto_switch_selected)
    {
        if (vis_locked)
        {

            vision_.lock.loss_hold = false;
            vision_.lock.loss_ms = 0U;
        }
        else if (in.vision.is_online(SAFETY_VIS_TIMEOUT_MS) && !in.vision.has_ref())
        {

            vis_mode_request_locked = false;
            vision_.lock.loss_hold = false;
            vision_.lock.loss_ms = 0U;
        }
        else
        {
            if (!vision_.lock.loss_hold)
            {
                vision_.lock.loss_hold = true;
                vision_.lock.loss_ms = in.nowMs;
            }
            const bool inside_loss_grace =
                in.nowMs - vision_.lock.loss_ms < cfg::gimbal.vision.lock.grace_ms;
            vis_mode_request_locked = inside_loss_grace;
        }
    }
    else
    {
        vision_.lock.loss_hold = false;
        vision_.lock.loss_ms = 0U;
    }

    state_.fsm.update(select_mode(in.remote, state_.main_imu_online, state_.base_imu_online,
                                  state_.motors_ready, state_.fb_stale, vis_mode_request_locked,
                                  tf_requested, tf_.fsm.owns_control(), normal_roll_ref_reached()),
                      in.nowMs);
    const bool mode_switched = state_.fsm.changed();
    const OnmiGimbalMode previous_mode = state_.fsm.previous_state();
    const OnmiGimbalMode current_mode = state_.fsm.state();

    if (current_mode != OnmiGimbalMode::VISION)
    {
        vision_.lock.loss_hold = false;
        vision_.lock.loss_ms = 0U;
        vision_.lock.active = false;
        vision_.base.centering = false;
    }
    const auto normal_mode = [](OnmiGimbalMode mode)
    {
        return mode == OnmiGimbalMode::MANUAL || mode == OnmiGimbalMode::VISION ||
               mode == OnmiGimbalMode::SENTRY;
    };
    const bool normal_to_normal_switch =
        mode_switched && normal_mode(previous_mode) && normal_mode(current_mode);
    if (mode_switched)
    {

        if (normal_mode(current_mode))
        {
            begin_mode_switch_blend(current_mode, in.nowMs);
        }
        else
        {
            blend_.active = false;
        }
        if (normal_to_normal_switch)
        {

            manual_.ready = false;
            reset_vis_base_planner();
            if (current_mode == OnmiGimbalMode::SENTRY)
            {

                sentry_.ready = false;
                sentry_.since_ms = 0U;
            }
            state_.stopped = false;
        }
        else
        {
            const bool preserve_roll =
                active_control_mode(previous_mode) && active_control_mode(current_mode);
            reset_pids(!preserve_roll);
            state_.stopped = true;
        }
    }

    if (current_mode == OnmiGimbalMode::STOP || current_mode == OnmiGimbalMode::ARMING ||
        current_mode == OnmiGimbalMode::COUNT)
    {

        const auto tf_state = tf_.fsm.state();
        const bool switch_in_progress = tf_state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN ||
                                        tf_state == App::Onmi::TransformState::TO_TRANSFORM_ROLL ||
                                        tf_state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
                                        tf_state == App::Onmi::TransformState::TO_NORMAL_ALIGN ||
                                        tf_state == App::Onmi::TransformState::TO_NORMAL_ROLL;
        if (switch_in_progress)
        {
            tf_.fsm.force_fault();
        }
        else
        {

            tf_.fsm.reset();
        }
        tf_.sync_done = false;
        tf_.external_pending = false;
        tf_.from_stop = false;
        tf_.stable_count = 0U;
        tf_.since_ms = 0U;
    }
    else
    {

        const bool normal_mode_requested = current_mode == OnmiGimbalMode::MANUAL ||
                                           current_mode == OnmiGimbalMode::VISION ||
                                           current_mode == OnmiGimbalMode::SENTRY;
        const bool normal_recovery_from_stop_requested =
            normal_mode_requested && state_.stopped && mode_switched &&
            (previous_mode == OnmiGimbalMode::STOP || previous_mode == OnmiGimbalMode::ARMING ||
             previous_mode == OnmiGimbalMode::COUNT);

        if (normal_recovery_from_stop_requested && tf_.fsm.faulted())
        {
            tf_.fsm.reset();
        }
        update_tf_fsm(current_mode == OnmiGimbalMode::TRANSFORM,
                      normal_recovery_from_stop_requested);
    }

    publish_motion_gate();

    auto stop_after_gate_failure = [&]()
    {
        publish_stop();
        state_.stopped = true;
    };

    switch (current_mode)
    {
    case OnmiGimbalMode::STOP:
    case OnmiGimbalMode::COUNT:
        (void)set_ref(current_mode, in.remote, in.mainImu, in.bigImu, in.vision);
        publish_stop();
        state_.stopped = true;
        return;

    case OnmiGimbalMode::ARMING:
        if (set_ref(current_mode, in.remote, in.mainImu, in.bigImu, in.vision) &&
            update_control(current_mode, in.mainImu, in.bigImu, in.smallFb))
        {
            publish_control(true);
        }
        else
        {
            publish_stop();
        }
        state_.stopped = true;
        return;

    case OnmiGimbalMode::MANUAL:
    case OnmiGimbalMode::VISION:
    case OnmiGimbalMode::SENTRY:
        if (tf_.fsm.faulted())
        {

            publish_stop();
            state_.stopped = true;
            return;
        }
        if ((tf_.fsm.owns_control()
                 ? !system_safe_for_tf_output(in.remote, state_.motors_ready, state_.fb_stale)
                 : !system_safe_for_external_output(in.remote, state_.main_imu_online,
                                                    state_.base_imu_online, state_.motors_ready,
                                                    state_.fb_stale)))
        {
            stop_after_gate_failure();
            return;
        }
        if (tf_.fsm.owns_control())
        {

            if (!update_tf_control(in.bigFb, in.smallFb, in.pitchFb))
            {
                stop_after_gate_failure();
                return;
            }
        }
        else
        {
            if (tf_.external_pending)
            {

                if (!sync_refs_from_imus(in.mainImu, in.bigImu))
                {
                    stop_after_gate_failure();
                    return;
                }
                tf_.external_pending = false;

                begin_mode_switch_blend(current_mode, in.nowMs);
            }

            if (!set_ref(current_mode, in.remote, in.mainImu, in.bigImu, in.vision))
            {
                stop_after_gate_failure();
                return;
            }

            update_mode_switch_blend(in.nowMs);

            if (!update_control(current_mode, in.mainImu, in.bigImu, in.smallFb))
            {
                stop_after_gate_failure();
                return;
            }
        }
        publish_control(true);
        state_.stopped = false;
        return;

    case OnmiGimbalMode::TRANSFORM:
        break;
    }

    if (current_mode == OnmiGimbalMode::TRANSFORM)
    {

        if (!system_safe_for_tf_output(in.remote, state_.motors_ready, state_.fb_stale) ||
            tf_.fsm.faulted() || !tf_.fsm.owns_control() ||
            !update_tf_control(in.bigFb, in.smallFb, in.pitchFb))
        {
            publish_stop();
            state_.stopped = true;
            return;
        }
        publish_control(true);
        state_.stopped = false;
        return;
    }
}

void GimbalCtrl::Stop()
{
    state_.fsm.force(OnmiGimbalMode::STOP, Sentry::Time::now_ms());
    tf_.fsm.reset();
    tf_.sync_done = false;
    tf_.external_pending = false;
    tf_.from_stop = false;
    tf_.since_ms = 0U;
    reset_pids();
    publish_stop();
    state_.stopped = true;
}

float GimbalCtrl::remote_channel_normalized(const Sentry::RemoteData &remote, std::uint8_t index)
{
    float value = 0.0f;
    switch (index)
    {
    case DT7_CH_LEFT_Y:
        value = remote.left_y;
        break;
    case DT7_CH_LEFT_X:
        value = remote.left_x;
        break;
    case DT7_CH_RIGHT_Y:
        value = remote.right_y;
        break;
    case DT7_CH_RIGHT_X:
        value = remote.right_x;
        break;
    case DT7_CH_LEFT_SCROLL:
    case DT7_CH_RIGHT_SCROLL:
        value = remote.scroll_norm;
        break;
    default:
        value = 0.0f;
        break;
    }
    return std::isfinite(value) ? Sentry::Utils::clamp(value, -1.0f, 1.0f) : 0.0f;
}

std::uint8_t GimbalCtrl::dt7_switch(const Sentry::RemoteData &remote, std::uint8_t index)
{
    if (remote.timestamp == 0U)
        return 0U;
    if (index == DT7_CH_S1)
        return remote.s1;
    if (index == DT7_CH_S2)
        return remote.s2;
    return 0U;
}

bool GimbalCtrl::remote_connected(const Sentry::RemoteData &remote)
{
    return remote.timestamp != 0U &&
           Sentry::Time::fresh(remote.timestamp, SAFETY_REMOTE_TIMEOUT_MS);
}

bool GimbalCtrl::remote_allows_enable(const Sentry::RemoteData &remote)
{
    return remote_connected(remote) && !(dt7_switch(remote, DT7_CH_S1) == cfg::gimbal.input.switches.disable_s1 &&
                                         dt7_switch(remote, DT7_CH_S2) == cfg::gimbal.input.switches.disable_s2);
}

bool GimbalCtrl::switch_selected(std::uint8_t s1, std::uint8_t s2, std::uint8_t expected_s1,
                                 std::uint8_t expected_s2)
{
    return s1 == expected_s1 && s2 == expected_s2;
}

bool GimbalCtrl::active_control_mode(OnmiGimbalMode mode)
{
    return mode == OnmiGimbalMode::MANUAL || mode == OnmiGimbalMode::VISION ||
           mode == OnmiGimbalMode::SENTRY || false;
}

bool GimbalCtrl::vis_lock_active(const App::Vision::VisionControlRequest &vision)
{
    return vision.is_online(SAFETY_VIS_TIMEOUT_MS) && vision.has_ref() &&
           std::isfinite(vision.yaw_angle_rad) && std::isfinite(vision.yaw_speed_radps) &&
           std::isfinite(vision.yaw_accel_radps2) && std::isfinite(vision.pitch_angle_rad);
}

bool GimbalCtrl::update_vis_mode_debounce(bool raw_mode_active, std::uint32_t now_ms)
{
    if (!raw_mode_active)
    {
        vision_.mode.rising = false;
        vision_.mode.rising_ms = 0U;
        if (!vision_.mode.active)
        {
            vision_.mode.falling = false;
            vision_.mode.falling_ms = 0U;
            return false;
        }

        if (!vision_.mode.falling)
        {
            vision_.mode.falling = true;
            vision_.mode.falling_ms = now_ms;
        }
        const std::uint32_t deadtime_ms = cfg::gimbal.vision.mode.switch_deadtime_ms;
        if (deadtime_ms != 0U && (now_ms - vision_.mode.falling_ms) < deadtime_ms)
        {

            return true;
        }

        vision_.mode.active = false;
        vision_.mode.falling = false;
        vision_.mode.falling_ms = 0U;
        return false;
    }

    vision_.mode.falling = false;
    vision_.mode.falling_ms = 0U;
    if (vision_.mode.active)
    {
        return true;
    }

    if (!vision_.mode.rising)
    {
        vision_.mode.rising = true;
        vision_.mode.rising_ms = now_ms;
    }

    const std::uint32_t deadtime_ms = cfg::gimbal.vision.mode.switch_deadtime_ms;
    if (deadtime_ms != 0U && (now_ms - vision_.mode.rising_ms) < deadtime_ms)
    {
        return false;
    }

    vision_.mode.active = true;
    vision_.mode.rising = false;
    return true;
}

OnmiGimbalMode GimbalCtrl::select_mode(const Sentry::RemoteData &remote, bool main_imu_online,
                                       bool base_imu_online, bool motors_ready, bool fb_stale,
                                       bool vis_ref_locked, bool tf_requested,
                                       bool tf_control_active, bool normal_roll_ready)
{
    const std::uint8_t s1 = dt7_switch(remote, DT7_CH_S1);
    const std::uint8_t s2 = dt7_switch(remote, DT7_CH_S2);
    if (!remote_allows_enable(remote) || fb_stale || s1 == 0U || s2 == 0U)
    {
        return OnmiGimbalMode::STOP;
    }
    // 普通控制依赖两路 IMU；先检查它们，避免 ARMING 使能电机后立即因 IMU 离线退回 STOP。
    // 变形控制使用独立的编码器安全门，保留其不依赖 IMU 的路径。
    if (!tf_requested && !tf_control_active && (!main_imu_online || !base_imu_online))
    {
        return OnmiGimbalMode::STOP;
    }
    if (!motors_ready)
    {
        return OnmiGimbalMode::ARMING;
    }
    if (tf_requested)
    {
        return OnmiGimbalMode::TRANSFORM;
    }

    const auto select_auto = [&]()
    {
        if (!switch_selected(s1, s2, cfg::gimbal.input.switches.auto_s1, cfg::gimbal.input.switches.auto_s2))
        {
            return OnmiGimbalMode::MANUAL;
        }
        if (vis_ref_locked)
        {
            return OnmiGimbalMode::VISION;
        }
        return normal_roll_ready ? OnmiGimbalMode::SENTRY : OnmiGimbalMode::MANUAL;
    };

    if (tf_control_active)
    {

        return select_auto();
    }

    return select_auto();
}

bool GimbalCtrl::system_safe_for_external_output(const Sentry::RemoteData &remote,
                                                 bool main_imu_online, bool base_imu_online,
                                                 bool motors_ready, bool fb_stale)
{

    return remote_allows_enable(remote) && main_imu_online && base_imu_online && motors_ready &&
           !fb_stale;
}

bool GimbalCtrl::system_safe_for_tf_output(const Sentry::RemoteData &remote, bool motors_ready,
                                           bool fb_stale)
{

    return remote_allows_enable(remote) && motors_ready && !fb_stale;
}

float GimbalCtrl::apply_deadband(float value, float deadband)
{
    if (!std::isfinite(value))
    {
        return 0.0f;
    }
    return std::fabs(value) < std::fabs(deadband) ? 0.0f : value;
}

float GimbalCtrl::clamp_speed_radps(float speed_radps, float limit_radps)
{
    return clamp_by_abs_limit(speed_radps, limit_radps);
}

float GimbalCtrl::slew_limit(float current, float target, float max_delta)
{
    if (!std::isfinite(target))
    {
        target = 0.0f;
    }
    if (!std::isfinite(current))
    {
        current = 0.0f;
    }
    const float limit = std::fabs(max_delta);
    if (limit <= 0.0f)
    {
        return target;
    }
    const float delta = target - current;
    if (delta > limit)
    {
        return current + limit;
    }
    if (delta < -limit)
    {
        return current - limit;
    }
    return target;
}

float GimbalCtrl::wrap_angle_rad(float angle_rad)
{
    return Sentry::Utils::normalize_angle_error(angle_rad);
}

GimbalCtrl::Inputs GimbalCtrl::readInputs()
{
    Inputs in{};
    in.nowMs = Sentry::Time::now_ms();
    in.remote = sentry_data.remote.read();
    in.host = true ? sentry_data.host.read() : App::Chassis::HostMotionRequest{};
    const App::Chassis::ChassisOdometry chassis_odometry =
        true ? sentry_data.odometry.read() : App::Chassis::ChassisOdometry{};
    state_.chassis_locked = chassis_odometry.is_online(SAFETY_CHASSIS_LOCK_TIMEOUT_MS) &&
                            chassis_odometry.gimbal_motion_lock != 0U;
    in.vision = sentry_data.vision.read();

    in.mainImu = sentry_data.main_imu.read();
    in.bigImu = sentry_data.big_imu.read();
    in.bigFb = sentry_data.base_fb.read();
    in.smallFb = sentry_data.yaw_fb.read();
    in.pitchFb = sentry_data.pitch_fb.read();
    in.rollFb = sentry_data.roll_fb.read();
    in.bigStatus = sentry_data.base_status.read();
    in.pitchStatus = sentry_data.pitch_status.read();
    in.rollStatus = sentry_data.roll_status.read();
    return in;
}

void GimbalCtrl::updateFeedback(const Inputs &in)
{
    publish_heading(in.bigImu);
    publish_yaw_joint(in.bigFb);

    state_.remote_online = remote_connected(in.remote);
    state_.s1 = dt7_switch(in.remote, DT7_CH_S1);
    state_.s2 = dt7_switch(in.remote, DT7_CH_S2);
    state_.main_imu_online = in.mainImu.is_online(SAFETY_MAIN_IMU_TIMEOUT_MS);
    state_.base_imu_online = in.bigImu.is_online(SAFETY_BASE_IMU_TIMEOUT_MS);

    update_motor_feedback(base_.motor, in.bigFb);
    update_motor_feedback(yaw_.motor, in.smallFb);
    update_motor_feedback(pitch_.motor, in.pitchFb);
    update_motor_feedback(roll_.motor, in.rollFb);
    base_.motor.status_ok = in.bigStatus.enabled;
    yaw_.motor.status_ok = yaw_.motor.online;
    pitch_.motor.status_ok = in.pitchStatus.enabled;
    roll_.motor.status_ok = in.rollStatus.enabled;
    if (roll_.motor.online)
        roll_.fb_ms = in.rollFb.timestamp;

    state_.fb_stale = (in.bigStatus.fb_ts != 0U && !base_.motor.online) ||
                      (in.smallFb.timestamp != 0U && !yaw_.motor.online) ||
                      (in.pitchStatus.fb_ts != 0U && !pitch_.motor.online) ||
                      (in.rollStatus.fb_ts != 0U && !roll_.motor.online);
    state_.motors_ready = base_.motor.status_ok && yaw_.motor.status_ok &&
                          pitch_.motor.status_ok && roll_.motor.status_ok && base_.motor.online &&
                          yaw_.motor.online && pitch_.motor.online && roll_.motor.online;

    if (state_.main_imu_online)
    {
        const float main_yaw_rad = read_yaw_rad(in.mainImu);
        const float pitch_rad = read_pitch_rad(in.mainImu);
        if (std::isfinite(main_yaw_rad))
            yaw_.angle_rad = main_yaw_rad;
        if (std::isfinite(pitch_rad))
            pitch_.angle_rad = pitch_rad;
        pitch_.speed_radps = Sentry::Utils::deg_to_rad(in.mainImu.gyro_dps[0]);
    }
    else
        pitch_.speed_radps = std::numeric_limits<float>::quiet_NaN();

    if (!std::isfinite(pitch_.speed_radps))
        pitch_.speed_radps = std::numeric_limits<float>::quiet_NaN();

    if (state_.base_imu_online)
    {
        const float base_rad = Sentry::Utils::normalize_angle_0_2pi(
            Sentry::Utils::deg_to_rad(in.bigImu.angle_deg[cfg::gimbal.input.base_imu_yaw_axis]));
        if (std::isfinite(base_rad))
            base_.angle_rad = base_rad;
    }
}

void GimbalCtrl::update_motor_feedback(MotorState &motor,
                                       const Sentry::MotorFeedback &feedback)
{
    motor.online = feedback.timestamp != 0U && feedback.is_online(SAFETY_MOTOR_FB_TIMEOUT_MS);
    if (motor.online)
        motor.angle_rad = read_motor_rad(feedback);
    motor.speed_radps = std::isfinite(feedback.output_speed_radps)
                            ? feedback.output_speed_radps
                            : std::numeric_limits<float>::quiet_NaN();
}

GimbalCtrl::GimbalCtrl() {}

bool GimbalCtrl::Init()
{
    static_assert(cfg::gimbal.input.base_imu_yaw_axis < 3U,
                  "OnmiGimbal::base_imu_yaw_axis 必须选择 angle_deg[0..2]");

    configure_gimbal_ctrls();
    reset_pids();
    tf_.fsm.reset();
    tf_.sync_done = false;
    tf_.external_pending = false;
    tf_.from_stop = false;
    tf_.since_ms = 0U;
    state_.request = App::Onmi::GimbalModeRequest{};
    state_.chassis_locked = false;

    state_.fsm.init(OnmiGimbalMode::STOP, Sentry::Time::now_ms());
    return true;
}

void GimbalCtrl::configure_gimbal_ctrls()
{
    vision_.angle_hold.configure({

        cfg::gimbal.vision.base.hold.kp_radps_per_rad,

        cfg::gimbal.vision.base.hold.speed_limit_radps,

        cfg::gimbal.vision.base.hold.accel_limit_radps2,

        cfg::gimbal.vision.base.hold.motion_trend_cutoff_hz,

        cfg::gimbal.vision.base.hold.motion_speed_start_radps,

        cfg::gimbal.vision.base.hold.motion_speed_full_radps,

        cfg::gimbal.vision.base.hold.motion_rise_time_sec,

        cfg::gimbal.vision.base.hold.motion_release_time_sec,

        cfg::gimbal.vision.base.hold.anchor_follow_time_sec,

        cfg::gimbal.vision.base.hold.center_filter_time_sec,

        cfg::gimbal.vision.base.hold.ki_radps_per_rad_sec,

        cfg::gimbal.vision.base.hold.integral_limit_radps,

        cfg::gimbal.vision.base.hold.integral_release_time_sec,

        cfg::gimbal.vision.base.hold.integral_speed_start_radps,

        cfg::gimbal.vision.base.hold.integral_speed_full_radps,

    });
    yaw_.limit.set_limits(-SAFETY_YAW_MECH_LIMIT_RAD, SAFETY_YAW_MECH_LIMIT_RAD);
    pitch_.limit.set_limits(SAFETY_PITCH_MIN_RAD, SAFETY_PITCH_MAX_RAD);
    base_.speed_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.base.speed_pid.kp, cfg::gimbal.base.speed_pid.ki, cfg::gimbal.base.speed_pid.kd,
        cfg::gimbal.base.speed_pid.output_max, cfg::gimbal.base.speed_pid.output_min,
        cfg::gimbal.base.speed_pid.integral_max});
    tf_.base.angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.transform.base.angle_pid.kp, cfg::gimbal.transform.base.angle_pid.ki, cfg::gimbal.transform.base.angle_pid.kd,
        cfg::gimbal.transform.base.angle_pid.output_max, cfg::gimbal.transform.base.angle_pid.output_min,
        cfg::gimbal.transform.base.angle_pid.integral_max});
    tf_.base.angle_pid.set_integral_separation(cfg::gimbal.transform.base.angle_pid.integral_separation_rad,
                                               cfg::gimbal.transform.base.angle_pid.integral_reset_rad);
    yaw_.angle_pid.set_config(
        ALG::Omni::TimePid::Config{cfg::gimbal.yaw.angle_pid.kp, cfg::gimbal.yaw.angle_pid.ki,
                                   cfg::gimbal.yaw.angle_pid.kd, cfg::gimbal.yaw.angle_pid.output_max,
                                   cfg::gimbal.yaw.angle_pid.output_min, cfg::gimbal.yaw.angle_pid.integral_max});
    yaw_.angle_pid.set_integral_separation(cfg::gimbal.yaw.angle_pid.integral_separation_rad,
                                           cfg::gimbal.yaw.angle_pid.integral_reset_rad);
    yaw_.speed_pid.set_config(
        ALG::Omni::TimePid::Config{cfg::gimbal.yaw.speed_pid.kp, cfg::gimbal.yaw.speed_pid.ki,
                                   cfg::gimbal.yaw.speed_pid.kd, cfg::gimbal.yaw.motor.max_torque_nm(),
                                   -cfg::gimbal.yaw.motor.max_torque_nm(), cfg::gimbal.yaw.speed_pid.integral_max});
    tf_.yaw.angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.transform.yaw.angle_pid.kp, cfg::gimbal.transform.yaw.angle_pid.ki, cfg::gimbal.transform.yaw.angle_pid.kd,
        cfg::gimbal.transform.yaw.angle_pid.output_max, cfg::gimbal.transform.yaw.angle_pid.output_min,
        cfg::gimbal.transform.yaw.angle_pid.integral_max});
    tf_.yaw.angle_pid.set_integral_separation(cfg::gimbal.transform.yaw.angle_pid.integral_separation_rad,
                                              cfg::gimbal.transform.yaw.angle_pid.integral_reset_rad);
    tf_.yaw_speed_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.transform.yaw.speed_pid.kp, cfg::gimbal.transform.yaw.speed_pid.ki, cfg::gimbal.transform.yaw.speed_pid.kd,
        cfg::gimbal.transform.yaw.speed_pid.output_max, cfg::gimbal.transform.yaw.speed_pid.output_min,
        cfg::gimbal.transform.yaw.speed_pid.integral_max});
    yaw_.sentry_angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.sentry.yaw.encoder_angle_pid.kp, cfg::gimbal.sentry.yaw.encoder_angle_pid.ki,
        cfg::gimbal.sentry.yaw.encoder_angle_pid.kd, cfg::gimbal.sentry.yaw.encoder_angle_pid.output_max,
        cfg::gimbal.sentry.yaw.encoder_angle_pid.output_min,
        cfg::gimbal.sentry.yaw.encoder_angle_pid.integral_max});
    yaw_.sentry_angle_pid.set_integral_separation(
        cfg::gimbal.sentry.yaw.encoder_angle_pid.integral_separation_rad,
        cfg::gimbal.sentry.yaw.encoder_angle_pid.integral_reset_rad);
    yaw_.sentry_speed_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.sentry.yaw.encoder_speed_pid.kp, cfg::gimbal.sentry.yaw.encoder_speed_pid.ki,
        cfg::gimbal.sentry.yaw.encoder_speed_pid.kd, cfg::gimbal.sentry.yaw.encoder_speed_pid.output_max,
        cfg::gimbal.sentry.yaw.encoder_speed_pid.output_min,
        cfg::gimbal.sentry.yaw.encoder_speed_pid.integral_max});
    pitch_.angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.pitch.angle_pid.kp, cfg::gimbal.pitch.angle_pid.ki, cfg::gimbal.pitch.angle_pid.kd,
        cfg::gimbal.pitch.angle_pid.output_max_radps, cfg::gimbal.pitch.angle_pid.output_min_radps,
        cfg::gimbal.pitch.angle_pid.integral_max_rad_s});
    pitch_.angle_pid.set_integral_separation(cfg::gimbal.pitch.angle_pid.integral_separation_rad,
                                             cfg::gimbal.pitch.angle_pid.integral_reset_rad);
    tf_.pitch.angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.transform.pitch.angle_pid.kp, cfg::gimbal.transform.pitch.angle_pid.ki, cfg::gimbal.transform.pitch.angle_pid.kd,
        cfg::gimbal.transform.pitch.angle_pid.output_max_radps, cfg::gimbal.transform.pitch.angle_pid.output_min_radps,
        cfg::gimbal.transform.pitch.angle_pid.integral_max_rad_s});
    tf_.pitch.angle_pid.set_integral_separation(cfg::gimbal.transform.pitch.angle_pid.integral_separation_rad,
                                                cfg::gimbal.transform.pitch.angle_pid.integral_reset_rad);
    pitch_.sentry_angle_pid.set_config(ALG::Omni::TimePid::Config{
        cfg::gimbal.sentry.pitch.encoder_angle_pid.kp, cfg::gimbal.sentry.pitch.encoder_angle_pid.ki,
        cfg::gimbal.sentry.pitch.encoder_angle_pid.kd,
        cfg::gimbal.sentry.pitch.encoder_angle_pid.output_max_radps,
        cfg::gimbal.sentry.pitch.encoder_angle_pid.output_min_radps,
        cfg::gimbal.sentry.pitch.encoder_angle_pid.integral_max_rad_s});
    pitch_.sentry_angle_pid.set_integral_separation(
        cfg::gimbal.sentry.pitch.encoder_angle_pid.integral_separation_rad,
        cfg::gimbal.sentry.pitch.encoder_angle_pid.integral_reset_rad);
    configure_roll_ctrl();
    manual_.planner.configure(
        {cfg::gimbal.base.manual.speed_limit_radps, cfg::gimbal.base.manual.accel_limit_radps2,
         cfg::gimbal.base.manual.accel_limit_radps2, cfg::gimbal.base.manual.jerk_limit_radps3});
    vision_.planner.configure({cfg::gimbal.vision.base.speed_limit_radps, cfg::gimbal.vision.base.accel_limit_radps2,
                               cfg::gimbal.vision.base.accel_limit_radps2, cfg::gimbal.vision.base.jerk_limit_radps3});
    base_.disturbance.configure({cfg::gimbal.base.observer.nominal_inertia_kgm2,
                                 cfg::gimbal.base.observer.nominal_damping_nms_rad,
                                 cfg::gimbal.base.observer.bandwidth_rad_s, cfg::gimbal.base.observer.comp_gain,
                                 cfg::gimbal.base.observer.comp_limit_nm});
    dual_.reaction_ff.configure(
        {cfg::gimbal.yaw.motor.torque_constant_nm_per_a, cfg::gimbal.dual_yaw.reaction.comp_gain,
         cfg::gimbal.dual_yaw.reaction.filter_cutoff_hz, cfg::gimbal.dual_yaw.reaction.torque_limit_nm,
         cfg::gimbal.dual_yaw.reaction.torque_slew_rate_nmps, cfg::gimbal.dual_yaw.reaction.coupled_inertia_kgm2,
         cfg::gimbal.dual_yaw.reaction.inertia_gain, cfg::gimbal.dual_yaw.reaction.rel_speed_filter_cutoff_hz,
         cfg::gimbal.dual_yaw.reaction.rel_accel_filter_cutoff_hz});
    yaw_.auto_ctrl.configure({cfg::gimbal.yaw.auto_control.j_kg_m2,
                              cfg::gimbal.yaw.auto_control.b_nms_rad,
                              cfg::gimbal.yaw.auto_control.k_theta,
                              cfg::gimbal.yaw.auto_control.k_omega,
                              cfg::gimbal.yaw.auto_control.k_i,
                              cfg::gimbal.yaw.auto_control.theta_integral_limit_rad_s,
                              cfg::gimbal.yaw.auto_control.ff_gain,
                              cfg::gimbal.yaw.auto_control.tau_coulomb_nm,
                              cfg::gimbal.yaw.auto_control.coulomb_smooth_rad_s,
                              cfg::gimbal.yaw.auto_control.eso_bandwidth_rad_s,
                              cfg::gimbal.yaw.auto_control.eso_comp_gain,
                              cfg::gimbal.yaw.auto_control.eso_comp_limit_nm,
                              cfg::gimbal.yaw.auto_control.eso_omega_gate_rad_s,
                              cfg::gimbal.yaw.auto_control.eso_alpha_gate_rad_s2,
                              cfg::gimbal.yaw.auto_control.tau_bias_ki,
                              cfg::gimbal.yaw.auto_control.tau_bias_limit_nm,
                              cfg::gimbal.yaw.auto_control.tau_meas_lpf_alpha,
                              cfg::gimbal.yaw.auto_control.theta_deadband_rad,
                              cfg::gimbal.yaw.motor.max_torque_nm(),
                              -cfg::gimbal.yaw.motor.max_torque_nm(),
                              cfg::gimbal.yaw.motor.max_torque_nm(),
                              cfg::gimbal.yaw.auto_control.torque_slew_rate_nm_s,
                              cfg::gimbal.yaw.auto_control.eso_enable,
                              cfg::gimbal.yaw.auto_control.eso_comp_enable,
                              cfg::gimbal.yaw.auto_control.torque_slew_enable,
                              cfg::gimbal.yaw.auto_control.omega_deadband_rad_s,
                              cfg::gimbal.yaw.auto_control.reference_speed_deadband_rad_s});
    yaw_.torque_converter.configure({cfg::gimbal.yaw.motor.torque_constant_nm_per_a, cfg::gimbal.yaw.motor.current_max_a,
                                     cfg::gimbal.yaw.motor.command_max_raw, cfg::gimbal.yaw.motor.command_limit_raw});
    const ALG::Omni::SCurveVelocityPlanning::Config tf_axis_plan_config{
        cfg::gimbal.roll.trajectory.max_speed_radps, cfg::gimbal.roll.trajectory.max_accel_radps2,
        cfg::gimbal.roll.trajectory.max_accel_radps2, cfg::gimbal.roll.trajectory.max_jerk_radps3};
    tf_.base.planner.configure(tf_axis_plan_config);
    tf_.yaw.planner.configure(tf_axis_plan_config);
    tf_.pitch.planner.configure(tf_axis_plan_config);
}

bool GimbalCtrl::set_ref(OnmiGimbalMode mode, const Sentry::RemoteData &remote,
                         const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu,
                         const App::Vision::VisionControlRequest &vision)
{
    switch (mode)
    {
    case OnmiGimbalMode::STOP:
    case OnmiGimbalMode::COUNT:
        return set_idle_ref();

    case OnmiGimbalMode::ARMING:
        return set_idle_ref();

    case OnmiGimbalMode::MANUAL:
        return set_manual_ref(remote, main_imu, base_imu);

    case OnmiGimbalMode::VISION:
        return set_vis_ref(main_imu, base_imu, vision);

    case OnmiGimbalMode::SENTRY:
        return set_sentry_ref(main_imu, base_imu);

    case OnmiGimbalMode::TRANSFORM:
        return false;
    }
    return false;
}

bool GimbalCtrl::set_idle_ref()
{
    yaw_.ref_rad = yaw_.angle_rad;
    yaw_.aim_rad = yaw_.ref_rad;
    yaw_.offset_rad = current_yaw_offset_rad();
    yaw_.limit.reset(yaw_.offset_rad);
    yaw_.ref_offset_rad = yaw_.limit.target();
    pitch_.limit.reset(pitch_.angle_rad);
    pitch_.ref_rad = pitch_.limit.target();
    roll_.ref_rad = roll_.motor.angle_rad;
    reset_control_outputs();
    return true;
}

bool GimbalCtrl::set_manual_ref(const Sentry::RemoteData &remote, const Sentry::IMUData &main_imu,
                                const Sentry::IMUData &base_imu)
{
    if (state_.stopped && !sync_refs_from_imus(main_imu, base_imu))
    {
        return false;
    }

    const float yaw_input = apply_deadband(remote_channel_normalized(remote, DT7_CH_RIGHT_X),
                                           cfg::gimbal.input.analog_deadband);
    const float pitch_input = apply_deadband(remote_channel_normalized(remote, DT7_CH_RIGHT_Y),
                                             cfg::gimbal.input.analog_deadband);

    const float yaw_delta_rad = cfg::gimbal.input.yaw_stick_sign * yaw_input * cfg::gimbal.yaw.limits.rate_radps * DT_SEC;

    const float desired_yaw_ref_rad =
        Sentry::Utils::normalize_angle_0_2pi(yaw_.aim_rad + yaw_delta_rad);
    if (!set_yaw_ref_from_global(desired_yaw_ref_rad))
    {
        return false;
    }

    yaw_.aim_rad = yaw_.ref_rad;

    const float pitch_delta_rad = pitch_input * cfg::gimbal.pitch.limits.rate_radps * DT_SEC;
    pitch_.ref_rad = pitch_.limit.update(pitch_delta_rad);

    set_roll_mode_ref(OnmiGimbalMode::MANUAL);

    return std::isfinite(yaw_.ref_rad) && std::isfinite(pitch_.ref_rad) &&
           std::isfinite(roll_.ref_rad);
}

bool GimbalCtrl::set_vis_ref(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu,
                             const App::Vision::VisionControlRequest &vision)
{
    if (state_.stopped && !sync_refs_from_imus(main_imu, base_imu))
    {
        return false;
    }
    set_roll_mode_ref(OnmiGimbalMode::VISION);

    if (!vis_lock_active(vision))
    {

        const bool mode_glitch_hold = vision_.mode.active && vision_.mode.falling;
        const float current_offset_rad = current_yaw_offset_rad();
        const float desired_delta_rad = angle_error_rad(yaw_.aim_rad, yaw_.angle_rad);
        const bool hold = (vision_.lock.loss_hold || mode_glitch_hold) &&
                          std::isfinite(yaw_.aim_rad) && std::isfinite(pitch_.ref_rad) &&
                          std::isfinite(current_offset_rad) && std::isfinite(desired_delta_rad) &&
                          std::isfinite(roll_.ref_rad);
        vision_.ref.speed_radps = 0.0f;
        vision_.ref.accel_radps2 = 0.0f;
        if (hold)
        {
            vision_.q.request_rad = wrap_angle_rad(current_offset_rad + desired_delta_rad);
            vision_.q.predicted_rad = vision_.q.request_rad;
        }
        return hold;
    }

    const float incoming_ref_rad = Sentry::Utils::normalize_angle_0_2pi(vision.yaw_angle_rad);
    if (!vision_.base.centering)
    {
        yaw_.aim_rad = incoming_ref_rad;
        vision_.ref.speed_radps =
            clamp_speed_radps(vision.yaw_speed_radps, cfg::gimbal.yaw.limits.speed_limit_radps);
        vision_.ref.accel_radps2 =
            clamp_by_abs_limit(vision.yaw_accel_radps2, cfg::gimbal.yaw.limits.ref_accel_limit_radps2);
    }
    else
    {

        yaw_.aim_rad = vision_.base.init_rad;
        vision_.ref.speed_radps = 0.0f;
        vision_.ref.accel_radps2 = 0.0f;
    }
    const float current_offset_rad = current_yaw_offset_rad();
    const float desired_delta_rad = angle_error_rad(yaw_.aim_rad, yaw_.angle_rad);
    if (!std::isfinite(current_offset_rad) || !std::isfinite(desired_delta_rad))
    {
        return false;
    }

    vision_.q.request_rad = wrap_angle_rad(current_offset_rad + desired_delta_rad);
    vision_.q.predicted_rad =
        vision_.q.request_rad + vision_.ref.speed_radps * cfg::gimbal.vision.q.predict_time_sec +
        0.5f * vision_.ref.accel_radps2 * cfg::gimbal.vision.q.predict_time_sec * cfg::gimbal.vision.q.predict_time_sec;
    const float pitch_ref_rad =
        Sentry::Utils::clamp(vision.pitch_angle_rad, SAFETY_PITCH_MIN_RAD, SAFETY_PITCH_MAX_RAD);
    pitch_.limit.reset(pitch_ref_rad);
    pitch_.ref_rad = pitch_.limit.target();
    if (!set_yaw_ref_from_global(yaw_.aim_rad) || !std::isfinite(pitch_.ref_rad) ||
        !std::isfinite(roll_.ref_rad))
    {
        return false;
    }

    if (!vision_.lock.active)
    {
        vision_.lock.active = true;
        begin_vis_init_lock_centering();
    }
    return true;
}

bool GimbalCtrl::set_sentry_ref(const Sentry::IMUData &main_imu, const Sentry::IMUData &base_imu)
{

    const bool need_init_sync = state_.stopped && !sentry_.ready;
    if (need_init_sync && !sync_refs_from_imus(main_imu, base_imu))
    {
        return false;
    }

    if (!sentry_.ready)
    {

        sentry_.ready = true;
        sentry_.since_ms = Sentry::Time::now_ms();
    }

    set_roll_mode_ref(OnmiGimbalMode::SENTRY);
    if (!cfg::gimbal.sentry.enabled)
    {

        yaw_.ref_offset_rad = current_yaw_offset_rad();
        yaw_.ref_rad = yaw_.motor.angle_rad;
        yaw_.aim_rad = yaw_.ref_rad;
        pitch_.ref_rad = cfg::gimbal.sentry.pitch.use_imu_fb ? pitch_.angle_rad : pitch_.motor.angle_rad;
        return std::isfinite(yaw_.ref_offset_rad) && std::isfinite(pitch_.ref_rad) &&
               std::isfinite(roll_.ref_rad);
    }

    const std::uint32_t now_ms = Sentry::Time::now_ms();
    const float elapsed_sec = static_cast<float>(now_ms - sentry_.since_ms) * 0.001f;
    const float yaw_phase =
        Sentry::Utils::Constants::TWO_PI * cfg::gimbal.sentry.yaw.frequency_hz * elapsed_sec;
    const float pitch_phase =
        Sentry::Utils::Constants::TWO_PI * cfg::gimbal.sentry.pitch.frequency_hz * elapsed_sec;
    const float sentry_yaw_offset_rad = Sentry::Utils::clamp(
        cfg::gimbal.sentry.yaw.center_rad + cfg::gimbal.sentry.yaw.amplitude_rad * std::sin(yaw_phase),
        cfg::gimbal.sentry.yaw.min_rad, cfg::gimbal.sentry.yaw.max_rad);
    const float sentry_pitch_ref_rad = Sentry::Utils::clamp(
        cfg::gimbal.sentry.pitch.center_rad + cfg::gimbal.sentry.pitch.amplitude_rad * std::sin(pitch_phase),
        cfg::gimbal.sentry.pitch.min_rad, cfg::gimbal.sentry.pitch.max_rad);

    pitch_.ref_rad = sentry_pitch_ref_rad;
    return set_sentry_yaw_ref_from_offset(sentry_yaw_offset_rad) && std::isfinite(pitch_.ref_rad) &&
           std::isfinite(roll_.ref_rad);
}

void GimbalCtrl::begin_mode_switch_blend(OnmiGimbalMode current_mode, std::uint32_t now_ms)
{
    if (cfg::gimbal.mode.blend_ms == 0U)
    {
        blend_.active = false;
        return;
    }

    const bool sentry_yaw_fb = current_mode == OnmiGimbalMode::SENTRY;
    const float measured_yaw_rad = sentry_yaw_fb ? yaw_.motor.angle_rad : yaw_.angle_rad;
    const bool sentry_pitch_encoder_fb =
        current_mode == OnmiGimbalMode::SENTRY && !cfg::gimbal.sentry.pitch.use_imu_fb;
    const float measured_pitch_rad =
        sentry_pitch_encoder_fb ? pitch_.motor.angle_rad : pitch_.angle_rad;

    blend_.yaw_rad = measured_yaw_rad;
    blend_.pitch_rad = measured_pitch_rad;
    if (!std::isfinite(blend_.yaw_rad))
    {
        blend_.yaw_rad = 0.0f;
    }
    if (!std::isfinite(blend_.pitch_rad))
    {
        blend_.pitch_rad = 0.0f;
    }
    blend_.since_ms = now_ms;
    blend_.active = true;
}

void GimbalCtrl::update_mode_switch_blend(std::uint32_t now_ms)
{
    if (!blend_.active)
    {
        return;
    }

    const std::uint32_t duration_ms = cfg::gimbal.mode.blend_ms;
    const std::uint32_t elapsed_ms = now_ms - blend_.since_ms;
    const float alpha =
        duration_ms == 0U
            ? 1.0f
            : Sentry::Utils::clamp(static_cast<float>(elapsed_ms) / static_cast<float>(duration_ms),
                                   0.0f, 1.0f);

    if (std::isfinite(yaw_.ref_rad))
    {
        const float delta = angle_error_rad(yaw_.ref_rad, blend_.yaw_rad);
        yaw_.ref_rad = Sentry::Utils::normalize_angle_0_2pi(blend_.yaw_rad + delta * alpha);
    }
    if (std::isfinite(pitch_.ref_rad))
    {
        pitch_.ref_rad = blend_.pitch_rad + (pitch_.ref_rad - blend_.pitch_rad) * alpha;
    }

    if (alpha >= 1.0f)
    {
        blend_.active = false;
    }
}

float GimbalCtrl::current_yaw_offset_rad() const
{
    const float rel_rad = encoder_rel_yaw_rad();
    return std::isfinite(rel_rad) ? rel_rad : 0.0f;
}

bool GimbalCtrl::set_sentry_yaw_ref_from_offset(float ref_offset_rad)
{
    const float current_offset_rad = current_yaw_offset_rad();
    if (!std::isfinite(ref_offset_rad) || !std::isfinite(current_offset_rad))
    {
        return false;
    }

    yaw_.limit.reset(current_offset_rad);
    yaw_.ref_offset_rad = yaw_.limit.update(ref_offset_rad - current_offset_rad);
    yaw_.offset_rad = current_offset_rad;
    yaw_.ref_rad =
        Sentry::Utils::normalize_angle_0_2pi(cfg::gimbal.yaw.encoder.zero_rad + yaw_.ref_offset_rad);
    yaw_.aim_rad = yaw_.ref_rad;
    return std::isfinite(yaw_.ref_offset_rad) && std::isfinite(yaw_.ref_rad);
}

float GimbalCtrl::encoder_rel_yaw_rad() const
{
    if (!std::isfinite(yaw_.motor.angle_rad))
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float rel_rad = angle_error_rad(yaw_.motor.angle_rad, cfg::gimbal.yaw.encoder.zero_rad);
    return wrap_angle_rad(rel_rad);
}

bool GimbalCtrl::set_yaw_ref_from_global(float desired_ref_rad)
{
    const float current_offset_rad = current_yaw_offset_rad();
    if (!std::isfinite(desired_ref_rad) || !std::isfinite(yaw_.angle_rad) ||
        !std::isfinite(base_.angle_rad) || !std::isfinite(current_offset_rad))
    {
        return false;
    }

    const float desired_delta_rad = angle_error_rad(desired_ref_rad, yaw_.angle_rad);
    yaw_.limit.reset(current_offset_rad);
    yaw_.ref_offset_rad = yaw_.limit.update(desired_delta_rad);
    yaw_.ref_rad = Sentry::Utils::normalize_angle_0_2pi(yaw_.angle_rad +
                                                        (yaw_.ref_offset_rad - current_offset_rad));
    yaw_.offset_rad = current_offset_rad;
    return std::isfinite(yaw_.ref_offset_rad) && std::isfinite(yaw_.ref_rad);
}

bool GimbalCtrl::set_yaw_ref_from_offset(float ref_offset_rad)
{
    const float current_offset_rad = current_yaw_offset_rad();
    if (!std::isfinite(ref_offset_rad) || !std::isfinite(yaw_.angle_rad) ||
        !std::isfinite(current_offset_rad))
    {
        return false;
    }

    yaw_.limit.reset(current_offset_rad);
    yaw_.ref_offset_rad = yaw_.limit.update(ref_offset_rad - current_offset_rad);
    yaw_.ref_rad = Sentry::Utils::normalize_angle_0_2pi(yaw_.angle_rad +
                                                        (yaw_.ref_offset_rad - current_offset_rad));
    yaw_.aim_rad = yaw_.ref_rad;
    yaw_.offset_rad = current_offset_rad;
    return std::isfinite(yaw_.ref_offset_rad) && std::isfinite(yaw_.ref_rad);
}

bool GimbalCtrl::set_refs_from_imus(const Sentry::IMUData &main_imu,
                                    const Sentry::IMUData &base_imu)
{
    const float main_yaw_rad = read_yaw_rad(main_imu);
    const float base_rad = Sentry::Utils::normalize_angle_0_2pi(
        Sentry::Utils::deg_to_rad(base_imu.angle_deg[cfg::gimbal.input.base_imu_yaw_axis]));
    const float pitch_rad = read_pitch_rad(main_imu);
    const float roll_rad = roll_.motor.angle_rad;
    if (!std::isfinite(main_yaw_rad) || !std::isfinite(base_rad) || !std::isfinite(pitch_rad) ||
        !std::isfinite(roll_rad))
    {
        return false;
    }

    yaw_.ref_rad = Sentry::Utils::normalize_angle_0_2pi(main_yaw_rad);
    yaw_.aim_rad = yaw_.ref_rad;
    yaw_.offset_rad = current_yaw_offset_rad();
    yaw_.limit.reset(yaw_.offset_rad);
    yaw_.ref_offset_rad = yaw_.limit.target();
    pitch_.limit.reset(pitch_rad);
    pitch_.ref_rad = pitch_.limit.target();
    roll_.ref_rad = roll_rad;
    return true;
}

bool GimbalCtrl::sync_refs_from_imus(const Sentry::IMUData &main_imu,
                                     const Sentry::IMUData &base_imu)
{
    if (!set_refs_from_imus(main_imu, base_imu))
    {
        return false;
    }
    reset_pids(false);
    state_.stopped = false;
    return true;
}

void GimbalCtrl::AxisPath::reset(float angle_rad, float target_rad)
{
    start_rad = angle_rad;
    delta_rad = GimbalCtrl::angle_error_rad(target_rad, start_rad);
    path_rad = 0.0f;
    ref_rad = start_rad;
}

bool GimbalCtrl::AxisPath::advance(ALG::Omni::SCurveVelocityPlanning &planner, float max_speed,
                                   float max_accel, float max_jerk, float dt_sec)
{
    const float distance = std::fabs(delta_rad);
    if (!std::isfinite(start_rad) || !std::isfinite(delta_rad) || !std::isfinite(path_rad) ||
        !std::isfinite(max_speed) || !std::isfinite(max_accel) || !std::isfinite(max_jerk) ||
        !std::isfinite(dt_sec) || dt_sec <= 0.0f || max_speed <= 0.0f || max_accel <= 0.0f ||
        max_jerk <= 0.0f)
    {
        return false;
    }
    if (distance <= 0.0000174533f)
    {
        path_rad = 0.0f;
        planner.reset(0.0f);
        ref_rad = Sentry::Utils::normalize_angle_0_2pi(start_rad);
        return true;
    }
    path_rad = std::fmin(std::fmax(path_rad, 0.0f), distance);
    const float remaining = distance - path_rad;
    if (remaining <= 0.0000174533f)
    {
        path_rad = distance;
        planner.reset(0.0f);
        ref_rad = Sentry::Utils::normalize_angle_0_2pi(start_rad + delta_rad);
        return true;
    }

    const float braking_speed = std::sqrt(2.0f * max_accel * remaining);
    const float desired_speed = std::fmin(max_speed, braking_speed);
    const float planned_speed = planner.update(desired_speed, dt_sec);
    path_rad = std::fmin(distance, path_rad + std::fmax(0.0f, planned_speed * dt_sec));
    ref_rad = Sentry::Utils::normalize_angle_0_2pi(start_rad + delta_rad * (path_rad / distance));
    if (path_rad >= distance - 0.0000174533f)
    {
        path_rad = distance;
        planner.reset(0.0f);
        ref_rad = Sentry::Utils::normalize_angle_0_2pi(start_rad + delta_rad);
        return true;
    }
    return false;
}

bool GimbalCtrl::update_tf_control(const Sentry::MotorFeedback &base_fb,
                                   const Sentry::MotorFeedback &yaw_fb,
                                   const Sentry::MotorFeedback &pitch_fb)
{
    const auto state = tf_.fsm.state();

    if (!tf_.fsm.owns_control() || !std::isfinite(base_.motor.angle_rad) ||
        !std::isfinite(yaw_.motor.angle_rad) || !std::isfinite(pitch_.motor.angle_rad) ||
        !std::isfinite(roll_.motor.angle_rad) || !std::isfinite(base_fb.output_speed_radps) ||
        !std::isfinite(yaw_fb.output_speed_radps) || !std::isfinite(pitch_fb.output_speed_radps))
    {
        reset_control_outputs();
        return false;
    }

    float base_ref_rad = tf_.base.ref_rad;
    float yaw_ref_rad = tf_.yaw.ref_rad;
    float pitch_ref_rad = tf_.pitch.ref_rad;

    switch (state)
    {
    case App::Onmi::TransformState::TO_TRANSFORM_ALIGN:
        base_ref_rad = tf_.align.base.ref_rad;
        yaw_ref_rad = tf_.align.yaw.ref_rad;
        pitch_ref_rad = tf_.align.pitch.ref_rad;
        break;

    case App::Onmi::TransformState::TO_TRANSFORM_ROLL:
    case App::Onmi::TransformState::TRANSFORM:
        break;

    case App::Onmi::TransformState::TO_NORMAL_SYNC:
    case App::Onmi::TransformState::TO_NORMAL_ALIGN:
    case App::Onmi::TransformState::TO_NORMAL_ROLL:

        base_ref_rad = tf_.sync.base_rad;
        yaw_ref_rad = tf_.sync.yaw_rad;
        pitch_ref_rad = tf_.sync.pitch_rad;
        if (tf_.from_stop && (state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
                              state == App::Onmi::TransformState::TO_NORMAL_ALIGN))
        {
            base_ref_rad = tf_.recovery.base.ref_rad;
            yaw_ref_rad = tf_.recovery.yaw.ref_rad;
            pitch_ref_rad = tf_.recovery.pitch.ref_rad;
        }
        break;

    case App::Onmi::TransformState::NORMAL:
    case App::Onmi::TransformState::FAULT:
        reset_control_outputs();
        return false;
    }

    if (state == App::Onmi::TransformState::TO_TRANSFORM_ROLL ||
        state == App::Onmi::TransformState::TRANSFORM)
    {
        roll_.ref_rad = cfg::gimbal.roll.reference.transform_rad;
    }
    else if (state == App::Onmi::TransformState::TO_NORMAL_ROLL)
    {
        roll_.ref_rad = cfg::gimbal.roll.reference.normal_rad;
    }
    else if (state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN)
    {
        roll_.ref_rad = cfg::gimbal.roll.reference.normal_rad;
    }
    else if (state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
             state == App::Onmi::TransformState::TO_NORMAL_ALIGN)
    {
        roll_.ref_rad = tf_.recovery.roll_hold_rad;
    }

    const float base_setpoint_rad = shortest_setpoint_rad(base_ref_rad, base_.motor.angle_rad);
    const float base_fb_rad = base_.motor.angle_rad;
    const float base_angle_output_radps =
        clamp_speed_radps(tf_.base.angle_pid.update(base_setpoint_rad, base_fb_rad, DT_SEC),
                          cfg::gimbal.base.mit.speed_limit_radps);
    const float base_speed_ref_radps = base_angle_output_radps;
    const float base_speed_fb_radps = base_fb.output_speed_radps;
    base_.loop.raw_ref_radps = base_speed_ref_radps;
    base_.loop.ref_radps = base_speed_ref_radps;
    base_.loop.speed_radps = base_speed_fb_radps;

    base_.loop.torque_nm = 0.0f;

    const float yaw_setpoint_rad = shortest_setpoint_rad(yaw_ref_rad, yaw_.motor.angle_rad);
    const float yaw_fb_rad = yaw_.motor.angle_rad;
    const float yaw_angle_output_radps = clamp_speed_radps(
        tf_.yaw.angle_pid.update(yaw_setpoint_rad, yaw_fb_rad, DT_SEC), cfg::gimbal.yaw.limits.speed_limit_radps);
    const float yaw_speed_fb_radps = yaw_fb.output_speed_radps;
    yaw_.loop.raw_ref_radps = yaw_angle_output_radps;
    yaw_.loop.ref_radps = yaw_angle_output_radps;
    yaw_.loop.speed_radps = yaw_speed_fb_radps;
    yaw_.loop.manual_torque_nm =
        tf_.yaw_speed_pid.update(yaw_.loop.ref_radps, yaw_.loop.speed_radps, DT_SEC);
    yaw_.loop.torque_nm = yaw_.loop.manual_torque_nm;
    yaw_.loop.auto_torque_nm = 0.0f;
    yaw_.loop.cmd_raw = yaw_.torque_converter.torque_to_command(yaw_.loop.manual_torque_nm);

    const float pitch_setpoint_rad = shortest_setpoint_rad(pitch_ref_rad, pitch_.motor.angle_rad);
    const float pitch_ref_speed_radps = clamp_speed_radps(
        tf_.pitch.angle_pid.update(pitch_setpoint_rad, pitch_.motor.angle_rad, DT_SEC),
        cfg::gimbal.pitch.mit.speed_limit_radps);
    pitch_.loop.ref_radps = pitch_ref_speed_radps;
    pitch_.loop.gravity_nm = cfg::gimbal.pitch.gravity.k_nm == 0.0f
                                 ? 0.0f
                                 : cfg::gimbal.pitch.gravity.k_nm *
                                       pitch_.gravity_cosine.get(pitch_.motor.angle_rad +
                                                                 cfg::gimbal.pitch.gravity.offset_rad);

    const bool hold_roll_during_tf_align = state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN;
    const bool hold_roll_during_recovery = (state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
                                            state == App::Onmi::TransformState::TO_NORMAL_ALIGN);
    const float roll_hold_ref_rad =
        hold_roll_during_tf_align ? tf_.align.roll_hold_rad : tf_.recovery.roll_hold_rad;
    update_roll_control(!(hold_roll_during_tf_align || hold_roll_during_recovery),
                        roll_hold_ref_rad);

    dual_.state.reaction_nm = 0.0f;
    base_.loop.torque_nm =
        clamp_by_abs_limit(base_.loop.torque_nm, cfg::gimbal.base.mit.torque_limit_nm);

    return roll_.ctrl.output().valid && std::isfinite(base_.loop.torque_nm) &&
           std::isfinite(yaw_.loop.cmd_raw) && std::isfinite(pitch_.loop.ref_radps) &&
           std::isfinite(roll_.ref_radps) && std::isfinite(pitch_.loop.gravity_nm) &&
           std::isfinite(roll_.ctrl.output().gravity_torque_nm) &&
           std::isfinite(roll_.ctrl.output().comp_torque_nm);
}

void GimbalCtrl::update_tf_fsm(bool tf_requested, bool normal_recovery_requested)
{
    const std::uint32_t now = Sentry::Time::now_ms();
    const App::Onmi::TransformState state = tf_.fsm.state();

    const bool timed_state = state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN ||
                             state == App::Onmi::TransformState::TO_TRANSFORM_ROLL ||
                             state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
                             state == App::Onmi::TransformState::TO_NORMAL_ALIGN ||
                             state == App::Onmi::TransformState::TO_NORMAL_ROLL;
    if (timed_state && tf_.since_ms != 0U && now - tf_.since_ms > cfg::gimbal.transform.phase_timeout_ms)
    {
        tf_.fsm.force_fault();

        reset_tf_ctrls();
        return;
    }

    if (state == App::Onmi::TransformState::TO_NORMAL_SYNC && !tf_.sync_done)
    {
        capture_normal_sync_refs();
        tf_.sync_done = true;
    }

    bool tf_axes_stable = false;
    bool tf_roll_stable = false;
    bool normal_recovery_ready = !tf_.from_stop;
    bool normal_roll_stable = false;

    const bool updating_normal_recovery_references =
        tf_.from_stop && (state == App::Onmi::TransformState::TO_NORMAL_SYNC ||
                          state == App::Onmi::TransformState::TO_NORMAL_ALIGN);
    if (state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN && state_.chassis_locked)
    {
        update_tf_alignment_references();
    }
    if (updating_normal_recovery_references && state_.chassis_locked)
    {
        update_normal_recovery_references();
        normal_recovery_ready = tf_.recovery.done;
    }

    const bool checking_tf_axes = state == App::Onmi::TransformState::TO_TRANSFORM_ALIGN;
    const bool checking_tf_roll = state == App::Onmi::TransformState::TO_TRANSFORM_ROLL;
    const bool checking_normal_roll = state == App::Onmi::TransformState::TO_NORMAL_ROLL;

    bool reached = false;
    if (checking_tf_axes)
    {
        reached = tf_axes_reached();
    }
    else if (checking_tf_roll)
    {
        reached = tf_roll_reached(cfg::gimbal.roll.reference.transform_rad);
    }
    else if (checking_normal_roll)
    {
        reached = normal_roll_ref_reached();
    }

    if (checking_normal_roll || checking_tf_axes || checking_tf_roll)
    {
        if (reached)
        {
            if (tf_.stable_count < cfg::gimbal.transform.stable_cycles)
            {
                ++tf_.stable_count;
            }
        }
        else
        {
            tf_.stable_count = 0U;
        }
        const bool stable = tf_.stable_count >= cfg::gimbal.transform.stable_cycles;
        tf_axes_stable = checking_tf_axes && stable && state_.chassis_locked;
        tf_roll_stable = checking_tf_roll && stable && state_.chassis_locked;
        normal_roll_stable = checking_normal_roll && stable && state_.chassis_locked;
    }
    else
    {
        tf_.stable_count = 0U;
    }

    tf_.fsm.update(tf_requested, normal_recovery_requested, tf_axes_stable, tf_roll_stable,
                   tf_.sync_done && state_.chassis_locked,
                   normal_recovery_ready && state_.chassis_locked, normal_roll_stable);

    if (!tf_.fsm.changed())
    {
        return;
    }

    tf_.stable_count = 0U;
    tf_.since_ms = now;
    switch (tf_.fsm.state())
    {
    case App::Onmi::TransformState::TO_TRANSFORM_ALIGN:

        capture_tf_refs();
        reset_tf_ctrls();
        break;

    case App::Onmi::TransformState::TO_TRANSFORM_ROLL:
        roll_.ref_rad = cfg::gimbal.roll.reference.transform_rad;

        break;

    case App::Onmi::TransformState::TRANSFORM:
        roll_.ref_rad = cfg::gimbal.roll.reference.transform_rad;

        break;

    case App::Onmi::TransformState::TO_NORMAL_SYNC:

        if (normal_recovery_requested)
        {
            capture_normal_recovery_refs();
        }
        else
        {
            capture_normal_sync_refs();
        }
        tf_.sync_done = true;
        roll_.ref_rad = tf_.recovery.roll_hold_rad;
        reset_tf_ctrls();
        break;

    case App::Onmi::TransformState::TO_NORMAL_ALIGN:
        roll_.ref_rad = tf_.recovery.roll_hold_rad;
        reset_tf_ctrls();
        break;

    case App::Onmi::TransformState::TO_NORMAL_ROLL:
        roll_.ref_rad = cfg::gimbal.roll.reference.normal_rad;

        reset_tf_ctrls();
        break;

    case App::Onmi::TransformState::NORMAL:
        tf_.sync_done = false;
        tf_.from_stop = false;

        tf_.external_pending = true;
        roll_.ref_rad = cfg::gimbal.roll.reference.normal_rad;
        reset_tf_ctrls();
        break;

    case App::Onmi::TransformState::FAULT:
        break;
    }
}

void GimbalCtrl::publish_motion_gate()
{

    App::Chassis::GimbalMotionGate gate{};
    switch (tf_.fsm.state())
    {
    case App::Onmi::TransformState::NORMAL:
        gate.state = App::Chassis::GimbalMotionGateState::NORMAL;
        break;
    case App::Onmi::TransformState::TRANSFORM:
        gate.state = App::Chassis::GimbalMotionGateState::TRANSFORM;
        gate.tf_complete = 1U;
        break;
    case App::Onmi::TransformState::FAULT:
        gate.state = App::Chassis::GimbalMotionGateState::FAULT;
        break;
    case App::Onmi::TransformState::TO_NORMAL_SYNC:
    case App::Onmi::TransformState::TO_NORMAL_ALIGN:
    case App::Onmi::TransformState::TO_NORMAL_ROLL:
        gate.state = App::Chassis::GimbalMotionGateState::TO_NORMAL;
        break;
    case App::Onmi::TransformState::TO_TRANSFORM_ALIGN:
    case App::Onmi::TransformState::TO_TRANSFORM_ROLL:
        gate.state = App::Chassis::GimbalMotionGateState::TO_TRANSFORM;
        break;
    }

    gate.chassis_lock = gate.state != App::Chassis::GimbalMotionGateState::NORMAL &&
                                gate.state != App::Chassis::GimbalMotionGateState::TRANSFORM
                            ? 1U
                            : 0U;
    gate.timestamp = Sentry::Time::now_ms();
    sentry_data.motion_gate.publish(gate);
}

void GimbalCtrl::capture_tf_refs()
{

    tf_.base.ref_rad = Sentry::Utils::normalize_angle_0_2pi(cfg::gimbal.transform.reference.base_rad);
    tf_.yaw.ref_rad = Sentry::Utils::normalize_angle_0_2pi(cfg::gimbal.transform.reference.yaw_rad);
    tf_.pitch.ref_rad = Sentry::Utils::normalize_angle_0_2pi(cfg::gimbal.transform.reference.pitch_rad);
    roll_.ref_rad = cfg::gimbal.roll.reference.normal_rad;

    tf_.align.roll_hold_rad = roll_.motor.angle_rad;
    tf_.align.base.reset(base_.motor.angle_rad, tf_.base.ref_rad);
    tf_.align.yaw.reset(yaw_.motor.angle_rad, tf_.yaw.ref_rad);
    tf_.align.pitch.reset(pitch_.motor.angle_rad, tf_.pitch.ref_rad);
    tf_.align.done = std::fabs(tf_.align.base.delta_rad) <= 0.0000174533f &&
                     std::fabs(tf_.align.yaw.delta_rad) <= 0.0000174533f &&
                     std::fabs(tf_.align.pitch.delta_rad) <= 0.0000174533f;
    tf_.base.planner.reset(0.0f);
    tf_.yaw.planner.reset(0.0f);
    tf_.pitch.planner.reset(0.0f);
}

void GimbalCtrl::update_tf_alignment_references()
{
    if (tf_.align.done)
    {
        return;
    }

    const float max_speed = cfg::gimbal.roll.trajectory.max_speed_radps;
    const float max_accel = cfg::gimbal.roll.trajectory.max_accel_radps2;
    const float max_jerk = cfg::gimbal.roll.trajectory.max_jerk_radps3;
    const bool base_done =
        tf_.align.base.advance(tf_.base.planner, max_speed, max_accel, max_jerk, DT_SEC);
    const bool yaw_done =
        tf_.align.yaw.advance(tf_.yaw.planner, max_speed, max_accel, max_jerk, DT_SEC);
    const bool pitch_done =
        tf_.align.pitch.advance(tf_.pitch.planner, max_speed, max_accel, max_jerk, DT_SEC);
    tf_.align.done = base_done && yaw_done && pitch_done;
}

void GimbalCtrl::capture_normal_sync_refs()
{

    tf_.sync.base_rad = base_.motor.angle_rad;
    tf_.sync.yaw_rad = yaw_.motor.angle_rad;
    tf_.sync.pitch_rad = pitch_.motor.angle_rad;
    tf_.recovery.roll_hold_rad = roll_.motor.angle_rad;
    tf_.from_stop = false;
}

void GimbalCtrl::capture_normal_recovery_refs()
{

    capture_tf_refs();

    tf_.sync.base_rad = base_.motor.angle_rad;
    tf_.sync.yaw_rad = tf_.yaw.ref_rad;
    tf_.sync.pitch_rad = tf_.pitch.ref_rad;
    tf_.recovery.roll_hold_rad = roll_.motor.angle_rad;

    tf_.recovery.base.reset(base_.motor.angle_rad, tf_.sync.base_rad);
    tf_.recovery.yaw.reset(yaw_.motor.angle_rad, tf_.sync.yaw_rad);
    tf_.recovery.pitch.reset(pitch_.motor.angle_rad, tf_.sync.pitch_rad);
    tf_.recovery.done = std::fabs(tf_.recovery.base.delta_rad) <= 0.0000174533f &&
                        std::fabs(tf_.recovery.yaw.delta_rad) <= 0.0000174533f &&
                        std::fabs(tf_.recovery.pitch.delta_rad) <= 0.0000174533f;
    tf_.base.planner.reset(0.0f);
    tf_.yaw.planner.reset(0.0f);
    tf_.pitch.planner.reset(0.0f);
    tf_.from_stop = true;
}

void GimbalCtrl::update_normal_recovery_references()
{
    if (!tf_.from_stop || tf_.recovery.done)
    {
        return;
    }

    const float max_speed = cfg::gimbal.roll.trajectory.max_speed_radps;
    const float max_accel = cfg::gimbal.roll.trajectory.max_accel_radps2;
    const float max_jerk = cfg::gimbal.roll.trajectory.max_jerk_radps3;
    const bool base_done =
        tf_.recovery.base.advance(tf_.base.planner, max_speed, max_accel, max_jerk, DT_SEC);
    const bool yaw_done =
        tf_.recovery.yaw.advance(tf_.yaw.planner, max_speed, max_accel, max_jerk, DT_SEC);
    const bool pitch_done =
        tf_.recovery.pitch.advance(tf_.pitch.planner, max_speed, max_accel, max_jerk, DT_SEC);
    tf_.recovery.done = base_done && yaw_done && pitch_done;
}

bool GimbalCtrl::tf_axes_reached() const
{
    return std::isfinite(base_.motor.angle_rad) && std::isfinite(yaw_.motor.angle_rad) &&
           std::isfinite(pitch_.motor.angle_rad) && std::isfinite(base_.motor.speed_radps) &&
           std::isfinite(yaw_.motor.speed_radps) && std::isfinite(pitch_.motor.speed_radps) &&
           std::fabs(angle_error_rad(tf_.base.ref_rad, base_.motor.angle_rad)) <=
               cfg::gimbal.transform.arrival.angle_tolerance_rad &&
           std::fabs(angle_error_rad(tf_.yaw.ref_rad, yaw_.motor.angle_rad)) <=
               cfg::gimbal.transform.arrival.angle_tolerance_rad &&
           std::fabs(angle_error_rad(tf_.pitch.ref_rad, pitch_.motor.angle_rad)) <=
               cfg::gimbal.transform.arrival.angle_tolerance_rad &&
           std::fabs(base_.motor.speed_radps) <= cfg::gimbal.transform.arrival.speed_tolerance_radps &&
           std::fabs(yaw_.motor.speed_radps) <= cfg::gimbal.transform.arrival.speed_tolerance_radps &&
           std::fabs(pitch_.motor.speed_radps) <= cfg::gimbal.transform.arrival.speed_tolerance_radps;
}

bool GimbalCtrl::tf_roll_reached(float ref_rad) const
{
    return roll_.ctrl.trajectory().done() && std::isfinite(roll_.motor.angle_rad) &&
           std::isfinite(roll_.motor.speed_radps) &&
           std::fabs(angle_error_rad(ref_rad, roll_.motor.angle_rad)) <= cfg::gimbal.transform.arrival.angle_tolerance_rad &&
           std::fabs(roll_.motor.speed_radps) <= cfg::gimbal.transform.arrival.speed_tolerance_radps;
}

bool GimbalCtrl::normal_roll_ref_reached() const
{
    return roll_.ctrl.trajectory().done() && std::isfinite(roll_.motor.angle_rad) &&
           std::isfinite(roll_.motor.speed_radps) &&
           std::fabs(angle_error_rad(cfg::gimbal.roll.reference.normal_rad, roll_.motor.angle_rad)) <=
               cfg::gimbal.transform.arrival.angle_tolerance_rad &&
           std::fabs(roll_.motor.speed_radps) <= cfg::gimbal.transform.arrival.speed_tolerance_radps;
}

void GimbalCtrl::reset_tf_ctrls()
{
    tf_.base.angle_pid.reset();
    tf_.yaw.angle_pid.reset();
    tf_.yaw_speed_pid.reset();
    tf_.pitch.angle_pid.reset();
}

namespace
{

constexpr std::uint32_t kVisionInitialAcquireMaxMs = 300U;

}

void GimbalCtrl::update_vis_base_planner(float dt_sec)
{

    if (!std::isfinite(dt_sec) || dt_sec <= 0.0f || !std::isfinite(base_.loop.speed_radps) ||
        !std::isfinite(vision_.q.request_rad) || !std::isfinite(vision_.q.predicted_rad) ||
        !std::isfinite(yaw_.aim_rad))
    {
        vision_.trend.accel_radps2 = 0.0f;
        vision_.q.center_speed_radps = 0.0f;
        base_.loop.raw_ref_radps = 0.0f;
        base_.loop.ref_radps = 0.0f;
        return;
    }

    const std::uint32_t dt_ms =
        std::max<std::uint32_t>(1U, static_cast<std::uint32_t>(dt_sec * 1000.0f));
    if (!vision_.base.ready)
    {
        vision_.planner.reset(base_.loop.speed_radps, 0.0f);
        vision_.base.ready = true;
        record_vis_base_state_switch(vision_.base.centering ? VisionBigYawState::ACQUIRE
                                                            : VisionBigYawState::TRACK);
        vision_.base.acquire_ms = 0U;
        vision_.base.hold_ms = 0U;
        if (!std::isfinite(vision_.trend.angle_rad))
        {
            vision_.trend.angle_rad = yaw_.aim_rad;
        }
    }

    const float q_error = vision_.q.predicted_rad - cfg::gimbal.vision.q.center_rad;
    vision_.q.error_rad = q_error;
    const float q_abs = std::fabs(q_error);
    const float q_start = std::fabs(cfg::gimbal.vision.q.soft_start_rad);
    const float q_full = std::fmax(q_start + 1.0e-4f, std::fabs(cfg::gimbal.vision.q.soft_full_rad));
    float q_weight = 0.0f;
    if (q_abs >= q_full)
    {
        q_weight = 1.0f;
    }
    else if (q_abs > q_start)
    {
        const float x = (q_abs - q_start) / (q_full - q_start);
        q_weight = x * x * (3.0f - 2.0f * x);
    }
    vision_.q.center_weight = q_weight;

    const float normal_wn = std::fmax(cfg::gimbal.vision.base.trend.bandwidth_rad_s, 0.01f);
    const float fast_wn = std::fmax(cfg::gimbal.vision.base.fast_trend.bandwidth_rad_s, normal_wn);
    const float wn = normal_wn + (fast_wn - normal_wn) * q_weight;
    const float zeta = std::fmax(cfg::gimbal.vision.base.trend.damping, 0.1f);
    const float trend_error = angle_error_rad(yaw_.aim_rad, vision_.trend.angle_rad);

    vision_.trend.accel_radps2 =
        wn * wn * trend_error - 2.0f * zeta * wn * vision_.trend.speed_radps;
    vision_.trend.accel_radps2 =
        clamp_by_abs_limit(vision_.trend.accel_radps2, cfg::gimbal.vision.base.trend.accel_limit_radps2);
    vision_.trend.speed_radps += vision_.trend.accel_radps2 * dt_sec;
    vision_.trend.speed_radps =
        clamp_speed_radps(vision_.trend.speed_radps, cfg::gimbal.vision.base.speed_limit_radps);
    vision_.trend.angle_rad =
        wrap_angle_rad(vision_.trend.angle_rad + vision_.trend.speed_radps * dt_sec);

    vision_.q.center_speed_radps = cfg::gimbal.vision.q.centering_kp_radps_per_rad * q_error * q_weight;

    const float trend_base_speed = vision_.trend.speed_radps;
    vision_.trend.base_speed_radps = trend_base_speed;
    const float unplanned_speed = clamp_speed_radps(trend_base_speed + vision_.q.center_speed_radps,
                                                    cfg::gimbal.vision.base.speed_limit_radps);
    vision_.base.raw_ref_radps = unplanned_speed;
    base_.loop.raw_ref_radps = unplanned_speed;
    base_.loop.ref_radps = vision_.planner.update(base_.loop.raw_ref_radps, dt_sec);

    vision_.base.acquire_ms =
        std::min<std::uint32_t>(0xFFFFFFFFU - dt_ms, vision_.base.acquire_ms) + dt_ms;

    if (vision_.state != VisionBigYawState::ACQUIRE)
    {
        return;
    }

    const bool q_centered = std::fabs(q_error) <= std::fabs(cfg::gimbal.vision.acquire.q_tol_rad);
    const bool base_slow =
        std::fabs(base_.loop.speed_radps) <= std::fabs(cfg::gimbal.vision.acquire.base_speed_tol_radps);
    const bool yaw_slow =
        std::fabs(yaw_.loop.speed_radps) <= std::fabs(cfg::gimbal.vision.acquire.yaw_speed_tol_radps);
    if (q_centered && base_slow && yaw_slow)
    {
        vision_.base.hold_ms =
            std::min<std::uint32_t>(0xFFFFFFFFU - dt_ms, vision_.base.hold_ms) + dt_ms;
        if (vision_.base.hold_ms >= cfg::gimbal.vision.acquire.acquire_hold_ms)
        {
            vision_.base.centering = false;
            record_vis_base_state_switch(VisionBigYawState::TRACK);
            vision_.base.acquire_ms = 0U;
            vision_.base.hold_ms = 0U;
        }
    }
    else
    {
        vision_.base.hold_ms = 0U;
    }

    if (vision_.base.acquire_ms >= kVisionInitialAcquireMaxMs)
    {
        vision_.base.centering = false;
        record_vis_base_state_switch(VisionBigYawState::TRACK);
        vision_.base.hold_ms = 0U;
    }
}

void GimbalCtrl::record_vis_base_state_switch(VisionBigYawState next)
{
    if (vision_.state == next)
    {
        return;
    }
    vision_.state = next;
}

void GimbalCtrl::reset_vis_base_planner()
{
    vision_.planner.reset(0.0f, 0.0f);
    vision_.angle_hold.clear();
    vision_.state = VisionBigYawState::ACQUIRE;
    vision_.base = {};
    vision_.trend = {};
    vision_.q = {};
}

void GimbalCtrl::update_vis_yaw_alignment()
{
    if (vision_.alignment.valid || !state_.main_imu_online || !state_.base_imu_online ||
        !yaw_.motor.online || !std::isfinite(yaw_.angle_rad) || !std::isfinite(base_.angle_rad))
    {
        return;
    }

    const float q_rad = current_yaw_offset_rad();
    if (!std::isfinite(q_rad))
    {
        return;
    }

    vision_.alignment.offset_rad = wrap_angle_rad(yaw_.angle_rad - base_.angle_rad - q_rad);
    vision_.alignment.valid = std::isfinite(vision_.alignment.offset_rad);
}

void GimbalCtrl::begin_vis_init_lock_centering()
{
    if (!std::isfinite(yaw_.angle_rad) || !std::isfinite(yaw_.aim_rad))
    {
        vision_.base.centering = false;
        return;
    }

    vision_.base.init_rad = yaw_.aim_rad;
    vision_.base.centering = true;
    record_vis_base_state_switch(VisionBigYawState::ACQUIRE);
    vision_.base.hold_ms = 0U;
    vision_.base.acquire_ms = 0U;
    vision_.trend.angle_rad = vision_.base.init_rad;
    vision_.trend.speed_radps = 0.0f;
    vision_.trend.accel_radps2 = 0.0f;
}

void GimbalCtrl::publish_control(bool enable)
{
    const std::uint32_t now = Sentry::Time::now_ms();
    const bool tf_dm_control = tf_.fsm.owns_control();

    Sentry::DMMITCommand base_cmd{};
    base_cmd.pos_rad = 0.0f;
    base_cmd.vel_radps =
        enable && tf_dm_control
            ? clamp_speed_radps(base_.loop.ref_radps, cfg::gimbal.base.mit.speed_limit_radps)
            : 0.0f;
    base_cmd.kp = 0.0f;
    base_cmd.kd = tf_dm_control ? cfg::gimbal.transform.base.mit_kd : cfg::gimbal.base.mit.kd;
    base_cmd.torque_nm =
        enable ? clamp_by_abs_limit((tf_dm_control ? 0.0f : base_.loop.torque_nm) +
                                        (tf_dm_control ? cfg::gimbal.base.mit.torque_offset_nm
                                                       : cfg::gimbal.base.mit.torque_offset_nm),
                                    tf_dm_control ? cfg::gimbal.base.mit.torque_limit_nm
                                                  : cfg::gimbal.base.mit.torque_limit_nm)
               : 0.0f;
    base_cmd.enable = enable;
    base_cmd.timestamp = now;
    {
        sentry_data.base_mit.publish(base_cmd);
    }

    Sentry::DMMITCommand pitch_cmd{};
    pitch_cmd.pos_rad = 0.0f;
    pitch_cmd.vel_radps =
        enable ? clamp_speed_radps(pitch_.loop.ref_radps, tf_dm_control
                                                              ? cfg::gimbal.pitch.mit.speed_limit_radps
                                                              : cfg::gimbal.pitch.mit.speed_limit_radps)
               : 0.0f;
    pitch_cmd.kp = 0.0f;
    pitch_cmd.kd = tf_dm_control ? cfg::gimbal.transform.pitch.mit_kd : cfg::gimbal.pitch.mit.kd;
    pitch_cmd.torque_nm =
        enable ? clamp_by_abs_limit(pitch_.loop.gravity_nm +
                                        (tf_dm_control ? cfg::gimbal.pitch.mit.torque_offset_nm
                                                       : cfg::gimbal.pitch.mit.torque_offset_nm),
                                    tf_dm_control ? cfg::gimbal.pitch.mit.torque_limit_nm
                                                  : cfg::gimbal.pitch.mit.torque_limit_nm)
               : 0.0f;
    pitch_cmd.enable = enable;
    pitch_cmd.timestamp = now;
    {
        sentry_data.pitch_mit.publish(pitch_cmd);
    }

    Sentry::DMMITCommand roll_cmd{};
    roll_cmd.pos_rad = 0.0f;
    roll_cmd.vel_radps =
        enable ? clamp_speed_radps(roll_.ref_radps, cfg::gimbal.roll.mit.speed_limit_radps) : 0.0f;
    roll_cmd.kp = 0.0f;

    const auto &roll_output = roll_.ctrl.output();
    roll_cmd.kd = enable && roll_output.valid ? roll_output.damping_nms_rad : 0.0f;
    roll_cmd.torque_nm = enable
                             ? clamp_by_abs_limit(roll_output.valid ? roll_output.mit_ff_nm : 0.0f,
                                                  cfg::gimbal.roll.mit.torque_limit_nm)
                             : 0.0f;
    roll_cmd.enable = enable;
    roll_cmd.timestamp = now;
    {
        sentry_data.roll_mit.publish(roll_cmd);
    }
    roll_.ctrl.set_applied_command(roll_cmd.vel_radps, roll_cmd.kd, roll_cmd.torque_nm,
                                   roll_output.gravity_torque_nm, enable && roll_output.valid);

    {
        sentry_data.base_cmd.publish(Sentry::MotorCommand{0, enable});
    }
    const std::int16_t yaw_final_command = enable ? yaw_.loop.cmd_raw : 0;
    {
        sentry_data.yaw_cmd.publish(Sentry::MotorCommand{yaw_final_command, enable});
    }

    const float applied_yaw_torque_nm =
        enable ? yaw_.torque_converter.command_to_torque_Nm(yaw_final_command) : 0.0f;
    yaw_.auto_ctrl.set_applied_torque_nm(
        (state_.fsm.state() == OnmiGimbalMode::VISION || false) ? applied_yaw_torque_nm : 0.0f);
    {
        sentry_data.pitch_cmd.publish(Sentry::MotorCommand{0, enable});
    }
    {
        sentry_data.roll_cmd.publish(Sentry::MotorCommand{0, enable});
    }

    const bool tf_fb = tf_.fsm.owns_control();
    const float gate_speed = tf_fb ? base_.motor.speed_radps : base_.loop.speed_radps;
    const App::Onmi::GimbalMotionStatus motionStatus{
        std::isfinite(gate_speed) ? gate_speed : 0.0f,
        std::isfinite(base_.loop.ref_radps) ? base_.loop.ref_radps : 0.0f, now};
    App::Onmi::gimbalMotionStatus.publish(motionStatus);
}

void GimbalCtrl::publish_heading(const Sentry::IMUData &base_imu)
{
    if (false || base_imu.timestamp == 0U || base_imu.timestamp == state_.heading_ms ||
        !base_imu.is_online(SAFETY_BASE_IMU_TIMEOUT_MS))
    {
        return;
    }
    state_.heading_ms = base_imu.timestamp;

    const float yaw_deg = base_imu.angle_deg[cfg::gimbal.input.base_imu_yaw_axis];
    if (!std::isfinite(yaw_deg))
    {
        return;
    }

    const float yaw_rad =
        wrap_angle_rad(Sentry::Utils::deg_to_rad(yaw_deg) + cfg::gimbal.integration.heading_offset_rad);
    if (!std::isfinite(yaw_rad))
    {
        return;
    }

    sentry_data.heading.publish(App::Chassis::HeadingFeedback{yaw_rad, base_imu.timestamp});
}

void GimbalCtrl::publish_yaw_joint(const Sentry::MotorFeedback &base_fb)
{
    if (false || base_fb.timestamp == 0U || base_fb.timestamp == state_.joint_ms ||
        !base_fb.is_online(SAFETY_YAW_FB_TIMEOUT_MS) || !std::isfinite(base_fb.output_angle_rad))
    {
        return;
    }
    state_.joint_ms = base_fb.timestamp;

    App::Chassis::GimbalYawJointFeedback feedback{};
    feedback.yaw_angle_rad = Sentry::Utils::normalize_angle_0_2pi(base_fb.output_angle_rad);
    feedback.timestamp = base_fb.timestamp;
    sentry_data.yaw_joint.publish(feedback);
}

void GimbalCtrl::publish_stop()
{

    const bool preserve_sentry_phase =
        state_.fsm.state() == OnmiGimbalMode::SENTRY && sentry_.ready;
    const std::uint32_t saved_sentry_started_at_ms = sentry_.since_ms;
    reset_pids();
    if (preserve_sentry_phase)
    {
        sentry_.since_ms = saved_sentry_started_at_ms;
        sentry_.ready = true;
    }
    publish_control(false);
}

unsigned GimbalCtrl::GetVofaData(float *out, bool full) const
{
    const auto v = sentry_data.vision.read();
    const auto mit = sentry_data.base_mit.read();
    const bool transform = tf_.fsm.owns_control();
    const bool sentry = state_.fsm.state() == OnmiGimbalMode::SENTRY;
    const float yaw_fb = transform || sentry ? yaw_.motor.angle_rad : yaw_.angle_rad;
    const float pitch_fb = transform || (sentry && !cfg::gimbal.sentry.pitch.use_imu_fb)
                               ? pitch_.motor.angle_rad
                               : pitch_.angle_rad;
    if (!full)
    {
        const float data[] = {v.yaw_angle_rad,
                              wrap_angle_rad(yaw_fb),
                              wrap_angle_rad(yaw_.ref_rad),
                              wrap_angle_rad(v.yaw_angle_rad - yaw_fb),
                              yaw_.loop.ref_radps,
                              yaw_.loop.speed_radps,
                              v.pitch_angle_rad,
                              wrap_angle_rad(pitch_fb)};
        std::copy(data, data + 8, out);
        return 8;
    }
    const float data[] = {
        v.yaw_angle_rad,
        v.yaw_speed_radps,
        v.yaw_accel_radps2,
        wrap_angle_rad(yaw_fb),
        yaw_.loop.speed_radps,
        wrap_angle_rad(v.yaw_angle_rad - yaw_fb),
        wrap_angle_rad(yaw_.ref_rad),
        yaw_.loop.ref_radps,
        v.pitch_angle_rad,
        v.pitch_speed_radps,
        v.pitch_accel_radps2,
        wrap_angle_rad(pitch_fb),
        pitch_.speed_radps,
        wrap_angle_rad(v.pitch_angle_rad - pitch_fb),
        wrap_angle_rad(pitch_.ref_rad),
        pitch_.loop.ref_radps,
        float(v.control_mode),
        float(v.vis_ready),
        mit.torque_nm,
        dual_.state.reaction_nm,
        base_.loop.dist_torque_nm,
        base_.loop.ref_radps,
        base_.loop.speed_radps,
        wrap_angle_rad(base_.angle_rad),
        base_.motor.angle_rad,
        dual_.state.speed_radps,
        dual_.reaction_ff.rel_accel_radps2(),
        dual_.reaction_ff.inertia_comp_nm(),
        yaw_.torque_converter.command_to_torque_Nm(sentry_data.yaw_cmd.read().current)};
    std::copy(data, data + 29, out);
    return 29;
}
