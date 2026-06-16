#include "ControlTask.hpp"

#include <cmath>
#include <math.h>

using namespace ALG::PID;
using namespace ALG::ADRC;
using namespace Alg::Feedforward;


float debug_pitch1_motor_deg = 0.0f;


/* 基础工具与参数 ----------------------------------------------------------------------------------------*/
namespace
{
// 圆周率，所有内部角度计算默认使用弧度制。
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

// 控制任务周期，单位 s，需要和 FreeRTOS 中 Control 任务的 osDelay(1) 对齐。
constexpr float kControlPeriod = 0.001f;

// 各轴目标速度限幅，避免外环目标过大导致底层电机瞬间冲击。
constexpr float kYawVelocityLimit = 5.0f;
constexpr float kPitch2VelocityLimit = 5.0f;

// MIT 扭矩通道限幅，和达妙电机参数表中的扭矩范围保持安全余量。
constexpr float kYawTorqueLimit = 5.0f;
constexpr float kPitch1TorqueLimit = 6.0f;
constexpr float kPitch2TorqueLimit = 5.0f;

// 手动速度模式下，遥控摇杆/鼠标输入到目标角速度的比例系数。
constexpr float kYawManualStickScale = 5.0f;
constexpr float kPitch2ManualStickScale = 3.0f;
constexpr float kYawManualMouseScale = 0.010f;
constexpr float kPitch2ManualMouseScale = 0.010f;

// 角度模式下，遥控输入先积分成角度目标，这里是角度目标变化率比例。
constexpr float kYawAngleStickRate = 2.5f;
constexpr float kPitch2AngleStickRate = 2.5f;
constexpr float kYawAngleMouseRate = 0.0030f;
constexpr float kPitch2AngleMouseRate = 0.0030f;

// Pitch2 全模式软件限位，基于 IMU 绝对俯仰角限制在 +/-30 度。
constexpr float kPitch2AngleLimitRad = 30.0f * kDegToRad;
constexpr float kPitch2AngleMin = -kPitch2AngleLimitRad;
constexpr float kPitch2AngleMax = kPitch2AngleLimitRad;

// 开机起立目标：Pitch1 抬到 70度后，再把云台控制权交给操作手。
constexpr float kBigPitchTargetRad = 70.0f * kDegToRad;
constexpr float kStandupToleranceDeg = 3.0f;
constexpr float kTransformYawTargetEncoderRad = 2.3508f;

// 通用限幅工具：把 value 限制在 [min_value, max_value] 内。
float ClampValue(float value, float min_value, float max_value)
{
    if (!std::isfinite(value))
    {
        return 0.0f;
    }
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

float HoldLastFinite(float value, float &last_value)
{
    if (std::isfinite(value))
    {
        last_value = value;
    }
    return last_value;
}

// Pitch2 目标角限幅：使用 IMU 绝对角度坐标系，限制在 +/-30 度。
float ClampPitch2AngleTarget(float angle_rad)
{
    return ClampValue(angle_rad, kPitch2AngleMin, kPitch2AngleMax);
}

// Pitch2 执行层限位：到达 IMU 角度边界后，只允许速度命令往安全方向返回。
float LimitPitch2VelocityByAngle(float current_angle_rad, float velocity_cmd)
{
    if (current_angle_rad >= kPitch2AngleMax && velocity_cmd > 0.0f)
    {
        return 0.0f;
    }
    if (current_angle_rad <= kPitch2AngleMin && velocity_cmd < 0.0f)
    {
        return 0.0f;
    }
    return velocity_cmd;
}

// Yaw 角度归一化工具：把角度折回 [-pi, pi]，避免跨零点时外环走远路。
float WrapAngleRad(float angle)
{
    if (!std::isfinite(angle))
    {
        return 0.0f;
    }
    while (angle > kPi)
    {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0f * kPi;
    }
    return angle;
}

// 获取 Yaw 空间绝对角度，单位 rad，用 IMU 做视觉/角度模式锁定。
float GetYawAngle()
{
    static float last_yaw_angle = 0.0f;
    return HoldLastFinite(HI12.GetAngle(2) * kDegToRad, last_yaw_angle);
}

// 获取 Yaw 空间角速度，单位 rad/s，内外环统一使用 IMU 反馈。
float GetYawVel()
{
    static float last_yaw_vel = 0.0f;
    return HoldLastFinite(HI12.GetGyroRad(2), last_yaw_vel);
}

// 获取 Pitch1 电机当前角度，单位 rad。
float GetPitch1Angle()
{
    static float last_pitch1_angle = 0.0f;
    return HoldLastFinite(MotorJ4340.getAngleRad(1), last_pitch1_angle);
}

// 获取 Pitch1 电机当前角度，单位 degree，供重力前馈使用。
float GetPitch1AngleDeg()
{
    static float last_pitch1_angle_deg = 0.0f;
    return HoldLastFinite(MotorJ4340.getAngleDeg(1), last_pitch1_angle_deg);
}

// 获取 Pitch1 电机当前角速度，单位 rad/s，供 VMC 阻尼项使用。
float GetPitch1Vel()
{
    static float last_pitch1_vel = 0.0f;
    return HoldLastFinite(MotorJ4340.getVelocityRads(1), last_pitch1_vel);
}

// 获取 Pitch2 空间绝对角度，单位 rad，供角度 PID 使用。
float GetPitch2Angle()
{
    static float last_pitch2_angle = 0.0f;
    return HoldLastFinite(HI12.GetAngle(1) * kDegToRad, last_pitch2_angle);
}

// 获取 Pitch2 空间绝对角度，单位 degree，供重力前馈使用。
float GetPitch2AngleDeg()
{
    static float last_pitch2_angle_deg = 0.0f;
    return HoldLastFinite(HI12.GetAngle(1), last_pitch2_angle_deg);
}

// 获取 Pitch2 空间角速度，单位 rad/s，内外环统一使用 IMU 反馈。
float GetPitch2Vel()
{
    static float last_pitch2_vel = 0.0f;
    return HoldLastFinite(HI12.GetGyroRad(0), last_pitch2_vel);
}

 
// float Sin_target(float addition_angle, int time){
//     static int tick = 0;
//     static int direct = 1;
//     if (tick > time) {
//         direct *= -1;
//         tick = 0;
//     }
//     tick++;
//     return addition_angle * direct;

// }

}  // namespace

/* 全局对象与控制器 --------------------------------------------------------------------------------------*/
// 发射相关标志位，当前文件保留外部接口，避免其他任务引用断开。
bool shoot = false;

// 云台期望值集合：SetTarget_Gimbal() 只负责更新目标，不直接输出到电机。
GimbalTarget_t cmd = {
    GIMBAL_MODE_VELOCITY,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    {0.0f, 0.0f}
};

// 开机起立锁定状态：从 STOP 唤醒时记录 IMU 初始姿态，起立完成前屏蔽操控输入。
bool is_standup_complete = false;
static float init_pitch2_angle = 0.0f;
static float init_yaw_angle = 0.0f;
static TDFilter yaw_vision_td(50.0f, kControlPeriod);

// 云台输出集合：MotorTask 只读取这里的结果并通过 CAN 下发。
GimbalOutput_t out = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 云台有限状态机，负责 STOP / MANUAL / VISION / TRANSFORM 等状态流转。
Gimbal_FSM gimbal_fsm;

// Yaw 速度环 ADRC：目标是角速度，输出为 MIT 扭矩通道。
FirstLADRC yaw_adrc(25.0f, 70.0f, 105.0f, kControlPeriod, 800.0f, 3.0f);

// Pitch2 速度环 ADRC：速度模式下输出目标扭矩，角度模式下由达妙底层速度环接管。
FirstLADRC pitch2_adrc(25.0f, 75.0f, 150.0f, 0.001, 500.0f, kPitch2TorqueLimit);

// Yaw 角度外环 PID：角度误差 -> 目标角速度。
PID yaw_transform_angle_pid(16.0f, 0.002f, 0.0f, kYawVelocityLimit, 100.0f, 0.0f);

// Yaw 变形专用外环 PID：仅供 TRANSFORM 模式使用，基于电机绝对编码器反馈。
PID yaw_encoder_transform_pid(16.0f, 0.002f, 0.0f, kYawVelocityLimit, 100.0f, 0.0f);

// 预留 Yaw ADRC 实例，保留原接口，方便后续调参/切换实验。
FirstLADRC yaw_adrc_try(25.0f, 160.0f, 1.0f, kControlPeriod, 80.0f, kYawTorqueLimit);

// Yaw 复合前馈对象，当前版本保留对象但主控制暂未叠加。
GimbalFullCompensation gimbal_yaw(0.0f, kControlPeriod, 0.0f, 0.0f);

// Pitch1 斜坡轨迹规划器：把目标角度阶跃转换为平滑位置轨迹，避免起立时暴力拔枪。
static TrajectoryPlanner pitch1_planner(2.5f, 12.0f, kControlPeriod);
static TrajectoryPlanner yaw_transform_planner(3.0f, 15.0f, kControlPeriod);

//sin波发生器
//static SineWaveGenerator yaw_test_gen(2.0f, 1.0f, 0.0f, kControlPeriod);


// Pitch1 重力前馈，用于补偿大 Pitch 自重，最终从 MIT 扭矩通道下发。
static Gravity gravity_forward_pitch1(2.0f, 20.0f);

// Pitch2 角度外环：角度模式下目标角度 -> 目标速度 V_des。
PID pitch2_angle_pid(40.0f, 0.0f, 0.0f, kPitch2VelocityLimit, 0.0f, 0.0f);

// 手动速度目标滤波，减少遥控输入突变带来的目标速度阶跃。
static TDFilter yaw_manual_filter(30.0f, kControlPeriod);
static TDFilter pitch2_manual_filter(100.0f, kControlPeriod);

// Pitch2 重力前馈。注意 GravityFeedforward() 内部按 degree 计算三角函数。
Gravity gravity_forward_pitch2(0.35f, 0.0f);

/* 模式切换与辅助函数 ------------------------------------------------------------------------------------*/
// 计算 Pitch2 重力补偿扭矩，输入必须是 degree，不能直接传 rad。
static float GetPitch2GravityTorque()
{
    gravity_forward_pitch2.GravityFeedforward(GetPitch2AngleDeg());
    return gravity_forward_pitch2.getFeedforward();
}

// 将 Pitch1 目标同步到当前角度，用于切模式时避免 Pitch1 目标跳变。
static void ResetPitch1HoldTarget()
{
    pitch1_planner.Reset(GetPitch1Angle());
}

// 切入速度模式时同步目标和控制器状态，避免从角度模式切回来时突然给速度。
static void SyncVelocityModeTargets()
{
    cmd.current_mode = GIMBAL_MODE_VELOCITY;
    cmd.yaw_vel = 0.0f;
    cmd.pitch2_vel = 0.0f;
    yaw_manual_filter.reset(0.0f);
    pitch2_manual_filter.reset(0.0f);
    yaw_adrc.Reset(GetYawVel(), 0.0f);
    const float current_pitch2_vel = GetPitch2Vel();
    pitch2_adrc.Reset(current_pitch2_vel, 0.0f);
    pitch2_angle_pid.reset();
    if (is_standup_complete)
    {
        ResetPitch1HoldTarget();
    }
}

// 切入角度模式时把角度目标对齐当前反馈，避免角度外环瞬间产生大误差。
static void SyncAngleModeTargets()
{
    cmd.current_mode = GIMBAL_MODE_ANGLE;
    cmd.yaw_angle = GetYawAngle();
    cmd.pitch2_angle = ClampPitch2AngleTarget(GetPitch2Angle());
    yaw_vision_td.reset(0.0f);
    yaw_transform_angle_pid.reset();
    pitch2_angle_pid.reset();
    yaw_adrc.Reset(GetYawVel(), 0.0f);
    if (is_standup_complete)
    {
        ResetPitch1HoldTarget();
    }
}

// Pitch1 单轴 VMC：规划目标位置/速度 -> 虚拟弹簧阻尼扭矩 + 重力前馈。
static void UpdatePitch1Control()
{
    const float current_angle = GetPitch1Angle();
    const float current_vel = GetPitch1Vel();

    const float target_angle = pitch1_planner.Update(cmd.pitch1);
    const float target_vel = pitch1_planner.GetVelocity();
    out.pitch1_vel = target_vel;

    gravity_forward_pitch1.GravityFeedforward(GetPitch1AngleDeg() + GimbalConfig::kPitch1GravityOffsetDeg);
    const float tau_gravity = gravity_forward_pitch1.getFeedforward();

    float spring_kp = GimbalConfig::kPitch1_VMC_Kp;
    float damper_kd = GimbalConfig::kPitch1_VMC_Kd;
    float final_tau_gravity = tau_gravity;

    if (gimbal_fsm.Get_Now_State() == TRANSFORM)
    {
        const bool is_dropping_allowed =
            (cmd.pitch1 < GimbalConfig::kPitch1TransformSafeDropDeg * kDegToRad);
        const bool is_in_landing_zone =
            (current_angle < GimbalConfig::kPitch1TransformLandingDeg * kDegToRad);

        if (is_dropping_allowed && is_in_landing_zone)
        {
            spring_kp = 0.0f;
            damper_kd *= 2.0f;
            final_tau_gravity *= 0.0f;
        }
        else
        {
            spring_kp *= 0.4f;
            damper_kd *= 2.0f;
        }
    }

    const float tau_spring = spring_kp * (target_angle - current_angle);
    const float tau_damper = damper_kd * (target_vel - current_vel);
    const float total_torque = tau_spring + tau_damper + final_tau_gravity;

    out.pitch1_torq = ClampValue(total_torque, -kPitch1TorqueLimit, kPitch1TorqueLimit);
}

// 起立到位检测：Pitch1 抬到目标附近后，同步当前姿态并解除遥控输入屏蔽。
static void UpdateStandupState()
{
    if (!is_standup_complete)
    {
        if (std::fabs(GetPitch1Angle() - kBigPitchTargetRad) < (kStandupToleranceDeg * kDegToRad))
        {
            is_standup_complete = true;
            cmd.pitch2_angle = ClampPitch2AngleTarget(GetPitch2Angle());
            cmd.yaw_angle = GetYawAngle();
            cmd.pitch2_vel = 0.0f;
            cmd.yaw_vel = 0.0f;
            yaw_manual_filter.reset(0.0f);
            pitch2_manual_filter.reset(0.0f);
            yaw_vision_td.reset(0.0f);
            yaw_adrc.Reset(GetYawVel(), 0.0f); 
            const float current_pitch2_vel = GetPitch2Vel();
            pitch2_adrc.Reset(current_pitch2_vel, 0.0f);
            yaw_transform_angle_pid.reset();
        }
    }
}

/* 目标设定 ----------------------------------------------------------------------------------------------*/
// 速度模式目标设定：Yaw 和 Pitch2 都生成目标角速度。
void velocity_SetTarget()
{
    cmd.current_mode = GIMBAL_MODE_VELOCITY;

    if (!is_standup_complete)
    {
        return;
    }

    const float yaw_input =
        -DT7.get_right_x() * kYawManualStickScale +
        static_cast<float>(DT7.get_mouseX()) * kYawManualMouseScale;
    const float pitch2_input =
        DT7.get_right_y() * kPitch2ManualStickScale +
        static_cast<float>(DT7.get_mouseY()) * kPitch2ManualMouseScale;

    cmd.yaw_vel =
        ClampValue(yaw_manual_filter.filter(yaw_input), -kYawVelocityLimit, kYawVelocityLimit);
    
//    cmd.yaw_vel =  yaw_test_gen.Update();

    cmd.pitch2_vel =
        ClampValue(pitch2_manual_filter.filter(pitch2_input), -kPitch2VelocityLimit, kPitch2VelocityLimit);
}

// 角度模式目标设定：遥控/鼠标输入先积分成角度目标，再交给角度外环。
void Angle_SetTarget()
{
    cmd.current_mode = GIMBAL_MODE_ANGLE;

    if (!is_standup_complete)
    {
        return;
    }

    // 先滤波每周期角度增量，再叠加到绝对角度目标，避免 TD 记住 +/-pi 过零历史。
    const float raw_yaw_delta =
        (-DT7.get_right_x() * kYawAngleStickRate +
         static_cast<float>(DT7.get_mouseX()) * kYawAngleMouseRate) * kControlPeriod;
    const float filtered_yaw_delta = yaw_vision_td.filter(raw_yaw_delta);
    const float pitch2_delta =
        (DT7.get_right_y() * kPitch2AngleStickRate +
         static_cast<float>(DT7.get_mouseY()) * kPitch2AngleMouseRate) * kControlPeriod;

    cmd.yaw_angle = WrapAngleRad(cmd.yaw_angle + filtered_yaw_delta);
    cmd.pitch2_angle = ClampPitch2AngleTarget(
        cmd.pitch2_angle + pitch2_delta);
}

// 云台目标调度入口：根据当前状态刷新控制目标，并处理退出变形后的自动起立保护。
void SetTarget_Gimbal()
{
    static Enum_Gimbal_States last_state = STOP;
    const Enum_Gimbal_States current_state = gimbal_fsm.Get_Now_State();

    if (current_state != last_state)
    {
        if ((last_state == STOP && current_state != STOP) ||
            (last_state == TRANSFORM && current_state != TRANSFORM && current_state != STOP))
        {
            is_standup_complete = false;
            init_pitch2_angle = ClampPitch2AngleTarget(GetPitch2Angle());
            init_yaw_angle = GetYawAngle();
            cmd.pitch1 = kBigPitchTargetRad;
            pitch1_planner.Reset(GetPitch1Angle());
        }

        // 模式变化时先同步目标，减少角度控和速度控来回切换时的突跳。
        if (current_state == MANUAL || current_state == KEYBOARD)
        {
            SyncVelocityModeTargets();
        }
        else if (current_state == VISION)
        {
            SyncAngleModeTargets();
        }
        else if (is_standup_complete && current_state != TRANSFORM)
        {
            ResetPitch1HoldTarget();
        }

        last_state = current_state;
    }

    switch (current_state)
    {
    case STOP:
        cmd.current_mode = GIMBAL_MODE_VELOCITY;
        cmd.yaw_vel = 0.0f;
        cmd.pitch2_vel = 0.0f;
        break;

    case MANUAL:
    case KEYBOARD:
        velocity_SetTarget();
        break;

    case VISION:
        Angle_SetTarget();
        break;

    case TRANSFORM:
        // 变形模式下彻底断开遥控器积分，由 gimbal_transform() 接管。
        break;

    default:
        break;
    }
}

/* 控制器执行 --------------------------------------------------------------------------------------------*/
// 速度控制器：Yaw 和 Pitch2 都走 ADRC 纯扭矩，Pitch2 额外叠加重力前馈。
static void velocity_Control()
{
    debug_pitch1_motor_deg = MotorJ4340.getAngleDeg(1);
    UpdateStandupState();

    const float current_yaw_vel = GetYawVel();
    const float current_pitch2_vel = GetPitch2Vel();
    const float current_pitch2_angle = GetPitch2Angle();
    const float pitch2_gravity_torque = GetPitch2GravityTorque();

    UpdatePitch1Control();

    // 起立阶段：Yaw 继续走 ADRC 扭矩锁定，Pitch2 改为角度外环速度透传给达妙底层。
    if (!is_standup_complete)
    {
        const float current_yaw_angle = GetYawAngle();
        const float yaw_error = WrapAngleRad(init_yaw_angle - current_yaw_angle);
        const float virtual_yaw_angle = current_yaw_angle + yaw_error;
        
        const float hold_yaw_vel = ClampValue(
            yaw_transform_angle_pid.UpDate(virtual_yaw_angle, current_yaw_angle),
            -kYawVelocityLimit,
            kYawVelocityLimit);
        const float hold_pitch2_vel = ClampValue(
            pitch2_angle_pid.UpDate(init_pitch2_angle, current_pitch2_angle),
            -kPitch2VelocityLimit,
            kPitch2VelocityLimit);

        // Yaw 用 ADRC 锁姿态，达妙底层关闭 Kd，纯扭矩输出。
        out.yaw_vel = 0.0f;
        out.yaw_torq = ClampValue(
            yaw_adrc.LADRC_1(hold_yaw_vel, current_yaw_vel),
            -kYawTorqueLimit,
            kYawTorqueLimit);

        // Pitch2 直接下发目标速度，依靠重力前馈 + 达妙硬件 Kd 实现高刚度位置锁死。
        out.pitch2_vel = LimitPitch2VelocityByAngle(current_pitch2_angle, hold_pitch2_vel);
        out.pitch2_torq = ClampValue(
            pitch2_gravity_torque,
            -kPitch2TorqueLimit,
            kPitch2TorqueLimit);
        return;
    }

    const float yaw_torque_cmd = yaw_adrc.LADRC_1(cmd.yaw_vel, current_yaw_vel);
    out.yaw_vel = 0.0f;
    out.yaw_torq = ClampValue(yaw_torque_cmd, -kYawTorqueLimit, kYawTorqueLimit);

    const float safe_pitch2_vel = LimitPitch2VelocityByAngle(
        current_pitch2_angle,
        cmd.pitch2_vel);
    const float pitch2_torque_cmd =
        pitch2_adrc.LADRC_1(safe_pitch2_vel, current_pitch2_vel) + pitch2_gravity_torque;

    out.pitch2_vel = 0.0f;
    out.pitch2_torq = ClampValue(
        pitch2_torque_cmd,
        -kPitch2TorqueLimit,
        kPitch2TorqueLimit);
}

// 角度控制器：STM32 做 Pitch2 角度外环，只把目标速度和重力前馈交给达妙。
static void angle_Control()
{
    UpdateStandupState();

    const float current_yaw_angle = GetYawAngle();
    const float current_pitch2_angle = GetPitch2Angle();
    const float pitch2_gravity_torque = GetPitch2GravityTorque();

    UpdatePitch1Control();

    if (!is_standup_complete)
    {
        const float yaw_error = WrapAngleRad(init_yaw_angle - current_yaw_angle);
        const float virtual_yaw_angle = current_yaw_angle + yaw_error;
        const float hold_yaw_vel = ClampValue(
            yaw_transform_angle_pid.UpDate(virtual_yaw_angle, current_yaw_angle),
            -kYawVelocityLimit,
            kYawVelocityLimit);
        const float hold_pitch2_vel = ClampValue(
            pitch2_angle_pid.UpDate(init_pitch2_angle, current_pitch2_angle),
            -kPitch2VelocityLimit,
            kPitch2VelocityLimit);

        // 起立阶段：角度模式仍利用达妙底层速度闭环，Pitch2 只叠加重力前馈。
        out.yaw_vel = hold_yaw_vel;
        out.yaw_torq = 0.0f;
        out.pitch2_vel = LimitPitch2VelocityByAngle(current_pitch2_angle, hold_pitch2_vel);
        out.pitch2_torq = ClampValue(
            pitch2_gravity_torque,
            -kPitch2TorqueLimit,
            kPitch2TorqueLimit);
        return;
    }

    const float yaw_error = WrapAngleRad(cmd.yaw_angle - current_yaw_angle);
    const float virtual_yaw_angle = current_yaw_angle + yaw_error;

    const float yaw_velocity_cmd = ClampValue(
        yaw_transform_angle_pid.UpDate(virtual_yaw_angle, current_yaw_angle),
        -kYawVelocityLimit,
        kYawVelocityLimit);
    const float pitch2_velocity_cmd = ClampValue(
        pitch2_angle_pid.UpDate(cmd.pitch2_angle, current_pitch2_angle),
        -kPitch2VelocityLimit,
        kPitch2VelocityLimit);

    out.yaw_vel = yaw_velocity_cmd;
    out.yaw_torq = 0.0f;

    // 角度模式：Pitch2_Angle_pid 输出 V_des，底层速度闭环由达妙 Kd 完成。
    out.pitch2_vel = LimitPitch2VelocityByAngle(current_pitch2_angle, pitch2_velocity_cmd);
    out.pitch2_torq = ClampValue(
        pitch2_gravity_torque,
        -kPitch2TorqueLimit,
        kPitch2TorqueLimit);
}

/* 各状态执行 --------------------------------------------------------------------------------------------*/
// STOP 状态：清零输出并复位控制器，保证电机下发为安全值。
static void gimbal_stop()
{
    yaw_adrc.Reset(0.0f, 0.0f);
    pitch2_adrc.Reset(0.0f, 0.0f);
    yaw_transform_angle_pid.reset();
    yaw_encoder_transform_pid.reset();
    pitch2_angle_pid.reset();

    out.yaw_vel = 0.0f;
    out.yaw_torq = 0.0f;
    out.pitch1_vel = 0.0f;
    out.pitch1_torq = 0.0f;
    out.pitch2_vel = 0.0f;
    out.pitch2_torq = 0.0f;
}

// MANUAL/KEYBOARD 状态：走速度模式控制。
static void gimbal_manual()
{
    velocity_Control();
}

// VISION 状态：走角度模式控制，外环输出目标速度。
static void gimbal_vision()
{
    angle_Control();
}


// TRANSFORM 状态：执行变形控制逻辑（分步走：Yaw回正 -> Pitch1平滑下降）
static void gimbal_transform()
{
    cmd.current_mode = GIMBAL_MODE_ANGLE;

    const float current_yaw_angle = GetYawAngle();

    // 获取 Yaw 轴电机的当前绝对编码器弧度反馈
    const float current_yaw_motor_rad = MotorJ4310.getAngleRad(2);

    static uint32_t last_transform_enter_count = 0;
    const uint32_t current_transform_enter_count = gimbal_fsm.Get_State_Enter_Count(TRANSFORM);
    static bool yaw_aligned_latch = false;
    static float locked_yaw_rad = 0.0f;

    if (current_transform_enter_count != last_transform_enter_count)
    {
        yaw_aligned_latch = false;
        last_transform_enter_count = current_transform_enter_count;

        // 状态复位
        yaw_transform_planner.Reset(current_yaw_motor_rad);
        yaw_encoder_transform_pid.reset();

        // 计算 Yaw 轴就近最短路径目标
        const float initial_error = WrapAngleRad(kTransformYawTargetEncoderRad - current_yaw_motor_rad);
        locked_yaw_rad = current_yaw_motor_rad + initial_error;

        // 切入变形模式瞬间清空 Pitch2 角度环历史，避免退出时积分突跳。
        pitch2_angle_pid.reset();
    }

    // 后台目标覆盖，防积分风暴
    cmd.yaw_angle = current_yaw_angle;
    cmd.pitch2_angle = 0.0f;

    if (!yaw_aligned_latch)
    {
        cmd.pitch1 = kBigPitchTargetRad;
        if (std::fabs(WrapAngleRad(kTransformYawTargetEncoderRad - current_yaw_motor_rad)) < 0.03f)
        {
            yaw_aligned_latch = true;
        }
    }
    else
    {
        cmd.pitch1 = 5.0f * kDegToRad;
    }

    // --- 执行层闭环 ---
    
    // 1. Yaw 轴控制（带动态过零防跳变）
    const float planned_yaw_motor_rad = yaw_transform_planner.Update(locked_yaw_rad);
    const float dynamic_error = WrapAngleRad(planned_yaw_motor_rad - current_yaw_motor_rad);
    const float virtual_yaw_motor_rad = current_yaw_motor_rad + dynamic_error;

    const float yaw_velocity_cmd = ClampValue(
        yaw_encoder_transform_pid.UpDate(virtual_yaw_motor_rad, current_yaw_motor_rad),
        -kYawVelocityLimit,
        kYawVelocityLimit
    );
    out.yaw_vel = yaw_velocity_cmd;
    out.yaw_torq = 0.0f;

    // 2. Pitch1 轴控制（单轴 VMC 扭矩控制）
    UpdatePitch1Control();

    // 3. Pitch2 轴控制：复合 VMC（增稳 + 防撞墙 + 触底卸力联动）。
    const float current_pitch2_imu = GetPitch2Angle();
    const float current_pitch2_vel_imu = GetPitch2Vel();
    const float current_pitch2_vel_motor = MotorJ4310.getVelocityRads(1);
    const float pitch2_relative_angle_deg = MotorJ4310.getAngleDeg(1);

    float level_kp = GimbalConfig::kTransformPitch2LevelKp;
    float level_kd = GimbalConfig::kTransformPitch2LevelKd;
    float wall_stiffness = GimbalConfig::kTransformPitch2WallStiffness;
    float final_pitch2_gravity = GetPitch2GravityTorque();
    float damping_velocity = current_pitch2_vel_imu;

    static bool pitch2_is_landing = false;
    const float pitch1_angle_deg = GetPitch1AngleDeg();

    if (pitch1_angle_deg < GimbalConfig::kPitch2HysteresisLowDeg)
    {
        pitch2_is_landing = true;
    }
    else if (pitch1_angle_deg > GimbalConfig::kPitch2HysteresisHighDeg)
    {
        pitch2_is_landing = false;
    }

    if (pitch2_is_landing)
    {
        level_kp = 0.0f;
        level_kd = GimbalConfig::kPitch2LandingDamper;
        damping_velocity = current_pitch2_vel_motor;
        wall_stiffness = 0.0f;
        final_pitch2_gravity *= 0.0f;
    }

    const float target_imu_rad = 0.0f;
    const float level_torque = level_kp * (target_imu_rad - current_pitch2_imu) -
                               level_kd * damping_velocity;

    float wall_torque = 0.0f;
    if (wall_stiffness > 0.0f && pitch2_relative_angle_deg < GimbalConfig::kTransformPitch2VirtualWallAngle)
    {
        const float penetration_rad =
            (GimbalConfig::kTransformPitch2VirtualWallAngle - pitch2_relative_angle_deg) * kDegToRad;
        wall_torque = wall_stiffness * penetration_rad;
    }

    pitch2_angle_pid.reset();
    pitch2_adrc.Reset(damping_velocity, 0.0f);

    out.pitch2_vel = 0.0f;
    out.pitch2_torq = ClampValue(
        level_torque + wall_torque + final_pitch2_gravity,
        -kPitch2TorqueLimit,
        kPitch2TorqueLimit);
}

/* 主循环与任务入口 --------------------------------------------------------------------------------------*/
// 在线检测：遥控、IMU、三路云台电机都在线时才允许云台进入工作状态。
static bool check_online()
{
    const bool remote_online = DT7.isConnected();
    const bool imu_online    = HI12.isConnected();
    const bool pitch2_online = MotorJ4310.isConnected(1);
    const bool yaw_online    = MotorJ4310.isConnected(2);
    const bool pitch1_online = MotorJ4340.isConnected(1);

    // ================= 插入蜂鸣器报警请求 =================
    // 获取蜂鸣器单例
    auto& buzzer = BSP::WATCH_STATE::BuzzerManagerSimple::getInstance();

    if (!remote_online) buzzer.requestRemoteRing();
    if (!imu_online)    buzzer.requestIMURing();
    
    // 给不同的云台电机分配不同的响铃 ID，方便听声辨位
    if (!pitch1_online) buzzer.requestMotorRing(1); // Pitch 1 掉线：滴 1 下
    if (!yaw_online)    buzzer.requestMotorRing(2); // Yaw 掉线：滴 2 下
    if (!pitch2_online) buzzer.requestMotorRing(3); // Pitch 2 掉线：滴 3 下
    // ======================================================

    return remote_online && imu_online && pitch2_online && yaw_online && pitch1_online;
}

// 状态机与目标初始化：上电时将角度/速度目标同步到当前反馈。
static void fsm_init()
{
    gimbal_fsm.Init();
    DT7.SetDeadzone(20.0f);

    cmd.current_mode = GIMBAL_MODE_VELOCITY;
    cmd.yaw_vel = 0.0f;
    cmd.pitch2_vel = 0.0f;
    cmd.yaw_angle = GetYawAngle();
    cmd.pitch2_angle = ClampPitch2AngleTarget(GetPitch2Angle());
    ResetPitch1HoldTarget();
}



// 云台主循环：完美结合 FSM 状态机与“单拨杆无伤热切”的终极版本
static void main_loop_gimbal(uint8_t left_switch, uint8_t right_switch, bool equipment_online)
{
    // 【视觉模式】双拨杆居中 (左3 且 右3) 时触发自瞄
    const bool req_vision = (left_switch == 3 && right_switch == 3);

    // 【变形模式】双上触发 (左1 且 右1)
    // 隐式互锁：底盘板接收同一路 DBUS，双上时底盘天然切入 NOTFOLLOW，云台端直接信任该物理信号。
    const bool req_transform = (left_switch == 1 && right_switch == 1);

    // 将物理拨杆和解析好的安全 Flag 喂给健壮的状态机
    gimbal_fsm.StateUpdate(left_switch, right_switch, equipment_online, req_vision, req_transform);
    gimbal_fsm.TIM_Update();

    // 目标调度与起立保护
    SetTarget_Gimbal();

    // 根据状态机分配执行层
    switch (gimbal_fsm.Get_Now_State())
    {
    case STOP:
        gimbal_stop();
        break;
    case MANUAL:
    case KEYBOARD: // 如果左3右3，会进键盘模式，但执行层和 MANUAL 共享，手感一致
        gimbal_manual(); 
        break;
    case VISION:
        gimbal_vision(); 
        break;
    case TRANSFORM:
        gimbal_transform();
        break;
    default:
        gimbal_stop();
        break;
    }
}

extern "C"
{
// FreeRTOS 控制任务入口，每 1 ms 执行一次云台控制链路。
void Control(void const * argument)
{
    
    (void)argument;
    fsm_init();
// 1. 各类外设、总线初始化...
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    
    // 2. 阻塞式播放 2 秒开机音效（控制逻辑在此处暂停，等待硬件准备就绪）
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().playStartupMusic();
    for (;;)
    {
       
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());
         // 4. 高频调用非阻塞 update，自动推进所有的报警音符
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        osDelay(1);
    }
}
}
