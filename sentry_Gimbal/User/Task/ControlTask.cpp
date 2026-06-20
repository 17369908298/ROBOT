#include "ControlTask.hpp"
#include "MotorTask.hpp"
#include "SerialTask.hpp"
#include "../core/BSP/Common/StateWatch/buzzer_manager.hpp"

#include <cmath>

namespace
{
constexpr float kYawVelocityLimit = 2.0f;
constexpr float kYawTorqueLimit = 7.0f;
constexpr float kYawManualStickScale = 1.0f;
constexpr float kYawAngleStickRate = 1.0f;

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

float WrapAngleRad(float target_angle, float current_angle)
{
    // 1. 计算两者的绝对误差
    float error = target_angle - current_angle;
    
    // 2. 将误差规范化到 [-π, π] 范围内（寻找最近的最短路径）
    if (!std::isfinite(error)) return current_angle;
    while (error > kPi) error -= 2.0f * kPi;
    while (error < -kPi) error += 2.0f * kPi;
    
    // 3. 将最短路径误差加回当前实际角度，得到等效的连续目标值
    return current_angle + error;
}


float GetYawAngle()
{
    static float last_yaw_angle = 0.0f;
    const float yaw_angle = -HI12.GetAngle(2);

    if (std::isfinite(yaw_angle))
    {
        last_yaw_angle = yaw_angle * kDegToRad;
    }

    return last_yaw_angle;
}

float GetYawVel()
{
    static float last_yaw_vel = 0.0f;
    const float yaw_vel = -HI12.GetGyro(2);

    if (std::isfinite(yaw_vel))
    {
        last_yaw_vel = yaw_vel * kDegToRad;
    }

    return last_yaw_vel;
}

void SyncYawHoldTarget()
{
    cmd.yaw_angle = GetYawAngle();
}

bool CheckOnline()
{
    const bool remote_online = DT7.isConnected();
    const bool imu_online = HI12.isConnected();
    const bool yaw_online = MotorJ6006_Yaw.isConnected(1, 2);

    return remote_online && imu_online && yaw_online;
}

void FsmInit()
{
    gimbal_fsm.Init();
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    DT7.SetDeadzone(20.0f);
    cmd.current_mode = GIMBAL_MODE_VELOCITY;
    cmd.yaw_vel = 0.0f;
    SyncYawHoldTarget();
}

TDFilter yaw_manual_filter(50.0f, kControlPeriodS);
Enum_Gimbal_States prev_gimbal_state = STOP;
bool prev_equipment_online = false;
} // namespace

Gimbal_FSM gimbal_fsm;
GimbalTarget_t cmd = {GIMBAL_MODE_VELOCITY, 0.0f, 0.0f};
GimbalOutput_t out = {0.0f, 0.0f};

ALG::PID::PID yaw_vel_pd(20.0f, 0.0f, 0.0f, kYawTorqueLimit, 0.0f, 0.0f);
ALG::PID::PID yaw_angle_pid(1.0f, 0.0f, 0.0f, kYawVelocityLimit, 100.0f, 0.0f);

void Yaw_SetTarget()
{
    const Enum_Gimbal_States now_state = gimbal_fsm.Get_Now_State();

    // 状态切换时同步一次角度目标，避免每个周期重置
    if (now_state != prev_gimbal_state)
    {
        SyncYawHoldTarget();
        prev_gimbal_state = now_state;
    }

    switch (now_state)
    {
    case STOP:
        cmd.current_mode = GIMBAL_MODE_VELOCITY;
        cmd.yaw_vel = 0.0f;
        break;

  case MANUAL:
case VISION:
{
    cmd.current_mode = GIMBAL_MODE_ANGLE;
    // 1. 先算出摇杆期望的“目标角速度”
    float stick_target_vel = yaw_manual_filter.filter(DT7.get_right_x() * kYawManualStickScale);
    stick_target_vel = ClampValue(stick_target_vel, -kYawVelocityLimit, kYawVelocityLimit);
    
    // 2. 将速度乘以时间周期 (dt = 0.001s)，转化为“期望的角度增量”
    cmd.yaw_angle += stick_target_vel * kControlPeriodS; 
    break;
}
    }
}

void Yaw_Control()
{
    const float current_yaw_vel = GetYawVel();
    const float current_yaw_angle = GetYawAngle();
    float target_vel = cmd.yaw_vel;

    if (cmd.current_mode == GIMBAL_MODE_ANGLE)
    {
        
        const float yaw_tar = WrapAngleRad(cmd.yaw_angle ,current_yaw_angle);
        out.yaw_torq  = ClampValue(
            yaw_angle_pid.UpDate(yaw_tar,current_yaw_angle),
            -kYawVelocityLimit,
            kYawVelocityLimit);
    }

}

void GimbalStop()
{
    cmd.current_mode = GIMBAL_MODE_VELOCITY;
    cmd.yaw_vel = 0.0f;
    SyncYawHoldTarget();
    out.yaw_vel = 0.0f;
    out.yaw_torq = 0.0f;
    yaw_angle_pid.reset();
    yaw_vel_pd.reset();
}

void MainLoopGimbal(uint8_t left_switch, uint8_t right_switch, bool equipment_online)
{
    auto& buzzer = BSP::WATCH_STATE::BuzzerManagerSimple::getInstance();

    // 掉线检测：从在线切换到离线时触发蜂鸣器
    if (prev_equipment_online && !equipment_online)
    {
        if (!DT7.isConnected())
            buzzer.requestRemoteRing();
        if (!HI12.isConnected())
            buzzer.requestIMURing();
        if (!MotorJ6006_Yaw.isConnected(1, 2))
            buzzer.requestMotorRing(1);
    }
    prev_equipment_online = equipment_online;

    // 每周期消费蜂鸣器队列
    buzzer.update();

    const bool req_vision =
        (left_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE) &&
        (right_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE);

    gimbal_fsm.StateUpdate(left_switch, right_switch, equipment_online, req_vision);
    gimbal_fsm.TIM_Update();

    Yaw_SetTarget();

    switch (gimbal_fsm.Get_Now_State())
    {
    case STOP:
        GimbalStop();
        break;

    case MANUAL:
    case VISION:
        Yaw_Control();
        break;

    default:
        GimbalStop();
        break;
    }
}

extern "C"
{
void Control(void const *argument)
{
    (void)argument;
    FsmInit();

    for (;;)
    {
        MainLoopGimbal(DT7.get_s1(), DT7.get_s2(), CheckOnline());
        osDelay(1);
    }
}
}
