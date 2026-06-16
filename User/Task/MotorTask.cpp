#include "MotorTask.hpp"

// Gravity tuning switch: 1 = feedforward only, 0 = normal closed-loop operation.
#define TUNE_GRAVITY_MODE 0

// Pitch2 gravity tuning switch: 1 = Pitch2 gravity feedforward only.
#define TUNE_PITCH2_GRAVITY_ONLY 0

namespace
{
static float ClampMotorValue(float value, float min_value, float max_value)
{
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
}

BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {2, 6}, {0x01, 0x05});  // 1: Pitch2, 2: Yaw
BSP::Motor::DM::J4340<1> MotorJ4340(0x00, {4}, {0x03});            // 1: Pitch1

/* CAN receive -------------------------------------------------------------------------------------------*/
void MotorInit(void)
{
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        MotorJ4310.Parse(frame);
        MotorJ4340.Parse(frame);
    });
}

/* DM motor state machine --------------------------------------------------------------------------------*/
template <typename MotorType>
static void handle_dm_motor_state_machine(
    MotorType& motor,
    uint8_t id,
    uint8_t &step,
    bool target_enable,
    float pos,
    float vel,
    float kp,
    float kd,
    float torq)
{
    // 获取硬件的绝对真实状态：0=未使能, 1=已使能无错, >1=硬件报错
    uint8_t err = motor.getError(id);

    if (target_enable)
    {
        // ✨ 核心修复：直接读取硬件真实状态！
        // 只要硬件处于使能无错(1)，立刻接管，无视软件标志位的初始值！
        if (err == 1)
        {
            motor.setIsenable(id, true); // 强制同步软件标志位
            step = 0;
            motor.ctrl_Mit(id, pos, vel, kp, kd, torq);
            return;
        }

        // --- 进入异常恢复与使能状态机 ---
        if (step == 0)
        {
            if (err > 1) { // 如果硬件报错
                motor.ClearErr(id, BSP::Motor::DM::MIT);
                step = 1; // 步进到 1，去等待清错成功
            } else if (err == 0) { // 如果硬件处于安全失能状态
                motor.On(id, BSP::Motor::DM::MIT);
                step = 51; // 步进到 51，去等待使能成功
            }
        }
        // 状态 1~50：等待 ClearErr 生效 (约 150ms 超时防卡死)
        else if (step >= 1 && step <= 50)
        {
            if (err == 0) { // 清错成功，状态变为了未使能(0)
                motor.On(id, BSP::Motor::DM::MIT);
                step = 51; // 转去等待使能
            } else if (err == 1) { // 居然意外使能成功了
                step = 0; // 下一周期直接接管
            } else {
                step++; // 继续等
                if (step > 50) step = 0; // 超时！从头再发一次 ClearErr
            }
        }
        // 状态 51~100：等待 On 生效 (约 150ms 超时防卡死)
        else if (step >= 51 && step <= 100)
        {
            if (err == 1) { // 使能成功(1)
                step = 0; // 完美！下个周期直接接管
            } else if (err > 1) { // 尝试使能的瞬间瞬间过流报错了
                step = 0; // 回退到 0，重新去发 ClearErr
            } else {
                step++; // 继续等
                if (step > 100) step = 0; // 超时！从头发一次 On
            }
        }
        else
        {
            step = 0; // 容错保护
        }
    }
    else // 断电分支逻辑
    {
        if (err == 0)
        {
            motor.setIsenable(id, false);
            step = 0;
            motor.ctrl_Mit(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            return;
        }

        if (step == 0)
        {
            motor.Off(id, BSP::Motor::DM::MIT);
            step = 1;
        }
        else if (step >= 1 && step <= 50)
        {
            if (err == 0) {
                step = 0;
            } else {
                step++;
                if (step > 50) step = 0; // 超时重试 Off
            }
        }
        else
        {
            step = 0;
        }
    }
}

/* Motor control -----------------------------------------------------------------------------------------*/
static void motor_control()
{
    static uint8_t send_seq = 0;
    send_seq++;

    static uint8_t re_enable_step_pitch2 = 0;
    static uint8_t re_enable_step_yaw = 0;
    static uint8_t re_enable_step_pitch1 = 0;

    const bool is_gimbal_active = (gimbal_fsm.Get_Now_State() != STOP);
    const bool is_transform = (gimbal_fsm.Get_Now_State() == TRANSFORM);

    if (send_seq % 3 == 0) // Pitch2
    {
        const bool pitch2_angle_mode = (cmd.current_mode == GIMBAL_MODE_ANGLE);
        const float pitch2_vel_cmd = (TUNE_GRAVITY_MODE || TUNE_PITCH2_GRAVITY_ONLY) ? 0.0f :
            ClampMotorValue(out.pitch2_vel, -8.0f, 8.0f);

        // TRANSFORM 模式下 Pitch2 软件侧已经做阻尼/虚拟墙，底层必须关闭 Kd，避免双重阻尼。
        const float pitch2_kd = (TUNE_GRAVITY_MODE || TUNE_PITCH2_GRAVITY_ONLY || is_transform) ? 0.0f :
            ((pitch2_angle_mode || !is_standup_complete) ? 0.9f : 0.0f);

        const float pitch2_torq_cmd =
            ClampMotorValue(out.pitch2_torq, -3.0f, 3.0f);

        handle_dm_motor_state_machine(MotorJ4310, 1, re_enable_step_pitch2, is_gimbal_active,
                                      0.0f, pitch2_vel_cmd, 0.0f, pitch2_kd, pitch2_torq_cmd);
    }
    else if (send_seq % 3 == 1) // Yaw
    {
        const bool yaw_angle_mode = (cmd.current_mode == GIMBAL_MODE_ANGLE);
        const float yaw_vel_cmd = yaw_angle_mode ?
            ClampMotorValue(out.yaw_vel, -7.0f, 7.0f) : 0.0f;
        const float yaw_kd = yaw_angle_mode ? 0.9f : 0.0f;
        const float yaw_torq_cmd = ClampMotorValue(out.yaw_torq, -7.0f, 7.0f);

        handle_dm_motor_state_machine(MotorJ4310,
                                      2,
                                      re_enable_step_yaw,
                                      is_gimbal_active,
                                      0.0f,
                                      yaw_vel_cmd,
                                      0.0f,
                                      yaw_kd,
                                      yaw_torq_cmd);
    }
    else // Pitch1
    {
        // Pitch1 已由 STM32 VMC 生成完整扭矩，达妙底层降维为纯扭矩执行器。
        const float pitch1_vel_cmd = 0.0f;
        const float pitch1_kd = 0.0f;
        const float pitch1_torq_cmd =
            ClampMotorValue(out.pitch1_torq, -6.0f, 6.0f);

        handle_dm_motor_state_machine(MotorJ4340, 1, re_enable_step_pitch1, is_gimbal_active,
                                      0.0f, pitch1_vel_cmd, 0.0f, pitch1_kd, pitch1_torq_cmd);
    }
}

/* Task entry --------------------------------------------------------------------------------------------*/
extern "C"
{
void Motor(void const * argument)
{
    (void)argument;
    MotorInit();

    for (;;)
    {
        motor_control();
        osDelay(1);
    }
}
}
