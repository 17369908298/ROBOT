#include "MotorTask.hpp"

namespace
{
constexpr uint8_t kDmMotorIndex = 1U;

bool ShouldEnableGimbalMotor()
{
    return gimbal_fsm.Get_Now_State() != STOP;
}

/* DM motor state machine --------------------------------------------------------------------------------*/
template <typename MotorType>
static void HandleDmMotorStateMachine(
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

void MotorControlLogic()
{
    static uint8_t j6006_enable_step = 0U;

    HandleDmMotorStateMachine(MotorJ6006_Yaw,
                              kDmMotorIndex,
                              j6006_enable_step,
                              ShouldEnableGimbalMotor(),
                              0.0f,
                               out.yaw_torq,
                              0.0f,
                              1.0f,
                              0.0f);
}
} // namespace

BSP::Motor::DM::J6006<1> MotorJ6006_Yaw(0x00, {1}, {0x02});

void MotorInit(void)
{
    static bool initialized = false;
    if (initialized)
    {
        return;
    }

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        MotorJ6006_Yaw.Parse(frame);
    });

    initialized = true;
}



void Motor(void const *argument)
{
    (void)argument;
    MotorInit();

    for (;;)
    {
        MotorControlLogic();
        osDelay(1);
    }
}

