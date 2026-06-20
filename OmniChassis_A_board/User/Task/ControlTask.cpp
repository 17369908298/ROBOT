#include "ControlTask.hpp"
#include <cmath>

extern bool alphabet[28];

namespace
{
constexpr int kAxisX = 0;
constexpr int kAxisY = 1;
constexpr int kAxisW = 2;

float limit_float(float value, float min_value, float max_value)
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

bool is_remote_stop()
{
    return DT7.get_s1() == ChassisConfig::kSwitchDown && DT7.get_s2() == ChassisConfig::kSwitchDown;
}

float get_scroll_spin_target()
{
    return DT7.get_scroll_() * ChassisConfig::kSpinSensitivity;
}

/**
 * @brief 更新底盘正运动学反馈。
 *
 * 每个控制周期先把 4 个 3508 的实际轮速喂给 omni_fk，
 * 保证 GetChassisVx/Vy/Vw 返回的是当前周期的真实反馈值。
 */
void update_chassis_kinematics()
{
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1),
                              Motor3508.getVelocityRads(2),
                              Motor3508.getVelocityRads(3),
                              Motor3508.getVelocityRads(4));
}

void clear_chassis_target()
{
    chassis_target.target_translation_x = 0.0f;
    chassis_target.target_translation_y = 0.0f;
    chassis_target.target_rotation = 0.0f;
}

void calculate_remote_translation_target()
{
    float remote_x = -DT7.get_left_x() * ChassisConfig::kMaxSpeed;
    float remote_y = -DT7.get_left_y() * ChassisConfig::kMaxSpeed;
    Chassis_Velocity_Normalize(remote_x, remote_y);

    // remote_x/remote_y 为遥控器摇杆输入，CalculateTranslation_xy 内部会完成云台坐标系到车体坐标系的旋转投影。
    CalculateTranslation_xy(Cboard.GetYawAngle(), remote_x, remote_y, 0.0f,
                            &chassis_target.target_x, &chassis_target.target_y, 0.0f);
}

void update_translation_slope()
{
    omni_target[kAxisY].TIM_Calculate_PeriodElapsedCallback(chassis_target.target_y, omni_fk.GetChassisVy());
    omni_target[kAxisX].TIM_Calculate_PeriodElapsedCallback(chassis_target.target_x, omni_fk.GetChassisVx());

    chassis_target.target_translation_x = omni_target[kAxisX].GetOut();
    chassis_target.target_translation_y = omni_target[kAxisY].GetOut();
}

void apply_omni_wheel_control()
{
    omni_ik.OmniInvKinematics(chassis_target.target_translation_x,
                              chassis_target.target_translation_y,
                              chassis_target.target_rotation,
                              0, 1.0f, 1.0f);

    for (int i = 0; i < ChassisConfig::kWheelCount; i++)
    {
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i + 1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = limit_float(chassis_output.out_wheel[i],
                                                  -ChassisConfig::kPidOutputLimit,
                                                  ChassisConfig::kPidOutputLimit);
    }
}
} // namespace

Chassis_FSM chassis_fsm;

float wheel_azimuth[ChassisConfig::kWheelCount] = {
    static_cast<float>(-ChassisConfig::kPi / 4.0),
    static_cast<float>(-3.0 * ChassisConfig::kPi / 4.0),
    static_cast<float>(3.0 * ChassisConfig::kPi / 4.0),
    static_cast<float>(ChassisConfig::kPi / 4.0),
};

float wheel_direction[ChassisConfig::kWheelCount] = {
    static_cast<float>(-3.0 * ChassisConfig::kPi / 4.0),
    static_cast<float>(3.0 * ChassisConfig::kPi / 4.0),
    static_cast<float>(ChassisConfig::kPi / 4.0),
    static_cast<float>(-ChassisConfig::kPi / 4.0),
};

Alg::CalculationBase::Omni_IK omni_ik(ChassisConfig::kChassisRadius, ChassisConfig::kWheelRadius,
                                      wheel_azimuth, wheel_direction);
Alg::CalculationBase::Omni_FK omni_fk(ChassisConfig::kChassisRadius, ChassisConfig::kWheelRadius,
                                      ChassisConfig::kWheelNumber, wheel_azimuth, wheel_direction);

ALG::PID::PID wheel_pid[ChassisConfig::kWheelCount] = {
    ALG::PID::PID(400.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f), // 轮向速度 PID 1 号轮
    ALG::PID::PID(400.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f), // 轮向速度 PID 2 号轮
    ALG::PID::PID(400.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f), // 轮向速度 PID 3 号轮
    ALG::PID::PID(400.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f), // 轮向速度 PID 4 号轮
};

ALG::PID::PID follow_pid(ChassisConfig::kFollowPidKp, ChassisConfig::kFollowPidKi, ChassisConfig::kFollowPidKd,
                         ChassisConfig::kPidOutputLimit, ChassisConfig::kPidIntegralLimit,
                         ChassisConfig::kPidIntegralSeparationThreshold); // 底盘跟随 PID

Alg::Utility::SlopePlanning omni_target[ChassisConfig::kAxisCount] = {
    Alg::Utility::SlopePlanning(ChassisConfig::kSlopeIncrease, ChassisConfig::kSlopeDecrease), // X
    Alg::Utility::SlopePlanning(ChassisConfig::kSlopeIncrease, ChassisConfig::kSlopeDecrease), // Y
    Alg::Utility::SlopePlanning(ChassisConfig::kSlopeIncrease, ChassisConfig::kSlopeDecrease), // W
};

ChassisContext chassis_context;
ControlTask &chassis_target = chassis_context.target;
Output_chassis &chassis_output = chassis_context.output;

bool check_online()
{
    bool isconnected = true;
    for (int i = 0; i < ChassisConfig::kWheelCount; i++)
    {
        if (!Motor3508.isConnected(i + 1, i + 1))
        {
            isconnected = false;
        }
    }

    if (/*!Cboard.isConnected() ||*/ !DT7.isConnected())
    {
        isconnected = false;
    }

    if (!isconnected)
    {
        return false;
    }

    return true;
}

/**
 * @brief 模块 1：底盘平移速度归一化 (防止对角线方向超速)
 * @param vx_target X轴目标速度 (传入引用，内部直接修改原变量)
 * @param vy_target Y轴目标速度 (传入引用，内部直接修改原变量)
 */
void Chassis_Velocity_Normalize(float &vx_target, float &vy_target)
{
    const float planar_limit = std::fmaxf(std::fabsf(vx_target), std::fabsf(vy_target));
    const float planar_speed = std::sqrtf(vx_target * vx_target + vy_target * vy_target);

    // 如果合速度大于单一轴的最大限制，则进行等比例缩放
    if (planar_limit > 0.0f && planar_speed > planar_limit)
    {
        const float scale = planar_limit / planar_speed;
        vx_target *= scale;
        vy_target *= scale;
    }
}

void fsm_init()
{
    chassis_fsm.Init();
}

/**
 * @brief 底盘相对云台坐标系的平移解算（包含自适应动态速度闭环补偿）
 * @param theta 云台当前反馈的 Yaw 轴角度（单位：弧度）
 * @param vx 笛卡尔坐标系下的 X 方向输入（云台系左右）
 * @param vy 笛卡尔坐标系下的 Y 方向输入（云台系前后）
 * @param phi 角度偏差补偿值
 * @param out_vx [输出指针] 计算补偿与旋转矩阵后，底盘坐标系下的 X 方向速度
 * @param out_vy [输出指针] 计算补偿与旋转矩阵后，底盘坐标系下的 Y 方向速度
 * @param psi 额外的相位角补偿 (保留小幅度纯相位补偿以抵消 CAN 总线固定时滞)
 */
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi)
{
    // === 1. 基础坐标系旋转变换 ===
    // 将云台反馈角减去正前方的零位基准角，再叠加额外的补偿角
    theta = theta - ChassisConfig::kYawStandard + phi;

    // 过零处理 (将角度归一化限制在 -PI 到 PI 之间)
    theta = fmod(theta, 2 * ChassisConfig::kPi);
    if (theta > ChassisConfig::kPi) theta -= 2 * ChassisConfig::kPi;
    else if (theta < -ChassisConfig::kPi) theta += 2 * ChassisConfig::kPi;

    // 预计算正弦和余弦值，包含额外的 psi 相位补偿
    float s = sinf(theta + psi);
    float c = cosf(theta + psi);

    // 遥控器控制量输入 (云台坐标系下)
    float raw_vx = ChassisConfig::kRemoteToChassisGain * vy;  // 云台系前后
    float raw_vy = -ChassisConfig::kRemoteToChassisGain * vx; // 云台系横移

    // 目标底盘速度（未补偿的理论投影）
    float target_chassis_vx = raw_vx * c - raw_vy * s;
    float target_chassis_vy = raw_vx * s + raw_vy * c;

    // === 2. 小陀螺动态速度闭环补偿 ===
    // 获取正解算算出的底盘实际物理速度 (底盘系) 与实际角速度
    float actual_vx = omni_fk.GetChassisVx();
    float actual_vy = omni_fk.GetChassisVy();
    float actual_wz = omni_fk.GetChassisVw();

    // 动态补偿系数 Kp = K_base * |w_actual|
   
    constexpr float k_base = 0.02f; 
    float kp = k_base * std::fabsf(actual_wz);

    // 利用实际速度与目标速度的差值，进行反向叠加补偿
    // 如果稳态时实际速度跟上了目标速度，差值为0，补偿自动消退，完美避免反向歪斜
    *out_vx = target_chassis_vx + kp * (target_chassis_vx - actual_vx);
    *out_vy = target_chassis_vy + kp * (target_chassis_vy - actual_vy);
}

/**
 * @brief 底盘跟随云台的角速度(w)规划。
 *
 * 计算底盘偏航角(Yaw)与云台相对基准角度(STANDARD)之间的误差，
 * 并通过 PID 算法输出一个用于底盘旋转的角速度，使底盘始终朝向云台的正面。
 */
void CalculateFollow()
{
    // 计算跟随误差：标准基准角度(云台相对底盘的正前方编码器值) - 当前云台的实际反馈偏航角
    float follow_error = ChassisConfig::kYawStandard - Cboard.GetYawAngle();

    // 角度归一化处理 (将误差限制在 -PI/2 到 PI/2，即 -90度 到 +90度之间)
    // 注意：这里的 1.5707963267f 就是 PI / 2
    // 如果误差超过 90 度，代码将其反转，这通常意味着底盘可以通过反向转动更近地到达目标，
    // 或者机械结构上允许底盘正反都能算作“跟随”（比如轮式步兵底盘前后对称）。
    while (follow_error > ChassisConfig::kHalfPi) follow_error -= 2 * ChassisConfig::kHalfPi;
    while (follow_error < -ChassisConfig::kHalfPi) follow_error += 2 * ChassisConfig::kHalfPi;

    // 死区处理 (防止极小误差引起的底盘微小高频震荡，通常称为“点头”或“发抖”)
    if (fabs(follow_error) < ChassisConfig::kFollowDeadband) follow_error = 0.0f;

    // 将处理后的误差送入跟随 PID 控制器计算
    // 目标值为 0.0f（即期望误差为0），当前值为算出的误差量 follow_error
    follow_pid.UpDate(0.0f, follow_error);
}

/**
 * @brief 底盘不跟随 (NOTFOLLOW) 目标规划。
 */
void Notfollow_SetTarget()
{
    if (is_remote_stop())
    {
        clear_chassis_target();
        return;
    }

    // NOTFOLLOW 模式下严格禁止任何旋转指令输入，避免干扰自瞄和 SPIN 状态
    const float raw_spin_target = 0.0f;

    calculate_remote_translation_target();
    update_translation_slope();

    omni_target[kAxisW].TIM_Calculate_PeriodElapsedCallback(raw_spin_target, omni_fk.GetChassisVw());
    chassis_target.target_rotation = omni_target[kAxisW].GetOut();
}

/**
 * @brief 底盘跟随 (FOLLOW) 目标规划。
 */
void Follow_SetTarget()
{
    if (is_remote_stop())
    {
        clear_chassis_target();
        return;
    }

    calculate_remote_translation_target();
    update_translation_slope();

    CalculateFollow();
    float follow_pid_output = follow_pid.getOutput();
    chassis_target.target_rotation = limit_float(follow_pid_output,
                                                 -ChassisConfig::kFollowOutputLimit,
                                                 ChassisConfig::kFollowOutputLimit);
}

/**
 * @brief 小陀螺 (SPIN) 目标规划。
 *
 * SPIN 模式不跟随云台，平移仍按云台坐标系解算；旋转角速度由归一化拨轮 get_scroll_() 连续控制。
 */
void Spin_SetTarget()
{
    calculate_remote_translation_target();
    update_translation_slope();

    omni_target[kAxisW].TIM_Calculate_PeriodElapsedCallback(get_scroll_spin_target(), omni_fk.GetChassisVw());
    chassis_target.target_rotation = omni_target[kAxisW].GetOut();
}

/**
 * @brief 设置底盘的目标值。
 *
 * 根据当前工作模式设置相应的目标值。
 */
void SetTarget()
{
    // 设置遥控器死区
    DT7.SetDeadzone(ChassisConfig::kRemoteDeadzone);

    switch (chassis_fsm.Get_Now_State())
    {
        case STOP:
            clear_chassis_target();
            break;
        case FOLLOW:
            Follow_SetTarget();
            break;
        case NOTFOLLOW:
            Notfollow_SetTarget();
            break;
        case SPIN:
            Spin_SetTarget();
            break;
        default:
            clear_chassis_target();
            break;
    }
}

/**
 * @brief 底盘急停。
 */
void chassis_stop()
{
    for (int i = 0; i < ChassisConfig::kWheelCount; i++)
    {
        wheel_pid[i].reset();
        chassis_output.out_wheel[i] = 0.0f;
    }

    // 清空底盘跟随云台 PID 积分，防止切回 FOLLOW 时发生突跳
    follow_pid.reset();
}

/**
 * @brief 底盘不跟随执行。
 */
void chassis_notfollow()
{
    apply_omni_wheel_control();
}

/**
 * @brief 底盘跟随执行。
 */
void chassis_follow()
{
    apply_omni_wheel_control();
}

/**
 * @brief 小陀螺执行。
 */
void chassis_spin()
{
    apply_omni_wheel_control();
}

/**
 * @brief 任务主函数。
 *
 * @param left_sw 左摇杆
 * @param right_sw 右摇杆
 * @param is_online 断联检测
 */
void main_loop_chassis(uint8_t left_sw, uint8_t right_sw, bool is_online, bool *alphabet)
{
    update_chassis_kinematics();

    chassis_fsm.StateUpdate(left_sw, right_sw, is_online, alphabet);
    SetTarget();

    switch (chassis_fsm.Get_Now_State())
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        case SPIN:
            chassis_spin();
            break;
        default:
            chassis_stop();
            break;
    }
}

/* 控制任务部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 控制任务主函数。
 *
 * @param argument 任务参数
 */
extern "C"
{
void Control(void const *argument)
{
    // 初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
    for (;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        //    SetTarget();
        // chassis_notfollow();
        bool dummy_alphabet[20] = {false};
        main_loop_chassis(DT7.get_s1(), DT7.get_s2(), check_online(), dummy_alphabet);

        osDelay(1);
    }
}
}
