/**
 * @file alg_fsm.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机 - 基于左右开关状态切换
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#include "FiniteStateMachine_launch.hpp"

/* Private variables ---------------------------------------------------------*/

// 状态名称定义
static const char* State_Names[LAUNCH_STATUS_COUNT] = {
    "LAUNCH_STOP",
    "LAUNCH_CEASEFIRE",
    "LAUNCH_ONLY",
    "LAUNCH_AUTO",
    "LAUNCH_JAM"
};

/**
 * @brief 状态机初始化
 */
void Launch_FSM::Init()
{
    // 初始化所有状态
    for (int i = 0; i < LAUNCH_STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    // 设置初始状态
    State_launch = LAUNCH_STOP;
    Last_Different_State = LAUNCH_STOP;
    Status[LAUNCH_STOP].Enter_Count = 1;
    
    // 初始化开关状态 
    StateLeft = 2;
    StateRight = 2;
    EquipmentOnline = false;
}

/**
 * @brief 设置左右开关状态
 *
 * @param left 左开关状态
 * @param right 右开关状态
 * @param equipment_online 所有设备是否在线
 */
void Launch_FSM::SetState(uint8_t left, uint8_t right, bool equipment_online)
{
    StateLeft = left;
    StateRight = right;
    EquipmentOnline = equipment_online;
}

/**
 * @brief 状态更新函数，根据左右开关状态更新底盘状态
 *
 * @param left 左开关状态
 * @param right 右开关状态
 * @param equipment_online 所有设备是否在线
 * @param Change 左键是否有松开过
 * @param time 单发超时时间阈值
 * @param Vision 视觉发的模式
 * @param is_vision 右键是否按下并且视觉标志位，或者视觉标志位
 * @param is_shoot 是否弹丸有射出
 * @param is_jamming 是否卡弹
 * @param alphabet 按键状态
 */
void Launch_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool Change, float time, uint8_t Vision, bool is_vision, bool is_shoot, bool is_jamming, bool *alphabet)
{
    TIM_Update();
    // 保存旧状态用于统计
    Enum_Launch_States old_state = State_launch;
    
    if (equipment_online == false || (left == 2) || (left == 1 && right == 2)) // 掉线及遥控下的停止位
    {
        State_launch = LAUNCH_STOP;
        return; 
    }
    if(left == 3 && right == 3) // 键鼠下的停止位 B为1
    {
        if(!alphabet[1]/*按键B为0*/)
        {
            State_launch = LAUNCH_STOP;
            return; 
        }
    }
    
    switch (State_launch)
    {
        case LAUNCH_STOP:
            State_launch = LAUNCH_CEASEFIRE;
            break;
        case LAUNCH_CEASEFIRE:
            if(left == 3 && right == 3) // 键鼠
            {
                if(is_vision)   // 右键且视觉标志为真
                {
                    if(Vision == 0) // 视觉给的停火
                    {
                        State_launch = LAUNCH_CEASEFIRE;
                    }
                    else if(Vision == 1) // 视觉给的单发
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(Vision == 2) // 视觉给的连发
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                }
                else // 非视觉时
                {
                    if(alphabet[26]/*左键被按下*/)
                    {
                        if(Last_Different_State == LAUNCH_ONLY) // 如果上一次状态是单发那么判断左键有没有松开
                        {
                            if(Change)  // 左键松开过，切单发
                            {
                                State_launch = LAUNCH_ONLY;
                            }
                            else    // 左键没松开，保持停火，确保只打一发
                            {
                                State_launch = LAUNCH_CEASEFIRE;
                            }
                        }
                        else if(Last_Different_State != LAUNCH_ONLY) // 如果上一次状态不是单发，那么左键按下切连发
                        {
                            State_launch = LAUNCH_AUTO;
                        }
                    }
                    else // 左键没有被按下
                    {
                        if(alphabet[25] && !alphabet[23]/*按键Z为1,按键X为0*/)
                        {
                            State_launch = LAUNCH_ONLY;
                        }
                        else if(!alphabet[25] && alphabet[23]/*按键Z为0,按键X为1*/)
                        {
                            State_launch = LAUNCH_AUTO;
                        }
                        else if(!alphabet[25] && !alphabet[23]/*按键Z为0,按键X为0*/)
                        {
                            State_launch = LAUNCH_CEASEFIRE;
                        }
                    }
                }

            }
            else  // 遥控
            {
                if(left == 3)
                {
                    if(right == 2)  // 左中右下：单发
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(right == 1) // 左中右上：连发
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                }
                else if(left == 1)
                {
                    if(right == 1 || right == 3)  // 左上右上，左上右中：视觉模式，视觉做判断
                    {
                        if(is_vision) // 视觉标志位为真
                        {
                            if(Vision == 0)    // 视觉停止位
                            {
                                State_launch = LAUNCH_CEASEFIRE;
                            }
                            else if(Vision == 1)    // 视觉单发
                            {
                                State_launch = LAUNCH_ONLY;
                            }
                            else if(Vision == 2)    // 视觉连发
                            {
                                State_launch = LAUNCH_AUTO;
                            }
                        }
                        else // 视觉标志位为假
                        {
                            State_launch = LAUNCH_CEASEFIRE;
                        }
                    }
                }
            }
            break;
        case LAUNCH_ONLY:
            if(is_jamming)
            {
                State_launch = LAUNCH_JAM;
            }
            else if(left == 3 && right == 3) // 键鼠
            {
                if(is_vision)  // 右键且视觉标志为真
                {
                    if(Vision == 0) // 视觉给的停火
                    {
                        State_launch = LAUNCH_CEASEFIRE;
                    }
                    else if(Vision == 1) // 视觉给的单发
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(Vision == 2) // 视觉给的连发
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                }
                else // 非视觉时
                {
                    if(is_shoot/*打了一发弹*/)  // 打了一发弹回到停火，等松开左键
                    {
                        State_launch = LAUNCH_CEASEFIRE;
                    }
                    else
                    {
                        if(State_Run_Time[LAUNCH_ONLY] > time)  // 键鼠单发时间超过阈值 则切连发
                        {
                            State_launch = LAUNCH_AUTO;
                            // 同步按键状态：模拟按下了X键，松开Z键
                            alphabet[23] = true;
                            alphabet[25] = false;
                        }
                        else if(!alphabet[25] && alphabet[23]/*按键Z为0,按键X为1*/)
                        {
                            State_launch = LAUNCH_AUTO;
                        }
                        else if(alphabet[25] && !alphabet[23]/*按键Z为1,按键X为0*/)
                        {
                            State_launch = LAUNCH_ONLY;
                        }
                        else if(!alphabet[25] && !alphabet[23]/*按键Z为0,按键X为0*/)
                        {
                            State_launch = LAUNCH_CEASEFIRE;
                        }
                    }
                }
            }
            else // 遥控
            {
                if(is_shoot/*打了一发弹*/)
                {
                    State_launch = LAUNCH_CEASEFIRE;
                }
                else
                {
                    if(left == 3)  
                    {
                        if(right == 2)  // 左中右下：单发
                        {
                            State_launch = LAUNCH_ONLY;
                        }
                        else if(right == 1)  // 左中右上：连发
                        {
                            State_launch = LAUNCH_AUTO;
                        }
                    }
                    else if(left == 1)
                    {
                        if(right == 1 || right == 3)  // 左上右上，左上右中：视觉模式，视觉做判断
                        {
                            if(is_vision) // 视觉标志位为真
                            {
                                if(Vision == 0)    // 视觉停止位
                                {
                                    State_launch = LAUNCH_CEASEFIRE;
                                }
                                else if(Vision == 1)    // 视觉单发
                                {
                                    State_launch = LAUNCH_ONLY;
                                }
                                else if(Vision == 2)    // 视觉连发
                                {
                                    State_launch = LAUNCH_AUTO;
                                }
                            }
                            else // 视觉标志位为假
                            {
                                State_launch = LAUNCH_CEASEFIRE;
                            }
                        }
                    }
                }

            }
            break;
        case LAUNCH_AUTO:
            if(is_jamming)
            {
                State_launch = LAUNCH_JAM;
            }
            else if(left == 3 && right == 3) // 键鼠
            {
                if(is_vision)  // 右键且视觉标志为真
                {
                    if(Vision == 0) // 视觉给的停火
                    {
                        State_launch = LAUNCH_CEASEFIRE;
                    }
                    else if(Vision == 1) // 视觉给的单发
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(Vision == 2) // 视觉给的连发
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                }
                else
                {
                    if(!alphabet[25] && alphabet[23]/*按键Z为0,按键X为1*/)
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                    else if(alphabet[25] && !alphabet[23]/*按键Z为1,按键X为0*/)
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(!alphabet[25] && !alphabet[23]/*按键Z为0,按键X为0*/)
                    {
                        State_launch = LAUNCH_CEASEFIRE;
                    }
                }
            }
            else // 遥控
            {
                if(left == 3)
                {
                    if(right == 2)  // 左中右下：单发
                    {
                        State_launch = LAUNCH_ONLY;
                    }
                    else if(right == 1)  // 左中右上：连发
                    {
                        State_launch = LAUNCH_AUTO;
                    }
                }
                else if(left == 1)
                {
                    if(right == 1 || right == 3)  // 左上右上，左上右中：视觉模式，视觉做判断
                    {
                        if(is_vision) // 视觉标志位为真
                        {
                            if(Vision == 0)    // 视觉停止位
                            {
                                State_launch = LAUNCH_CEASEFIRE;
                            }
                            else if(Vision == 1)    // 视觉单发
                            {
                                State_launch = LAUNCH_ONLY;
                            }
                            else if(Vision == 2)    // 视觉连发
                            {
                                State_launch = LAUNCH_AUTO;
                            }
                        }
                        else // 视觉标志位为假
                        {
                            State_launch = LAUNCH_CEASEFIRE;
                        }
                    }
                }
            }
            break;
        case LAUNCH_JAM:
            // 反转一段时间后，回到上一个状态
            if(State_Run_Time[LAUNCH_JAM] > 30) // 150ms,控制周期给到了200Hz，所以是30
            {
                State_launch = Last_Different_State;
            }
            break;
        default:
            break;
    }
    
    // 如果状态发生变化，更新统计信息
    if (old_state != State_launch) {
        // 更新上一次不同的状态
        Last_Different_State = old_state;

        // 更新原状态的运行时间
        Status[old_state].Total_Run_Time += State_Run_Time[old_state];
        State_Run_Time[old_state] = 0;
        
        // 更新新状态的进入次数
        Status[State_launch].Enter_Count++;
    }
}

/**
 * @brief 定时更新函数（用于时间统计）
 */
void Launch_FSM::TIM_Update()
{
    // 更新当前状态的运行时间
    State_Run_Time[State_launch]++;
}

/**
 * @brief 获取状态运行时间
 *
 * @param state 状态
 * @return uint32_t 运行时间
 */
uint32_t Launch_FSM::Get_State_Run_Time(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT) {
        return 0;
    }
    
    if (state == State_launch) {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    } else {
        return Status[state].Total_Run_Time;
    }
}

/**
 * @brief 获取状态进入次数
 *
 * @param state 状态
 * @return uint32_t 进入次数
 */
uint32_t Launch_FSM::Get_State_Enter_Count(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT) {
        return 0;
    }
    return Status[state].Enter_Count;
}

/**
 * @brief 重置状态统计信息
 *
 * @param state 状态
 */
void Launch_FSM::Reset_State_Statistics(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT) {
        return;
    }
    
    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;
    
    // 如果是当前状态，重新计数进入次数
    if (state == State_launch) {
        Status[state].Enter_Count = 1;
    }
}

