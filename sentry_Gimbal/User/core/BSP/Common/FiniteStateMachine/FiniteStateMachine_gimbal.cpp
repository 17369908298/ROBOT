#include "FiniteStateMachine_gimbal.hpp"

// 移除了 KEYBOARD 和 TRANSFORM 对应的字符串
static const char* State_Names[STATUS_COUNT] = {
    "STOP",
    "VISION",
    "MANUAL"
};

void Gimbal_FSM::Init()
{
    for (int i = 0; i < STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_gimbal = STOP;
    Status[STOP].Enter_Count = 1;

    StateLeft = 2;
    StateRight = 2;
    EquipmentOnline = false;
}

void Gimbal_FSM::SetState(uint8_t left, uint8_t right, bool equipment_online)
{
    StateLeft = left;
    StateRight = right;
    EquipmentOnline = equipment_online;
}

void Gimbal_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool vision_flag)
{
    Enum_Gimbal_States old_state = State_gimbal;
    SetState(left, right, equipment_online);

    // 1. 掉线或双下拨杆（急停）
    if (equipment_online == false || (left == 2 && right == 2))
    {
        State_gimbal = STOP;
    }
    // 2. 视觉模式优先级高于纯手动
    else if (vision_flag)
    {
        State_gimbal = VISION;
    }
    // 3. 其余一切情况，底盘无论怎么动，云台都是手动模式
    else
    {
        State_gimbal = MANUAL;
    }

    // 状态切换统计更新
    if (old_state != State_gimbal)
    {
        Status[old_state].Total_Run_Time += State_Run_Time[old_state];
        State_Run_Time[old_state] = 0;
        Status[State_gimbal].Enter_Count++;
    }
}

void Gimbal_FSM::TIM_Update()
{
    State_Run_Time[State_gimbal]++;
}

uint32_t Gimbal_FSM::Get_State_Run_Time(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_gimbal)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }
    return Status[state].Total_Run_Time;
}

uint32_t Gimbal_FSM::Get_State_Enter_Count(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }
    return Status[state].Enter_Count;
}

void Gimbal_FSM::Reset_State_Statistics(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_gimbal)
    {
        Status[state].Enter_Count = 1;
    }
}
