/**
 * @file FiniteStateMachine_gimbal.hpp
 * @brief 云台有限自动机 - 极致精简版
 */

#ifndef FINITESTATEMACHINE_GIMBAL_H
#define FINITESTATEMACHINE_GIMBAL_H

#include "stdint.h"

// 将宏定义和结构体加上 GIMBAL 前缀，防止与底盘的 FSM 发生重定义冲突
#define GIMBAL_STATUS_MAX (10)

/**
 * @brief 云台状态定义
 */
enum Enum_Gimbal_States
{
    STOP = 0,      // 急停/停止状态
    VISION,        // 视觉辅助状态
    MANUAL,        // 手动控制状态 (默认状态)
    STATUS_COUNT   // 状态数量
};

/**
 * @brief 状态结构体
 */
struct Struct_Gimbal_Status
{
    const char* Name;           // 状态名称
    uint32_t Enter_Count;       // 进入次数统计
    uint32_t Total_Run_Time;    // 总运行时间
    void* User_Data;            // 用户数据指针
};

/**
 * @brief 云台有限自动机核心
 */
class Gimbal_FSM
{
public:
    Struct_Gimbal_Status Status[GIMBAL_STATUS_MAX];
    Enum_Gimbal_States State_gimbal;

    void Init();
    inline Enum_Gimbal_States Get_Now_State();
    inline const char* Get_Now_State_Name();

    void SetState(uint8_t left, uint8_t right, bool equipment_online);

    // 移除了 transform_flag 参数
    void StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool vision_flag);

    void TIM_Update();
    uint32_t Get_State_Run_Time(Enum_Gimbal_States state);
    uint32_t Get_State_Enter_Count(Enum_Gimbal_States state);
    void Reset_State_Statistics(Enum_Gimbal_States state);
    inline uint32_t Get_Current_Duration();

private:
    uint8_t StateLeft = 2;
    uint8_t StateRight = 2;
    bool EquipmentOnline = false;
    uint32_t State_Run_Time[GIMBAL_STATUS_MAX] = {0};
};

inline Enum_Gimbal_States Gimbal_FSM::Get_Now_State()
{
    return State_gimbal;
}

inline const char* Gimbal_FSM::Get_Now_State_Name()
{
    return Status[State_gimbal].Name;
}

inline uint32_t Gimbal_FSM::Get_Current_Duration()
{
    return State_Run_Time[State_gimbal];
}

#endif
