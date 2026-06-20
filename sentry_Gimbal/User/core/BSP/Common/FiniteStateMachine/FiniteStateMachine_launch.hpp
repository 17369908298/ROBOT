/**
 * @file alg_fsm.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机 - 基于左右开关状态切换
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef FINITESTATEMACHINE_LAUNCH_H
#define FINITESTATEMACHINE_LAUNCH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported macros -----------------------------------------------------------*/

#define STATUS_MAX (10)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘状态定义
 */

enum Enum_Launch_States
{
    LAUNCH_STOP = 0,      // 停止状态
    LAUNCH_CEASEFIRE,
    LAUNCH_ONLY,
    LAUNCH_AUTO,
    LAUNCH_JAM,
    LAUNCH_STATUS_COUNT   // 状态数量
};


/**
 * @brief 状态结构体
 */
struct Struct_Status_launch
{
    const char* Name;           // 状态名称
    uint32_t Enter_Count;       // 进入次数统计
    uint32_t Total_Run_Time;    // 总运行时间
    void* User_Data;            // 用户数据指针
};

/**
 * @brief 有限自动机核心 - 基于左右开关状态切换
 */
class Launch_FSM
{
public:
    // 状态数组
    Struct_Status_launch Status[STATUS_MAX];

    // 当前底盘状态
    Enum_Launch_States State_launch;
    
    // 上一次不同的状态（记录上一个状态）
    Enum_Launch_States Last_Different_State;

    /**
     * @brief 初始化状态机
     */
    void Init();

    /**
     * @brief 获取当前状态
     */
    inline Enum_Launch_States Get_Now_State();

    /**
     * @brief 获取当前状态名称
     */
    inline const char* Get_Now_State_Name();

    /**
     * @brief 设置左右开关状态
     * 
     * @param left 左开关状态
     * @param right 右开关状态
     * @param equipment_online 设备是否在线
     */
    void SetState(uint8_t left, uint8_t right, bool equipment_online);

    /**
     * @brief 状态更新函数，根据左右开关状态更新底盘状态
     * 
     * @param left 左开关状态
     * @param right 右开关状态
     * @param equipment_online 设备是否在线
     * @param Change 左键是否有松开过
     * @param time 单发超时阈值时间
     * @param Vision 视觉给的的模式 0：停火，1：单发，2：连发
     * @param is_vision 视觉标志位是否真 是否右键加视觉标志位为真
     * @param is_shoot 单发时是否打了一发
     * @param is_jamming 是否检测到卡弹
     * @param alphabet 处理过的键盘输入字母是1还是0；数组的26，27分别是左键右键
     */
    void StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool Change, float time, uint8_t Vision, bool is_vision, bool is_shoot, bool is_jamming, bool *alphabet);

    /**
     * @brief 定时更新函数（用于时间统计）
     */
    void TIM_Update();

    /**
     * @brief 获取状态运行时间
     * 
     * @param state 状态
     * @return uint32_t 运行时间
     */
    uint32_t Get_State_Run_Time(Enum_Launch_States state);

    /**
     * @brief 获取状态进入次数
     * 
     * @param state 状态
     * @return uint32_t 进入次数
     */
    uint32_t Get_State_Enter_Count(Enum_Launch_States state);

    /**
     * @brief 重置状态统计信息
     * 
     * @param state 状态
     */
    void Reset_State_Statistics(Enum_Launch_States state);

private:
    // 左右开关状态
    uint8_t StateLeft = 2;
    uint8_t StateRight = 2;
    bool EquipmentOnline = false;
    
    // 状态运行时间计数
    uint32_t State_Run_Time[STATUS_MAX] = {0};
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前状态
 */
inline Enum_Launch_States Launch_FSM::Get_Now_State()
{
    return State_launch;
}

/**
 * @brief 获取当前状态名称
 */
inline const char* Launch_FSM::Get_Now_State_Name()
{
    return Status[State_launch].Name;
}

#endif
