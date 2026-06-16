#include "../User/core/APP/Heat_Detector/Heat_Control_Private.hpp"
#include "math.h"

void APP::Heat_Control_Private::ShootingDetection(float target_speed, float *current, float *velocity)
{
    this->shot = false;
    uint32_t now = HAL_GetTick();

    // 冷却时间，过滤掉任何可能的机械回弹干扰或电流拖尾 冷却时间 = 控制周期 / 发弹频率
    if (now - last_shoot_time < this->CoolingTime) 
    {
        return; 
    }
    
    // 过滤摩擦轮没转和刚转时的误判
    // 监测目标速度变化 (从停转 -> 启动)
    if (last_target_speed < 100.0f && fabs(target_speed) > 1000.0f)
    {
        // 捕捉到启动瞬间，记录时间戳
        acceleration_start_time = now;
    }
    last_target_speed = fabs(target_speed);

    // 未启动 或者 启动的前1.5s
    if (fabs(target_speed) < 1000.0f || (now - acceleration_start_time < 1500))
    {
        win_sum = 0.0f;
        for(int i=0; i < this->WindowSize; i++) window[i] = 0.0f;
        return;
    }

    // 获取中间变量
    float cur_L = current[0];
    float cur_R = current[1];
    float speed_L = fabs(velocity[0]);
    float speed_R = fabs(velocity[1]);

    float current_input = 0.0f;

    // 过滤匀速时（没有打弹）噪声导致的误判
    if (speed_L > 3000.0f && speed_R > 3000.0f)
    {
        float total_current = fabs(cur_L) + fabs(cur_R);
        current_input = total_current - this->NoiseThreshold; 
        
        if (current_input < 0.0f) current_input = 0.0f;
    }

    // 滑动窗口累加和更新
    win_sum -= window[win_idx];       
    window[win_idx] = current_input; 
    win_sum += current_input;        
    
    win_idx++;
    if (win_idx >= this->WindowSize) win_idx = 0;

    // 如果窗口积分超过阈值，则判定为打弹
    if (win_sum > this->IntegralThreshold)
    {
        last_shoot_time = now;
        
        // 触发后清空
        win_sum = 0.0f;
        for(int i=0; i < this->WindowSize; i++) window[i] = 0.0f;
        
        this->shot = true;
    }
}

void APP::Heat_Control_Private::CalorieDistribution()
{
    if (shot)
    {
        Now_Heat += 10.0f;        // 累加热量
    }
    
    Now_Heat -= static_cast<float>(Heat_CD) * DeltyT;
    if (Now_Heat < 0.0f)
        Now_Heat = 0.0f;


        float DeltaHeat = MaxHeat - Now_Heat; // 当前剩余热量

        if (DeltaHeat > 0.7f * MaxHeat) // 如果没有超过限制缓冲区域，不做限制
        {
            now_fire = target_fire; // 此时不做限制
        }
        else if (DeltaHeat >= 0.3f * MaxHeat &&
                 DeltaHeat < 0.7f * MaxHeat) // 如果在缓冲区域内，没有超过停止阈值，做线性限制
        {
            // 热量不足，进行插值计算
            float heat_diff = 0.7f * MaxHeat - 0.3f * MaxHeat; // 缓冲区域的热量差 (终点-起点)
            float weight_target = (DeltaHeat - 0.3f * MaxHeat) / heat_diff; // 目标权重 (当前-起点)/(终点-起点)
            float weight_heat_cd = (0.7f * MaxHeat - DeltaHeat) / heat_diff; // 散热权重 (终点-当前)/(终点-起点)

            // 进行权重分配；
            now_fire = target_fire * weight_target + (Heat_CD / 10.0f) * weight_heat_cd;
        }
        else if (DeltaHeat < 0.3f * MaxHeat) // 如果超过了限制缓冲区域，直接停止射击
        {
            now_fire = 0.0f;
        }
}