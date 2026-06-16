#ifndef HEAT_CONTROL_PRIVATE
#define HEAT_CONTROL_PRIVATE

#include "main.h"

namespace APP
{
    /**
     * @brief 热量控制系统私有实现类
     * 
     * 该类用于控制机器人发射机构的热量管理，通过监测电机电流和速度来判断是否可以安全发射弹丸，
     * 防止因过热导致发射机构损坏。主要功能包括射击检测、热量分配和热控制算法。
     */
    class Heat_Control_Private
    {
        public:
            /**
             * @brief 构造函数
             * @param window_size 滑动窗口大小，用于电流数据平滑处理
             * @param cooling_time 冷却时间，用于过滤机械回弹干扰或电流拖尾现象
             * @param integral_threshold 积分阈值，用于判断累计能量是否达到射击条件
             * @param noise_threshold 噪声阈值，用于识别正常运行时的背景噪声电流
             * @param delty_t 时间差参数，用于线性插值计算
             */
            Heat_Control_Private(uint8_t window_size, uint16_t cooling_time, float integral_threshold, float noise_threshold, float delty_t)
            {
                // 安全限制：窗口大小最大不超过200
                WindowSize = (window_size > 200) ? 200 : window_size;
                CoolingTime = cooling_time;
                IntegralThreshold = integral_threshold;
                NoiseThreshold = noise_threshold;
                DeltyT = delty_t;
                
                // 初始化状态变量
                win_idx = 0;                    // 窗口索引初始化为0
                win_sum = 0.0f;                 // 窗口总和初始化为0
                last_shoot_time = 0;            // 上次射击时间初始化为0
                acceleration_start_time = 0;    // 加速开始时间初始化为0
                last_target_speed = 0.0f;       // 上次目标速度初始化为0
                shot = false;                   // 射击标志初始化为false
                
                // 默认热量参数设置
                MaxHeat = 40.0f;    // 最大热量设为40.0
                Heat_CD = 12.0f;    // 散热速率设为12.0
                
                // 初始化滑动窗口数组
                for(int i=0; i<200; i++) window[i] = 0.0f;
            }

            /**
             * @brief 射击检测函数
             * @param target_speed 目标速度
             * @param current 电流指针，用于传入和更新电流数据
             * @param velocity 速度指针，用于传入和更新速度数据
             * 
             * 通过分析电流和速度数据来判断当前是否满足射击条件
             */
            void ShootingDetection(float target_speed, float *current, float *velocity);
            
            /**
             * @brief 热量分配函数
             * 
             * 根据当前系统状态和热量情况，合理分配可用的热量资源
             */
            void CalorieDistribution();
            
            /**
             * @brief 主热控制函数
             * @param target_speed 目标速度
             * @param current 电流指针
             * @param velocity 速度指针
             * @param max_heat 最大热量限制
             * @param heat_cd 散热系数
             * @param delty_t 时间差参数
             * @param target_fire 目标射击频率
             * 
             * 综合执行热控制流程：设置参数 -> 射击检测 -> 热量分配
             */
            void HeatControl(float target_speed, float *current, float *velocity, float max_heat, float heat_cd, float delty_t, float target_fire)
            {
                SetMaxHeat(max_heat);           // 设置最大热量
                SetHeatCD(heat_cd);             // 设置散热系数
                SetDeltyT(delty_t);             // 设置时间差参数
                SetTargetFire(target_fire);     // 设置目标射击频率
                ShootingDetection(target_speed, current, velocity);  // 执行射击检测
                CalorieDistribution();          // 执行热量分配
            }

            /**
             * @brief 设置目标射击频率
             * @param target_fire 目标射击频率值
             */
            void SetTargetFire(float target_fire)
            {
                this->target_fire = target_fire;
            }
            
            /**
             * @brief 设置最大热量限制
             * @param max_heat 最大热量值
             */
            void SetMaxHeat(float max_heat)
            {
                MaxHeat = max_heat;
            }
            
            /**
             * @brief 设置散热cd
             * @param heat_cd 散热系数值
             */
            void SetHeatCD(float heat_cd)
            {
                Heat_CD = heat_cd;
            }
            
            /**
             * @brief 设置时间差参数
             * @param delty_t 时间差值
             */
            void SetDeltyT(float delty_t)
            {
                DeltyT = delty_t;
            }

            /**
             * @brief 获取射击状态
             * @return bool 返回当前是否允许射击
             */
            bool GetShot()
            {
                return shot;
            }
            
            /**
             * @brief 获取当前射击频率
             * @return float 返回当前实际射击频率
             */
            float GetNowFire()
            {
                return now_fire;
            }
            
            /**
             * @brief 获取最大热量限制
             * @return float 返回最大热量值
             */
            float GetMaxHeat()
            {
                return MaxHeat;
            }
            
            /**
             * @brief 获取当前热量
             * @return float 返回当前系统热量值
             */
            float GetNowHeat()
            {
                return Now_Heat;
            }
            
        private:
            // === 配置参数 ===
            uint8_t WindowSize;         // 滑动窗口大小，用于存放电流数据
            uint16_t CoolingTime;       // 冷却时间
            float IntegralThreshold;    // 积分阈值
            float NoiseThreshold;       // 噪声阈值
            float MaxHeat;              // 最大热量限制
            float DeltyT;               // 时间差参数，用于线性插值计算和时间相关的算法处理
            float Now_Heat;             // 当前热量值
            float Heat_CD;              // 摩擦轮散热速率
            float target_fire;          // 目标射击频率

            // === 输出结果 ===
            bool shot;      // 射击标志位，true表示当前允许射击，false表示禁止射击
            float now_fire; // 当前实际射击频率，反映系统真实的发弹速率
            
            // === 中间计算变量 ===
            float window[200];          // 滑动窗口数组，固定最大缓冲区大小为200，用于存储历史电流数据
            int win_idx;                // 窗口索引，指向当前写入位置
            float win_sum;              // 窗口内数据总和，用于快速计算平均值
            uint32_t last_shoot_time;   // 上次射击时间戳，用于计算射击间隔
            uint32_t acceleration_start_time; // 加速开始时间戳，用于监测加速过程
            float last_target_speed;    // 上次目标速度记录，用于速度变化检测
    };
}

#endif // !HEAT_CONTROL_PRIVATE