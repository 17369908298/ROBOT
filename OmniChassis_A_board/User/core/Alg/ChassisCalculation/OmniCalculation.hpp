#ifndef OmniCalculation_HPP
#define OmniCalculation_HPP

#include "../user/core/Alg/ChassisCalculation/CalculationBase.hpp"
#include <math.h>

namespace Alg::CalculationBase
{
    /**
     * @class Omni_FK
     * @brief 全向轮底盘正向运动学计算类
     * 
     * 根据四个全向轮的转速计算底盘的运动状态（速度和角速度）
     * 继承自ForwardKinematicsBase类
     */
    class Omni_FK : public ForwardKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 中心投影点到轮子投影点的距离
             * @param s 轮子半径
             * @param n 轮子数量
             * @param wheel_azimuth 轮子方位角 轮子自身的滚轮朝向（即允许自由滚动的方向）相对于底盘自身坐标系 X 轴正方向的夹角
             * @param wheel_direction 轮子坐标 “底盘几何中心”指向“当前轮子中心安装点”的这个向量，与底盘自身坐标系 X 轴正方向的夹角。
             */
            Omni_FK(float r, float s, float n, float wheel_azimuth[4], float wheel_direction[4]) 
                : R(r), S(s), N(n)
            {
                for(int i = 0; i < 4; i++)
                {
                    ChassisVx = 0.0f; 
                    ChassisVy = 0.0f;
                    ChassisVw = 0.0f;
                    WheelAzimuth[i] = wheel_azimuth[i];
                    WheelDirection[i] = wheel_direction[i];
                }
            }

            /**
             * @brief 执行正向运动学计算(小心符号问题)
             * 
             * 根据已设置的四个轮子转速，计算底盘在三个自由度上的运动状态
             * 使用全向轮正向运动学公式
             */
            void ForKinematics()
            {
                float wheel_vx = 0.0f, wheel_vy = 0.0f;
                // 【核心修复】必须清零上一帧的累加结果！
                ChassisVx = 0.0f;
                ChassisVy = 0.0f;
                ChassisVw = 0.0f;
                
                for(int i = 0; i < 4; i++)
                {
                    wheel_vx = S * Get_w(i) * cosf(WheelAzimuth[i]);
                    wheel_vy = S * Get_w(i) * sinf(WheelAzimuth[i]);

                    ChassisVx += wheel_vx;
                    ChassisVy += wheel_vy;

                    // 计算角速度贡献 (基于轮子安装位置)
                    float delta_angle = WheelAzimuth[i] - WheelDirection[i];
                    ChassisVw += (Get_w(i) * S * sinf(delta_angle)) / R;
                }
                ChassisVx /= (N/2);
                ChassisVy /= (N/2);
                ChassisVw /= N;
            }

            /**
             * @brief 完整的全向轮正向运动学计算
             * @param w0 轮子0的转速
             * @param w1 轮子1的转速
             * @param w2 轮子2 of the转速
             * @param w3 轮子3 of the转速
             * 
             * 先设置轮子转速，然后执行正向运动学计算
             */            
            void OmniForKinematics(float w0, float w1, float w2, float w3)
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            /**
             * @brief 获取中心投影点到轮子投影点的距离
             * @return 中心投影点到轮子投影点的距离R
             */
            float GetRadius() const { return R; }

            /**
             * @brief 获取轮子半径 
             * @return 轮子半径S
             */
            float GetScaling() const { return S; }

            /**
             * @brief 获取底盘X方向速度
             * @return X方向速度(前进/后退方向)
             */                  
            float GetChassisVx() const { return ChassisVx; }
            
            /**
             * @brief 获取底盘Y方向速度
             * @return Y方向速度(左移/右移方向)
             */   
            float GetChassisVy() const { return ChassisVy; }

            /**
             * @brief 获取底盘角速度
             * @return 绕Z轴角速度(旋转速度)
             */     
            float GetChassisVw() const { return ChassisVw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheelAzimuth(int index) { return WheelAzimuth[index]; }

            /**
             * @brief 轮子的位置方向角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方向
             */
            float GetWheelDirection(int index) { return WheelDirection[index]; }
        

        private:
            float R;                 // 中心投影点到轮子投影点的距离
            float S;                 // 轮子半径
            float N;                 // 轮子数量
            float ChassisVx;         // 底盘X方向速度
            float ChassisVy;         // 底盘Y方向速度
            float ChassisVw;         // 底盘绕Z轴角速度
            float WheelAzimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            float WheelDirection[4]; // 轮子的位置方向角（从车体中心指向轮子的方向）(弧度)
    };




    /**
     * @class Omni_ID
     * @brief 全向轮底盘逆向动力学计算类
     * 
     * 根据底盘受到的外力和力矩计算每个轮子需要产生的扭矩
     * 继承自InverseDynamicsBase类
     */
    class Omni_ID : public InverseDynamicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 中心投影点到轮子投影点距离
             * @param s 轮子半径
             * @param n 轮子数量
             * @param wheel_azimuth 轮子方位角 轮子自身的滚轮朝向（即允许自由滚动的方向）相对于底盘自身坐标系 X 轴正方向的夹角
             * @param wheel_direction 轮子坐标 “底盘几何中心”指向“当前轮子中心安装点”的这个向量，与底盘自身坐标系 X 轴正方向的夹角。
             */
            Omni_ID(float r, float s, float n, float wheel_azimuth[4], float wheel_direction[4]) 
                : R(r), S(s), N(n)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    WheelAzimuth[i] = wheel_azimuth[i];
                    WheelDirection[i] = wheel_direction[i];
                }
            }

            /**
             * @brief 执行逆向动力学计算(小心符号问题)
             * 
             * 根据已设置的底盘受力情况，计算每个轮子需要产生的扭矩
             * 使用全向轮逆向动力学公式
             */
            void InverseDynamics()
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = (cosf(WheelAzimuth[i]) / (N/2) * GetFx() + sinf(WheelAzimuth[i]) / (N/2) * GetFy() + (R * sinf(WheelAzimuth[i] - WheelDirection[i])) / N * GetTorque()) * S;
                }
            }

            /**
             * @brief 完整的全向轮逆向动力学计算
             * @param fx X方向力
             * @param fy Y方向力
             * @param torque 绕Z轴力矩
             * 
             * 先设置底盘受力情况，然后执行逆向动力学计算
             */
            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            /**
             * @brief 获取指定索引的电机所需扭矩
             * @param index 电机索引(0-3)
             * @return 对应电机的扭矩
             */
            float GetMotorTorque(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return MotorTorque[index];
                }
                return 0.0f; // 错误情况返回0
            }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheelAzimuth(int index) { return WheelAzimuth[index]; }

            /**
             * @brief 获取指定轮的位置方向角
             * @param index 轮索引(0-3)
             * @return 对应轮的位置方向角
             */
            float GetWheelDirection(int index) { return WheelDirection[index]; }
        
        private:
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float N;              // 轮子数量
            float MotorTorque[4]; // 四个电机的扭矩
            float WheelAzimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            float WheelDirection[4]; // 轮子的位置方向角（从车体中心指向轮子的方向）(弧度)
    };




    /**
     * @class Omni_IK
     * @brief 全向轮底盘逆向运动学计算类
     * 
     * 根据期望的底盘运动状态计算每个轮子的目标速度
     * 继承自InverseKinematicsBase类
     */
    class Omni_IK : public InverseKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 轮子投影点到中心距离
             * @param s 轮子半径
             * @param wheel_azimuth 轮子方位角 轮子自身的滚轮朝向（即允许自由滚动的方向）相对于底盘自身坐标系 X 轴正方向的夹角
             * @param wheel_direction 轮子坐标 “底盘几何中心”指向“当前轮子中心安装点”的这个向量，与底盘自身坐标系 X 轴正方向的夹角。
             */
            Omni_IK(float r, float s, float wheel_azimuth[4], float wheel_direction[4]) 
                : R(r), S(s) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
                    WheelAzimuth[i] = wheel_azimuth[i];
                    WheelDirection[i] = wheel_direction[i];
                }
            }

            /**
             * @brief 计算考虑增益和相位调整后的速度
             * 
             * 根据设定的目标速度和相位角，计算实际需要的速度分量
             */
            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * GetSignal_x();
                Vy = GetSpeedGain() * GetSignal_y();
                Vw = GetRotationalGain() * GetSignal_w();
            }

            /**
             * @brief 执行逆向运动学计算(小心符号问题)
             * 
             * 根据已计算的速度分量，计算四个轮子的目标转速
             * 使用极坐标角公式
             */
            void InvKinematics()
            {   
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = (cosf(WheelAzimuth[i]) * Vx + sinf(WheelAzimuth[i]) * Vy + Vw * R * sinf(WheelAzimuth[i] - WheelDirection[i])) / S;
                }
            }

            /**
             * @brief 完整的全向轮逆向运动学计算
             * @param vx X方向速度
             * @param vy Y方向速度
             * @param vw 绕Z轴角速度
             * @param phase 旋转矩阵的角度，需要包含补偿相位
             * @param speed_gain 速度增益
             * 
             * 设置目标运动状态，计算速度分量，然后执行逆向运动学计算
             */
            void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            /**
             * @brief 获取指定索引的电机目标速度
             * @param index 电机索引(0-3)
             * @return 对应电机的目标速度
             */
            float GetMotor(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return Motor[index];
                }
                return 0.0f; // 错误情况返回0
            }
            
            
            /**
             * @brief 获取X方向速度分量
             * @return X方向速度分量
             */
            float GetVx() const { return Vx; }

            /**
             * @brief 获取Y方向速度分量
             * @return Y方向速度分量
             */
            float GetVy() const { return Vy; }
            
            /**
             * @brief 获取角速度分量
             * @return 角速度分量
             */
            float GetVw() const { return Vw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheelAzimuth(int index) { return WheelAzimuth[index]; }

            /**
             * @brief 获取指定轮的位置方向角
             * @param index 轮索引(0-3)
             * @return 对应轮的位置方向角
             */
            float GetWheelDirection(int index) { return WheelDirection[index]; }

        private:
            float Vx{0.0f};       // X方向速度分量
            float Vy{0.0f};       // Y方向速度分量
            float Vw{0.0f};       // 角速度分量
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float Motor[4];       // 四个电机的目标速度
            float WheelAzimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            float WheelDirection[4]; // 轮子的位置方向角（从车体中心指向轮子的方向）(弧度)
    };
}

#endif
