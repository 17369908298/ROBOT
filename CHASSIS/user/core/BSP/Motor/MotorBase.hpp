#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#pragma once

#include "../user/core/BSP/Common/StateWatch/state_watch.hpp"

#include "../user/core/HAL/CAN/can_hal.hpp"

namespace BSP::Motor
{
    template <uint8_t N> class MotorBase
    {
    protected:
        struct UnitData
        {
            double angle_Deg; // 单位度角度
            double angle_Rad; // 单位弧度角度

            double velocity_Rad; // 单位弧度速度
            double velocity_Rpm; // 单位rpm

            double current_A;     // 单位安培
            double torque_Nm;     // 单位牛米
            double temperature_C; // 单位摄氏度

            double last_angle;  // 上一次位置
            double add_angle;   // 增量位置
        };

        // 国际单位数据
        UnitData unit_data_[N]{};
        uint32_t feedback_tick_[N]{};
        // 设备在线检测
        BSP::WATCH_STATE::StateWatch state_watch_[N];
        // 首帧反馈前不能仅凭距 tick=0 未超时就判在线，防止使用默认角度记录原点。
        bool feedback_received_[N]{};
        // 数据
        virtual void Parse(const HAL::CAN::Frame &frame) = 0;
        bool is_Enable = false;

    public:
        MotorBase(uint32_t timeThreshold = 100)
            : state_watch_{}  // 确保数组被默认初始化
        {
            for (int i = 0; i < N; i++) 
            {
                // 直接构造 StateWatch 对象
                state_watch_[i] = BSP::WATCH_STATE::StateWatch(timeThreshold);
            }
        }

        /**
         * @brief 
         * 
         * @param id 电机个数id
         */
        void updateTimestamp(uint8_t id)
        {
            if (id > 0 && id <= N)
            {
                state_watch_[id - 1].UpdateLastTime();
                feedback_received_[id - 1] = true;
                feedback_tick_[id - 1] = HAL_GetTick();
            }
        }

        /**
         * @brief 
         * 
         * @param id_state 状态id，对应电机个数
         * @param id_ring 对应can接收id
         * @return true 
         * @return false 
         */
        bool isConnected(uint8_t id_state, uint8_t id_ring)
        {
            if (id_state > 0 && id_state <= N)
            {
                if (!feedback_received_[id_state - 1])
                    return false;
                state_watch_[id_state - 1].UpdateTime();
                state_watch_[id_state - 1].CheckStatus();
                const bool online = state_watch_[id_state - 1].GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
                //BSP::BUZZER::setMotorOnline(id_ring, online);
                return online;
            }
            return false;
        }
        /**
         * @brief 获取角度
         *
         * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 -
         * 0x200，也就是1,
         * @return float
         */
        uint32_t getFeedbackTick(uint8_t id) const { return id>0 && id<=N ? feedback_tick_[id-1] : 0U; }
        bool hasFeedback(uint8_t id) const {return id>0 && id<=N && feedback_received_[id-1];}
        float getAngleDeg(uint8_t id)
        {
            return this->unit_data_[id - 1].angle_Deg;
        }

        /**
         * @brief 获取弧度
         *
         * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 - 0x200，也就是1,
         * @return float
         */
        float getAngleRad(uint8_t id)
        {
            return this->unit_data_[id - 1].angle_Rad;
        }

        /**
         * @brief 获取上一次角度
         *
         * @param id CAN id
         * @return float
         */
        float getLastAngleDeg(uint8_t id)
        {
            return this->unit_data_[id - 1].last_angle;
        }

        /**
         * @brief 获取增量角度
         *
         * @param id CAN id
         * @return float
         */
        float getAddAngleDeg(uint8_t id)
        {
            return this->unit_data_[id - 1].add_angle;
        }

        /**
         * @brief 获取增量弧度
         *
         * @param id CAN id
         * @return float
         */
        float getAddAngleRad(uint8_t id)
        {
            // 内部累计值以度保存；仅在读取时转为 rad，保留多圈累计量。
            return this->unit_data_[id - 1].add_angle * (3.14159265358979323846 / 180.0);
        }

        /**
         * @brief 获取速度    单位：(rad/s)
         * 这里是输出轴的速度，而不是转子速度
         * @param id CAN id
         * @return float
         */
        float getVelocityRads(uint8_t id)
        {
            return this->unit_data_[id - 1].velocity_Rad;
        }

        /**
         * @brief 获取速度    单位：(rpm)
         * 这里转子速度，不是输出轴的
         * @param id CAN id
         * @return float
         */
        float getVelocityRpm(uint8_t id)
        {
            return this->unit_data_[id - 1].velocity_Rpm;
        }

        /**
         * @brief 获取电流值    单位：(A)
         *
         * @param id CAN id
         * @return float
         */
        float getCurrent(uint8_t id)
        {
            return this->unit_data_[id - 1].current_A;
        }

        /**
         * @brief 获取力矩    单位：(Nm)
         *
         * @param id CAN id
         * @return float
         */
        float getTorque(uint8_t id)
        {
            return this->unit_data_[id - 1].torque_Nm;
        }

        /**
         * @brief 获取温度    单位：(°)
         *
         * @param id CAN id
         * @return float
         */
        float getTemperature(uint8_t id)
        {
            return this->unit_data_[id - 1].temperature_C;
        }

        /**
         * @brief 获取掉线的电机编号
         *
         * @return 掉线的电机编号（1-N），如果都在线则返回0
         */
        uint8_t getOfflineStatus()
        {
            for (uint8_t i = 0; i < N; i++)
            {
                if (this->state_watch_[i].GetStatus() != BSP::WATCH_STATE::Status::ONLINE)
                {
                    return i + 1; // 返回掉线电机的编号（从1开始计数）
                }
            }

            return 0; // 所有电机都在线
        }

        /**
         * @brief 获取电机使能状态
         *
         * @return true 电机使能
         * @return false 电机失能
         */
        bool getIsenable()
        {
            return this->is_Enable;
        }

        /**
         * @brief 设置电机使能状态
         * @param is_Enable true:使能  false:失能
         */
        void setIsenable(bool is_Enable)
        {
            this->is_Enable = is_Enable;
        }

    };
} // namespace BSP::Motor

#endif
