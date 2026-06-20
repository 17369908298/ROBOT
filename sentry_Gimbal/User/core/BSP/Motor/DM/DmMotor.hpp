#ifndef Dm_Motor_hpp
#define Dm_Motor_hpp

#pragma once
#include <cmath>
#include "../MotorBase.hpp"
#include "../../../HAL/CAN/can_hal.hpp"

namespace BSP::Motor::DM
{
    enum Model
    {
        MIT = 0,
        ANGLEVELOCITY = 1,
        VELOCITY = 2
    };

    // 参数结构体定义
    struct Parameters
    {
        float P_MIN = 0.0;
        float P_MAX = 0.0;
        float V_MIN = 0.0;
        float V_MAX = 0.0;
        float T_MIN = 0.0;
        float T_MAX = 0.0;
        float KP_MIN = 0.0;
        float KP_MAX = 0.0;
        float KD_MIN = 0.0;
        float KD_MAX = 0.0;

        static constexpr uint32_t VelMode = 0x200;
        static constexpr uint32_t PosVelMode = 0x100;
        static constexpr double rad_to_deg = 1 / 0.017453292519611;

        Parameters(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, 
                   float kpmin, float kpmax, float kdmin, float kdmax)
            : P_MIN(pmin), P_MAX(pmax), V_MIN(vmin), V_MAX(vmax), 
              T_MIN(tmin), T_MAX(tmax), KP_MIN(kpmin), KP_MAX(kpmax),
              KD_MIN(kdmin), KD_MAX(kdmax)
        {
        }
    };

    /**
     * @brief 达妙电机的基类
     */
    template <uint8_t N> 
    class DMMotorBase : public MotorBase<N>
    {
    protected:
        // 🟢 完美修复：将结构体内部的数据类型改为 float，让 Keil Watch 窗口直接呈现真实物理量
        struct DMMotorfeedback
        {
            uint8_t  id;
            uint8_t  err;
            float    angle_Rad;     // 👈 改为 float，直接看当前弧度
            float    velocity_Rad;  // 👈 改为 float，直接看当前转速 (rad/s)
            float    torque_Nm;     // 👈 改为 float，直接看当前真实转矩 (N·m)
            uint8_t  T_Mos;
            uint8_t  T_Rotor;
        };

        /**
         * @brief 构造函数
         */
        DMMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
            : init_address(Init_id), params_(params)
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                recv_idxs_[i] = recv_ids[i];
                send_idxs_[i] = send_ids[i];
            }
        }

    private:
        float uint_to_float(int x_int, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            float offset = x_min;
            return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
        }

        int float_to_uint(float x, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            float offset = x_min;
            return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
        }

        void Configure(size_t i)
        {
            const auto &params = params_;
            constexpr double kPi = 3.14159265358979323846;

            // 1. 获取达妙电机的原始未处理数据 (局部多圈角度)
            double raw_rad = feedback_[i].angle_Rad;
            double raw_deg = raw_rad * params.rad_to_deg;

            // 2. 先处理多圈累加逻辑
            // 🚨 必须使用原始的 raw_deg 来判断跨圈跳变，否则 add_angle 累加器会错乱爆炸！
            double lastData = this->unit_data_[i].last_angle;
            double Data = raw_deg;

            double span = (params.P_MAX - params.P_MIN) * params.rad_to_deg;
            double half_span = span / 2.0;

            if (Data - lastData < -half_span)
                this->unit_data_[i].add_angle += (span - lastData + Data);
            else if (Data - lastData > half_span)
                this->unit_data_[i].add_angle += -(span - Data + lastData);
            else
                this->unit_data_[i].add_angle += (Data - lastData);

            this->unit_data_[i].last_angle = Data;

            // 3. 核心诉求：在这里把数据"拍扁"成严格单圈！
            // 弧度限制到 [-PI, PI]
            double single_rad = std::fmod(raw_rad, 2.0 * kPi);
            if (single_rad > kPi)       single_rad -= 2.0 * kPi;
            else if (single_rad < -kPi) single_rad += 2.0 * kPi;

            // 角度限制到 [-180, 180]
            double single_deg = std::fmod(raw_deg, 360.0);
            if (single_deg > 180.0)       single_deg -= 360.0;
            else if (single_deg < -180.0) single_deg += 360.0;

            // 4. 将处理好的单圈角度存入基类
            this->unit_data_[i].angle_Rad = single_rad;
            this->unit_data_[i].angle_Deg = single_deg;

            // 5. 同步其它物理量
            this->unit_data_[i].velocity_Rad = feedback_[i].velocity_Rad;
            this->unit_data_[i].velocity_Rpm = this->unit_data_[i].velocity_Rad * 9.5492965855f;
            this->unit_data_[i].torque_Nm = feedback_[i].torque_Nm;
            this->unit_data_[i].current_A = 0.0f;
            this->unit_data_[i].temperature_C = feedback_[i].T_Mos;
        }

    public:
        /**
         * @brief 解析CAN数据
         */
        void Parse(const HAL::CAN::Frame &frame) override
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                if (frame.id == init_address + recv_idxs_[i])
                {
                    const uint8_t* pData = frame.data;

                    feedback_[i].id = pData[0] & 0xF;
                    feedback_[i].err = (pData[0] >> 4) & 0xF;

                    // 🟢 核心修改：因为 feedback_ 变为了 float，我们先用局部变量把 CAN 字节拼接成原始定点整数
                    uint16_t raw_angle    = (pData[1] << 8) | pData[2];
                    uint16_t raw_velocity = (pData[3] << 4) | (pData[4] >> 4);
                    uint16_t raw_torque   = ((pData[4] & 0xF) << 8) | pData[5];

                    feedback_[i].T_Mos = pData[6];
                    feedback_[i].T_Rotor = pData[7];

                    // 🟢 核心修改：在这一步直接进行高精度解算，把解算后的 float 喂给 feedback_ 结构体
                    const auto &params = params_;
                    feedback_[i].angle_Rad    = uint_to_float(raw_angle, params.P_MIN, params.P_MAX, 16);
                    feedback_[i].velocity_Rad = uint_to_float(raw_velocity, params.V_MIN, params.V_MAX, 12);
                    feedback_[i].torque_Nm    = uint_to_float(raw_torque, params.T_MIN, params.T_MAX, 12);

                    // 刷新多圈角度统计和基类数据
                    Configure(i);
                    this->updateTimestamp(i + 1);
                }
            }
        }

        /**
         * @brief DM电机的MIT控制方法
         */
        void ctrl_Mit(uint8_t id, float _pos, float _vel, 
                float _KP, float _KD, float _torq)
        {
            uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
            pos_tmp = float_to_uint(_pos, params_.P_MIN, params_.P_MAX, 16);
            vel_tmp = float_to_uint(_vel, params_.V_MIN, params_.V_MAX, 12);
            kp_tmp = float_to_uint(_KP, params_.KP_MIN, params_.KP_MAX, 12);
            kd_tmp = float_to_uint(_KD, params_.KD_MIN, params_.KD_MAX, 12);
            tor_tmp = float_to_uint(_torq, params_.T_MIN, params_.T_MAX, 12);

            uint8_t send_data[8];
            send_data[0] = (pos_tmp >> 8);
            send_data[1] = (pos_tmp);
            send_data[2] = (vel_tmp >> 4);
            send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            send_data[4] = kp_tmp;
            send_data[5] = (kd_tmp >> 4);
            send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            send_data[7] = tor_tmp;

            HAL::CAN::Frame frame;
            frame.id = send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }


        /**
         * @brief DM电机的角度速度控制方法
         */
        void ctrl_AngleVelocity(uint8_t id, float _pos, float _vel)
        {
            uint8_t data[8];
            uint8_t *pbuf, *vbuf;

            pbuf = (uint8_t*)&_pos;
            vbuf = (uint8_t*)&_vel;

            for (int i = 0; i < 4; ++i) 
            {
                data[i] = pbuf[i];
                data[4 + i] = vbuf[i];
            }

            HAL::CAN::Frame frame;
            frame.id = 0X100 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

        /**
         * @brief DM电机的速度控制方法
         */
        void ctrl_Velocity(uint8_t id, float _vel)
        {
            uint8_t data[8] = {0};
            uint8_t *vbuf = (uint8_t*)&_vel;

            for (int i = 0; i < 4; ++i) 
            {
                data[i] = vbuf[i];
            }

            HAL::CAN::Frame frame;
            frame.id = 0X200 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }


        /**
         * @brief 使能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void On(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            
            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }
        
        /**
         * @brief 失能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void Off(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

        /**
         * @brief 获取DM电机错误码
         * @param id 电机编号 (1-based)
         * @return 0=未使能, 1=已使能无错, >1=硬件报错
         */
        uint8_t getError(uint8_t id)
        {
            return feedback_[id - 1].err;
        }

        /**
         * @brief 清除DM电机错误
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void ClearErr(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

    protected:
        const int16_t init_address;
        uint8_t recv_idxs_[N];
        uint32_t send_idxs_[N];
        DMMotorfeedback feedback_[N];
        Parameters params_;
    };

    /**
     * @brief J4310电机类
     */
    template <uint8_t N> 
    class J4310 : public DMMotorBase<N>
    {
    public:
        J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs, 
                            Parameters(-12.56f, 12.56f, -45.0f, 45.0f, -18.0f, 18.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

    /**
     * @brief S2325电机类
     */
    template <uint8_t N> 
    class S2325 : public DMMotorBase<N>
    {
    public:
        S2325(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs,
                            Parameters(-12.5f, 12.5f, -50.0f, 50.0f, -10.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

    template <uint8_t N>
    class J4340 : public DMMotorBase<N>
    {
    public:
        J4340(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs,
                            Parameters(-3.14, 3.14f, -50.0f, 50.0f, -9.0f, 9.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

    /**
     * @brief DMJ6006 电机类
     * 额定扭矩：4NM，峰值扭矩：11NM
     * 额定转速：150rpm，最大转速：240rpm
     * 减速比：6:1
     */
    template <uint8_t N>
    class J6006 : public DMMotorBase<N>
    {
    public:
        /**
         * @brief DMJ6006 电机构造函数
         *
         * 参数说明（根据DM电机通信协议）：
         * - 位置范围：根据14位编码器，减速比6:1，计算得 ±12.56 rad（±720°机械角度）
         * - 速度范围：根据额定转速150rpm，峰值可能更高，设定合理范围
         * - 力矩范围：根据额定4NM，峰值11NM，留一定余量
         */
        J6006(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs,
                            Parameters(
                                -12.56f, 12.56f,      // 位置范围 ±12.56 rad (输出轴 ±720°)
                                -25.0f, 25.0f,        // 速度范围 ±25 rad/s (输出轴 ±240rpm)
                                -12.0f, 12.0f,        // 力矩范围 ±12 Nm (额定4Nm，峰值11Nm，留余量)
                                0.0f, 500.0f,         // KP范围
                                0.0f, 5.0f            // KD范围
                            ))
        {
        }

        /**
         * @brief 获取输出轴的单圈角度 (Deg)
         * @param id 电机编号 (1-based)
         * @return 输出轴角度，范围限制在 [0.0, 360.0)
         */
        float getOutputAngleDeg(uint8_t id)
        {
            float rotor_total_deg = this->getAddAngleDeg(id);
            float output_total_deg = rotor_total_deg / 6.0f;
            float single_deg = std::fmod(output_total_deg, 360.0f);
            if (single_deg < 0.0f)
            {
                single_deg += 360.0f;
            }
            return single_deg;
        }

        /**
         * @brief 获取输出轴的单圈角度 (Rad)
         * @param id 电机编号 (1-based)
         * @return 输出轴角度，范围限制在 [0.0, 2π)
         */
        float getOutputAngleRad(uint8_t id)
        {
            constexpr float kPi = 3.14159265358979323846f;
            float rotor_total_rad = this->getAddAngleRad(id);
            float output_total_rad = rotor_total_rad / 6.0f;
            float single_rad = std::fmod(output_total_rad, 2.0f * kPi);
            if (single_rad < 0.0f)
            {
                single_rad += 2.0f * kPi;
            }
            return single_rad;
        }

        /**
         * @brief 获取输出轴转速 (rpm)
         * 反馈数据是转子速度，除以减速比得到输出轴速度
         */
        float getOutputVelocityRpm(uint8_t id)
        {
            return this->getVelocityRpm(id) / 6.0f;
        }

        /**
         * @brief 获取输出轴角速度 (rad/s)
         * 反馈数据是转子角速度，除以减速比得到输出轴角速度
         */
        float getOutputVelocityRads(uint8_t id)
        {
            return this->getVelocityRads(id) / 6.0f;
        }

        /**
         * @brief 获取输出轴扭矩 (Nm)
         * 反馈数据是转子扭矩，乘以减速比得到输出轴扭矩
         */
        float getOutputTorque(uint8_t id)
        {
            return this->getTorque(id) * 6.0f;
        }
    };

} // namespace BSP::Motor::DM

#endif
