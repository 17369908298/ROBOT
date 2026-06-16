#ifndef HI12_IMU_HPP
#define HI12_IMU_HPP

#include "HI12Base.hpp"

namespace BSP::IMU
{
    /**
     * @brief HI12 IMU传感器浮点数数据处理类
     * 
     * 该类用于处理来自HI12 IMU传感器的原始数据，并将其转换为浮点数值
     * 包括加速度、角速度、角度和四元数数据
     */
    class HI12_float : public HI12Base
    { 
        public: 
            /**
             * @brief 构造函数
             * @param huart UART句柄，用于与IMU传感器通信
             */
            HI12_float() 
                : acc{0}, gyro{0}, angle{0}, quaternion{0}, last_angle(0), add_angle(0)
            {
            }

            /**
             * @brief 更新IMU数据
             * @param pData 指向原始数据的指针
             * 
             * 从原始数据中解析出加速度、角速度、角度和四元数信息
             */
            void DataUpdate(uint8_t *pData)
            {
                if(pData == nullptr)
                {
                    return;
                }

                this->updateTimestamp();
                int frame_start = -1;
                for(int i = 0; i < RECEIVE_BUFFER_SIZE - FRAME_HEADER_SIZE; i++)  // 在整个缓冲区中搜索
                {
                    if(pData[i] == FRAME_HEADER_1 && pData[i+1] == FRAME_HEADER_2)
                    {
                        uint8_t len_low = pData[i + LENGTH_LOW_OFFSET];
                        uint8_t len_high = pData[i + LENGTH_HIGH_OFFSET];
                        int possible_len = len_low + (len_high << 8);
                            
                        // 检查长度是否合理
                        if(possible_len == PAYLOAD_LENGTH && i + FRAME_HEADER_SIZE + possible_len <= RECEIVE_BUFFER_SIZE)
                        {
                            frame_start = i;
                            break;
                        }
                    }
                }
                    
                if(frame_start < 0)
                {
                    return;
                }

                const uint8_t *frame = pData + frame_start;
                    
                // 从找到的帧头位置开始验证
                Verify(frame);
                    
                if(GetVerify())
                {
                    readAxisData(frame, ACC_OFFSET, acc);          // 单位: g
                    readAxisData(frame, GYRO_OFFSET, gyro);        // 单位: °/s
                    readAxisData(frame, ANGLE_OFFSET, angle);      // 单位: °
                    readQuaternionData(frame, QUATERNION_OFFSET);
                }

            }

            /**
             * @brief 获取加速度数据
             * @param index 索引值 (0:x/roll轴, 1:y/pitch轴, 2:z/yaw轴)
             * @return 对应轴的加速度值 (单位: g)
             */
            float GetAcc(int index) const
            {
                return acc[index];
            }

            /**
             * @brief 获取角速度数据
             * @param index 索引值 (0:x/roll轴, 1:y/pitch轴, 2:z/yaw轴)
             * @return 对应轴的角速度值 (单位: °/s)
             */
            float GetGyro(int index) const
            {
                return gyro[index];
            }

            /**
             * @brief 获取角速度数据
             * @param index 索引值 (0:x/roll轴, 1:y/pitch轴, 2:z/yaw轴)
             * @return 对应轴的角速度值 (单位: rad/s)
             */
            float GetGyroRad(int index) const
            {
                return gyro[index] * degToRad();
            }

            float GetGyroRPM(int index) const
            {
                return gyro[index] / degPerSecToRpm();
            }

            /**
             * @brief 获取角度数据
             * @param index 索引值 (0:x/roll轴, 1:y/pitch轴, 2:z/yaw轴)
             * @return 对应轴的角度值 (单位: °)
             */
            float GetAngle(int index) const
            {
                return angle[index];
            }

            /**
             * @brief 获取四元数数据
             * @param index 索引值 (0:w, 1:x, 2:y, 3:z)
             * @return 对应分量的四元数值
             */
            float GetQuaternion(int index) const
            {
                return quaternion[index];
            }

            /**
             * @brief 获取Pitch角度数据(0-180)
             * @return 对应轴的角度值 (单位: °)
             * 
             */
            float GetPitch_180() const
            {
                return angle[1] + 90.0f;
            }

            /**
             * @brief 获取Yaw角度数据(0-360)
             * @return 对应轴的角度值 (单位: °)
             * 
             */
            float GetYaw_360() const
            {
                return angle[2] + 180.0f;
            }

            /**
             * @brief 获取累计Yaw角度数据(0-360)
             * @return 对应轴的累计角度值 (单位: °)
             * 
             */
            float GetAddYaw()
            {
                float lastData = this->last_angle;
                float Data = this->angle[2] + 180.0f;

                if (Data - lastData < -180) // 正转
                    this->add_angle += (360 - lastData + Data);
                else if (Data - lastData > 180) // 反转
                    this->add_angle += -(360 - Data + lastData);
                else
                    this->add_angle += (Data - lastData);

                this->last_angle = Data;

                return this->add_angle;
            }
        private:
            static constexpr int RECEIVE_BUFFER_SIZE = HI12Base::RECEIVE_BUFFER_SIZE;
            static constexpr int FRAME_HEADER_SIZE = HI12Base::FRAME_HEADER_SIZE;
            static constexpr uint8_t FRAME_HEADER_1 = HI12Base::FRAME_HEADER_1;
            static constexpr uint8_t FRAME_HEADER_2 = HI12Base::FRAME_HEADER_2;
            static constexpr uint8_t LENGTH_LOW_OFFSET = HI12Base::LENGTH_LOW_OFFSET;
            static constexpr uint8_t LENGTH_HIGH_OFFSET = HI12Base::LENGTH_HIGH_OFFSET;
            static constexpr int PAYLOAD_LENGTH = HI12Base::EXPECTED_PAYLOAD_LENGTH;
            static constexpr uint8_t FLOAT_BYTES = 4;
            static constexpr uint8_t ACC_OFFSET = 12;
            static constexpr uint8_t GYRO_OFFSET = 24;
            static constexpr uint8_t ANGLE_OFFSET = 48;
            static constexpr uint8_t QUATERNION_OFFSET = 60;

            static constexpr float degToRad()
            {
                return 0.0174532925f;
            }

            static constexpr float degPerSecToRpm()
            {
                return 6.0f;
            }

            static constexpr uint8_t dataOffset(uint8_t first_offset, uint8_t index)
            {
                return first_offset + index * FLOAT_BYTES;
            }

            void readAxisData(const uint8_t *frame, uint8_t first_offset, float *target)
            {
                for(uint8_t axis = 0; axis < AXIS_COUNT; axis++)
                {
                    target[axis] = this->R4(frame + FRAME_HEADER_SIZE + dataOffset(first_offset, axis));
                }
            }

            void readQuaternionData(const uint8_t *frame, uint8_t first_offset)
            {
                for(uint8_t index = 0; index < QUATERNION_COUNT; index++)
                {
                    quaternion[index] = this->R4(frame + FRAME_HEADER_SIZE + dataOffset(first_offset, index));
                }
            }

            // 同轴索引约定: 0=x/roll, 1=y/pitch, 2=z/yaw
            float acc[3];            ///< 加速度数据 [x, y, z]
            float gyro[3];           ///< 角速度数据 [x, y, z]
            float angle[3];          ///< 欧拉角数据 [roll, pitch, yaw]
            float quaternion[4];     ///< 四元数数据 [w, x, y, z]
            float last_angle;
            float add_angle;
    };

    /**
     * @brief HI12 IMU传感器整型数据
     * 
     * 该类用于处理来自HI12 IMU传感器的原始数据，并将其转换为浮点数值
     * 使用整数解析后再转换为浮点数，适用于特定数据格式
     */
    // class HI12_int : public HI12Base
    // { 
    //     public: 
    //         /**
    //          * @brief 构造函数
    //          * @param huart UART句柄，用于与IMU传感器通信
    //          */
    //         HI12_int() 
    //             : offset(6), acc{0}, gyro{0}, angle{0}, quaternion{0}
    //         {
    //         }
     
    //         /**
    //          * @brief 更新IMU数据
    //          * @param pData 指向原始数据的指针
    //          * 
    //          * 从原始数据中解析出加速度、角速度、角度和四元数信息
    //          * 并将整数数据转换为对应的物理量浮点数值
    //          */
    //         void DataUpdate(uint8_t *pData)
    //         {
    //             this->updateTimestamp();
    //             Verify(pData);
    //             if(GetVerify())
    //             {
    //                 // 解析加速度数据 (单位: g)
    //                 // 原始数据为16位整数，乘以0.00048828转换为g值
    //                 acc[0] = this->Init16(pData+offset+0) * 0.00048828f;
    //                 acc[1] = this->Init16(pData+offset+2) * 0.00048828f;
    //                 acc[2] = this->Init16(pData+offset+4) * 0.00048828f;
                    
    //                 // 解析角速度数据 (单位: °/s)
    //                 // 原始数据为16位整数，乘以0.061035转换为°/s值
    //                 gyro[0] = this->Init16(pData+offset+6)  * 0.061035f;  
    //                 gyro[1] = this->Init16(pData+offset+8)  * 0.061035f;  
    //                 gyro[2] = this->Init16(pData+offset+10) * 0.061035f;  
                    
    //                 // 解析欧拉角数据 (单位: °)
    //                 // 原始数据为32位整数，乘以0.001转换为°值
    //                 angle[0] = this->Init32(pData+offset+18) * 0.001f;  // 使用Init32直接解析32位数据
    //                 angle[1] = this->Init32(pData+offset+22) * 0.001f;
    //                 angle[2] = this->Init32(pData+offset+26) * 0.001f;
                    
    //                 // 解析四元数数据
    //                 // 原始数据为16位无符号整数，乘以0.0001转换为四元数值
    //                 quaternion[0] = this->Uint16(pData+offset+32) * 0.0001f;
    //                 quaternion[1] = this->Uint16(pData+offset+34) * 0.0001f;
    //                 quaternion[2] = this->Uint16(pData+offset+36) * 0.0001f;
    //                 quaternion[3] = this->Uint16(pData+offset+38) * 0.0001f;
    //             }
    //         }

    //         /**
    //          * @brief 获取加速度数据
    //          * @param index 索引值 (0:x轴, 1:y轴, 2:z轴)
    //          * @return 对应轴的加速度值 (单位: g)
    //          */
    //         float GetAcc(int index)
    //         {
    //             return acc[index];
    //         }

    //         /**
    //          * @brief 获取角速度数据
    //          * @param index 索引值 (0:x轴, 1:y轴, 2:z轴)
    //          * @return 对应轴的角速度值 (单位: °/s)
    //          */
    //         float GetGyro(int index)
    //         {
    //             return gyro[index];
    //         }

    //         /**
    //          * @brief 获取角度数据
    //          * @param index 索引值 (0:roll, 1:pitch, 2:yaw)
    //          * @return 对应轴的角度值 (单位: °)
    //          */
    //         float GetAngle(int index)
    //         {
    //             return angle[index];
    //         }

    //         /**
    //          * @brief 获取四元数数据
    //          * @param index 索引值 (0:w, 1:x, 2:y, 3:z)
    //          * @return 对应分量的四元数值
    //          */
    //         float GetQuaternion(int index)
    //         {
    //             return quaternion[index];
    //         }

    //     private:
    //         int offset;              // 数据偏移量
    //         float acc[3];            // 加速度数据 [x, y, z]
    //         float gyro[3];           // 角速度数据 [x, y, z]
    //         float angle[3];          // 欧拉角数据 [roll, pitch, yaw]
    //         float quaternion[4];     // 四元数数据 [w, x, y, z]
    // };
}

#endif
