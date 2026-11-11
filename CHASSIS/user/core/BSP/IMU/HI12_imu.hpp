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
                : offset(6), acc{0}, gyro{0}, angle{0}, quaternion{0}, last_angle(0), add_angle(0), rx_buffer{0}, rx_size(0)
            {
            }

            /**
             * @brief 更新IMU数据
             * @param pData 指向原始数据的指针
             * @param size 本次实际收到的字节数，允许分包和多帧。
             * 
             * 从原始数据中解析出加速度、角速度、角度和四元数信息
             */
            void DataUpdate(const uint8_t *pData, uint16_t size)
            {
                if (pData == nullptr) return;
                for (uint16_t i = 0; i < size; ++i)
                {
                    rx_buffer[rx_size++] = pData[i];
                    while (rx_size >= 2U)
                    {
                        if (rx_buffer[0] == 0x5A && rx_buffer[1] == 0xA5)
                        {
                            if (rx_size < 4U) break;
                            if (rx_buffer[2] == 76U && rx_buffer[3] == 0U)
                            {
                                if (rx_size < sizeof(rx_buffer)) break;
                                Verify(rx_buffer); // 长度固定为 76，完整 82 字节后才能校验。
                                if (GetVerify())
                                {
                                    ParseFrame(rx_buffer);
                                    this->updateTimestamp();
                                    rx_size = 0U;
                                    break;
                                }
                            }
                        }
                        // 错位、长度错误或 CRC 错误：逐字节重新寻找帧头。
                        --rx_size;
                        memmove(rx_buffer, rx_buffer + 1, rx_size);
                    }
                }
            }

            // 接收中断/错误恢复后丢弃半帧，保留最后一次有效测量值。
            void ResetReceiveBuffer()
            {
                rx_size = 0U;
            }

        private:
            void ParseFrame(uint8_t *pData)
            {
                    // 解析加速度数据 (单位: g)
                    acc[0] = this->R4(pData+offset+12);
                    acc[1] = this->R4(pData+offset+16);
                    acc[2] = this->R4(pData+offset+20);
                    
                    // 解析角速度数据 (单位: °/s)
                    gyro[0] = this->R4(pData+offset+24);
                    gyro[1] = this->R4(pData+offset+28);
                    gyro[2] = this->R4(pData+offset+32);
                    
                    // 解析欧拉角数据 (单位: °)
                    angle[0] = this->R4(pData+offset+48);
                    angle[1] = this->R4(pData+offset+52);
                    angle[2] = this->R4(pData+offset+56);
                    
                    // 解析四元数数据
                    quaternion[0] = this->R4(pData+offset+60);
                    quaternion[1] = this->R4(pData+offset+64);
                    quaternion[2] = this->R4(pData+offset+68);
                    quaternion[3] = this->R4(pData+offset+72);
            }

        public:
            /**
             * @brief 获取加速度数据
             * @param index 索引值 (0:x轴, 1:y轴, 2:z轴)
             * @return 对应轴的加速度值 (单位: g)
             */
            float GetAcc(int index)
            {
                return acc[index];
            }

            /**
             * @brief 获取角速度数据
             * @param index 索引值 (0:x轴, 1:y轴, 2:z轴)
             * @return 对应轴的角速度值 (单位: °/s)
             */
            float GetGyro(int index)
            {
                return gyro[index];
            }

            /**
             * @brief 获取角速度数据
             * @param index 索引值 (0:x轴, 1:y轴, 2:z轴)
             * @return 对应轴的角速度值 (单位: rpm)
             */
            float GetGyroRPM(int index)
            {
                return gyro[index] / 6.0f;
            }

            /**
             * @brief 获取角度数据
             * @param index 索引值 (0:roll, 1:pitch, 2:yaw)
             * @return 对应轴的角度值 (单位: °)
             */
            float GetAngle(int index)
            {
                return angle[index];
            }

            /**
             * @brief 获取四元数数据
             * @param index 索引值 (0:w, 1:x, 2:y, 3:z)
             * @return 对应分量的四元数值
             */
            float GetQuaternion(int index)
            {
                return quaternion[index];
            }

            /**
             * @brief 获取Pitch角度数据(0-180)
             * @return 对应轴的角度值 (单位: °)
             * 
             */
            float GetPitch_180()
            {
                return angle[1] + 90.0f;
            }

            /**
             * @brief 获取Yaw角度数据(0-360)
             * @return 对应轴的角度值 (单位: °)
             * 
             */
            float GetYaw_360()
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
            int offset;              ///< 数据偏移量
            float acc[3];            ///< 加速度数据 [x, y, z]
            float gyro[3];           ///< 角速度数据 [x, y, z]
            float angle[3];          ///< 欧拉角数据 [roll, pitch, yaw]
            float quaternion[4];     ///< 四元数数据 [w, x, y, z]
            float last_angle;
            float add_angle;
            uint8_t rx_buffer[82];   ///< 跨接收回调保存未完成帧。
            uint16_t rx_size;
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
