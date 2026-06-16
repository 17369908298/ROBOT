#ifndef HI12BASE_HPP
#define HI12BASE_HPP 

#include "../user/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../user/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#include <stdint.h>
#include <string.h>

namespace BSP::IMU
{
    class HI12Base
    {
        public:
            HI12Base(int timeThreshold = 100) : statewatch_(timeThreshold) 
            {
            }
            virtual ~HI12Base() = default;

            static constexpr uint8_t AXIS_COUNT = 3;
            static constexpr uint8_t QUATERNION_COUNT = 4;
            static constexpr uint8_t FRAME_HEADER_SIZE = 6;
            static constexpr uint8_t FRAME_HEADER_1 = 0x5A;
            static constexpr uint8_t FRAME_HEADER_2 = 0xA5;
            static constexpr uint8_t LENGTH_LOW_OFFSET = 2;
            static constexpr uint8_t LENGTH_HIGH_OFFSET = 3;
            static constexpr uint8_t CRC_LOW_OFFSET = 4;
            static constexpr uint8_t CRC_HIGH_OFFSET = 5;
            static constexpr uint16_t EXPECTED_PAYLOAD_LENGTH = 76;
            static constexpr uint16_t RECEIVE_BUFFER_SIZE = FRAME_HEADER_SIZE + EXPECTED_PAYLOAD_LENGTH;
            
            float R4(const uint8_t *p) 
            {
                float r; 
                memcpy(&r,p,4); 
                return r;
            };

            int16_t Init16(const uint8_t *p) 
            {
                int16_t r; 
                memcpy(&r,p,2); 
                return __builtin_bswap16(r);
            };

            uint16_t Uint16(const uint8_t *p)
            {
                uint16_t r; 
                memcpy(&r,p,2); 
                return __builtin_bswap16(r);
            }

            int32_t Init32(const uint8_t *p) 
            {
                int32_t r; 
                memcpy(&r,p,4); 
                return __builtin_bswap32(r);
            };

            void crc16_update(uint16_t *currentCrc, const uint8_t *src, uint32_t lengthInBytes)
            {
                uint16_t crc = *currentCrc;  // 使用16位，不是32位！
                for (uint32_t j = 0; j < lengthInBytes; ++j)
                {
                    crc ^= (src[j] << 8);  // 将字节左移8位后异或
                    for (uint32_t i = 0; i < 8; ++i)
                    {
                        if (crc & 0x8000)  // 检查最高位
                        {
                            crc = (crc << 1) ^ 0x1021;
                        }
                        else
                        {
                            crc <<= 1;
                        }
                    }
                }
                *currentCrc = crc;
            }

            void header()
            {
                if(Header1 == FRAME_HEADER_1 && Header2 == FRAME_HEADER_2)
                {
                    Header_flag = true;
                }
                else
                {
                    Header_flag = false;
                }
            }

            void crc(const uint8_t *pData)
            {
                payload_len = Length1 + (Length2 << 8);
                if(payload_len != EXPECTED_PAYLOAD_LENGTH)
                {
                    CRC_flag = false;
                    return;
                }

                crc_calculated = 0;
                crc16_update(&crc_calculated, pData, 4);
                crc16_update(&crc_calculated, pData + FRAME_HEADER_SIZE, payload_len);
                crc_received = CRC1 + (CRC2 << 8);
                if(crc_calculated == crc_received)
                {
                    CRC_flag = true;
                }
                else 
                {
                    CRC_flag = false;
                }
            }
            
            void Verify(const uint8_t *pData)
            {
                SetHeader(pData);
                SetCrc(pData);
                SetLength(pData);
                header();
                crc(pData);
            }

            
            void updateTimestamp()
            {
                statewatch_.UpdateLastTime();
            }

            bool isConnected()
            {
                statewatch_.UpdateTime();
                statewatch_.CheckStatus();
                if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
                {
                    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestIMURing();
                }
                return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
            }

            void SetUart(UART_HandleTypeDef *huart)
            {
                huart_ = huart;
            }

            void SetHeader(const uint8_t *pData)
            {
                Header1 = pData[0];
                Header2 = pData[1];
            }

            void SetLength(const uint8_t *pData)
            {
                Length1 = pData[LENGTH_LOW_OFFSET];
                Length2 = pData[LENGTH_HIGH_OFFSET];
            }

            void SetCrc(const uint8_t *pData)
            {
                CRC1 = pData[CRC_LOW_OFFSET];
                CRC2 = pData[CRC_HIGH_OFFSET];
            }

            bool GetVerify()
            {
                if(Header_flag && CRC_flag)
                {
                    return Verify_flag = true;
                }
                else
                {
                    return Verify_flag = false;
                }
            }

            void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
            {
                if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
                {
                    __HAL_UART_CLEAR_OREFLAG(huart);
                    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
                }
            }

        private:
            BSP::WATCH_STATE::StateWatch statewatch_;
            UART_HandleTypeDef *huart_;
            uint8_t Header1;
            uint8_t Header2;
            uint8_t Length1;
            uint8_t Length2;
            uint8_t CRC1;
            uint8_t CRC2;
            uint16_t payload_len;
            uint16_t crc_received;
            uint16_t crc_calculated;
            bool Header_flag = false;
            bool CRC_flag = false;
            bool Verify_flag = false;
    };
}

#endif
