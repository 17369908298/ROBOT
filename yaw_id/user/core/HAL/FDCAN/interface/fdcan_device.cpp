#include "fdcan_device.hpp"

namespace HAL::FDCAN
{

// 静态方法实现
ID_t IFdcanDevice::extract_id(const FDCAN_RxHeaderTypeDef &rx_header)
{
    // FDCAN的标准帧与扩展帧共用Identifier字段，由IdType区分
    return rx_header.Identifier;
}

uint32_t IFdcanDevice::to_dlc_code(uint8_t length)
{
    // 本工程使用经典CAN格式（FDCAN_CLASSIC_CAN，最多8字节），
    // DLC编码0-8与字节数一一对应，超过8字节按8字节处理
    if (length > 8)
    {
        return FDCAN_DLC_BYTES_8;
    }

    return static_cast<uint32_t>(length);
}

uint8_t IFdcanDevice::to_data_length(uint32_t dlc)
{
    // 经典CAN格式下DLC编码即为字节数，FD格式的更大DLC（12-64字节）不在本工程使用范围内
    if (dlc > FDCAN_DLC_BYTES_8)
    {
        return 0;
    }

    return static_cast<uint8_t>(dlc);
}

} // namespace HAL::FDCAN
