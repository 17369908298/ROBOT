/**
 * @file fdcan_bus.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief FDCAN总线抽象接口
 * @version 0.0.1
 * @date 2026-09-15
 *
 * @copyright SZPU-RCIA (c) 2026
 *
 */

#pragma once
#include "fdcan_device.hpp"
#include <cstdint>

namespace HAL::FDCAN
{

// FDCAN设备ID枚举
enum class FdcanDeviceId : uint8_t
{
    HAL_Fdcan1 = 0,
    HAL_Fdcan2 = 1,
    HAL_Fdcan3 = 2,
    // 可以在此处添加更多FDCAN设备，无需修改接口
    MAX_DEVICES
};

// FDCAN总线抽象接口
class IFdcanBus
{
  public:
    virtual ~IFdcanBus() = default;

    // 获取指定ID的FDCAN设备
    virtual IFdcanDevice &get_device(FdcanDeviceId id) = 0;

    // 便捷访问方法（对标 UART 的 get_uart1() / get_uart5()）
    IFdcanDevice &get_fdcan1()
    {
        return get_device(FdcanDeviceId::HAL_Fdcan1);
    }
    IFdcanDevice &get_fdcan2()
    {
        return get_device(FdcanDeviceId::HAL_Fdcan2);
    }
    IFdcanDevice &get_fdcan3()
    {
        return get_device(FdcanDeviceId::HAL_Fdcan3);
    }

    // 检查指定ID的设备是否存在
    virtual bool has_device(FdcanDeviceId id) const = 0;
};

// 获取FDCAN总线单例实例
IFdcanBus &get_fdcan_bus_instance();

} // namespace HAL::FDCAN
