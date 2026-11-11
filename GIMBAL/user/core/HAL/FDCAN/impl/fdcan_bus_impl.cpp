#include "fdcan_bus_impl.hpp"

namespace HAL::FDCAN
{

// FdcanBus实现
FdcanBus &FdcanBus::instance()
{
    static FdcanBus instance;
    // 懒汉模式：在第一次获取实例时初始化
    if (!instance.initialized_)
    {
        instance.init();
        instance.initialized_ = true;
    }
    return instance;
}

FdcanBus::FdcanBus()
    // 初始化FDCAN1
    : fdcan1_(&hfdcan1, 0, FDCAN_RX_FIFO0),
      // 初始化FDCAN2
      fdcan2_(&hfdcan2, 0, FDCAN_RX_FIFO0),
      // 初始化FDCAN3
      fdcan3_(&hfdcan3, 0, FDCAN_RX_FIFO0)
{
    // 注册现有的设备
    register_device(FdcanDeviceId::HAL_Fdcan1, &fdcan1_);
    register_device(FdcanDeviceId::HAL_Fdcan2, &fdcan2_);
    register_device(FdcanDeviceId::HAL_Fdcan3, &fdcan3_);
}

void FdcanBus::init()
{
    // 初始化所有已注册的设备
    for (size_t i = 0; i < (size_t)FdcanDeviceId::MAX_DEVICES; ++i)
    {
        if (devices_[i] != nullptr)
        {
            devices_[i]->init();
            devices_[i]->start();
        }
    }
}

void FdcanBus::register_device(FdcanDeviceId id, FdcanDevice *device)
{
    if (id < FdcanDeviceId::MAX_DEVICES && device != nullptr)
    {
        devices_[(size_t)id] = device;
    }
}

IFdcanDevice &FdcanBus::get_device(FdcanDeviceId id)
{
    if (id < FdcanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr)
    {
        return *devices_[(size_t)id];
    }

    // 如果没有可用设备，返回fdcan1_（保证永远有返回值）
    return fdcan1_;
}

bool FdcanBus::has_device(FdcanDeviceId id) const
{
    return id < FdcanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr;
}

} // namespace HAL::FDCAN
