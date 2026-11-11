/**
 * @file fdcan_bus_impl.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief FDCAN总线HAL层实现
 * @version 0.0.1
 * @date 2026-09-15
 *
 * @copyright SZPU-RCIA (c) 2026
 *
 */

#pragma once
#include "../interface/fdcan_bus.hpp"
#include "fdcan_device_impl.hpp"

namespace HAL::FDCAN
{

// FDCAN总线管理实现类
class FdcanBus : public IFdcanBus
{
  public:
    // 获取单例实例
    static FdcanBus &instance();

    // 析构函数
    ~FdcanBus() override = default;

    // 实现IFdcanBus接口
    IFdcanDevice &get_device(FdcanDeviceId id) override;
    bool has_device(FdcanDeviceId id) const override;

    // 初始化FDCAN总线（私有，由instance()调用）
    void init();

  private:
    // 私有构造函数（单例模式）
    FdcanBus();

    // 注册一个FDCAN设备
    void register_device(FdcanDeviceId id, FdcanDevice *device);

    // 是否已初始化标志
    bool initialized_ = false;

    // 禁止拷贝构造和赋值操作
    FdcanBus(const FdcanBus &) = delete;
    FdcanBus &operator=(const FdcanBus &) = delete;

    // 使用指针数组代替固定成员变量
    FdcanDevice *devices_[(size_t)FdcanDeviceId::MAX_DEVICES] = {nullptr};

    // 实际设备实例
    FdcanDevice fdcan1_;
    FdcanDevice fdcan2_;
    FdcanDevice fdcan3_;
};

} // namespace HAL::FDCAN
