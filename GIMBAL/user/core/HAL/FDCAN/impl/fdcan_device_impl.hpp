/**
 * @file fdcan_device_impl.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief FDCAN设备HAL层实现
 * @version 0.0.1
 * @date 2026-09-15
 *
 * @copyright SZPU-RCIA (c) 2026
 *
 */

#pragma once
#include "../interface/fdcan_device.hpp"
#include <vector>

namespace HAL::FDCAN
{

// FDCAN硬件设备实现类
class FdcanDevice : public IFdcanDevice
{
  public:
    // 构造函数，初始化FDCAN设备
    // filter_index: 标准过滤器序号（0 ~ StdFiltersNbr-1）
    // fifo:         接收FIFO（FDCAN_RX_FIFO0 / FDCAN_RX_FIFO1）
    explicit FdcanDevice(FDCAN_HandleTypeDef *handle, uint32_t filter_index = 0, uint32_t fifo = FDCAN_RX_FIFO0);

    // 析构函数
    ~FdcanDevice() override = default;

    // 禁止拷贝构造和赋值操作
    FdcanDevice(const FdcanDevice &) = delete;
    FdcanDevice &operator=(const FdcanDevice &) = delete;

    // 实现IFdcanDevice接口
    void init() override;
    void start() override;
    bool send(const Frame &frame) override;
    bool receive(Frame &frame) override;
    FDCAN_HandleTypeDef *get_handle() const override;

    // 实现回调机制
    void register_rx_callback(RxCallback callback) override;
    void trigger_rx_callbacks(const Frame &frame) override;

  private:
    FDCAN_HandleTypeDef *handle_;
    uint32_t filter_index_;
    uint32_t fifo_;

    // 存储注册的回调函数
    std::vector<RxCallback> rx_callbacks_;

    // 配置过滤器
    void configure_filter();
};

} // namespace HAL::FDCAN
