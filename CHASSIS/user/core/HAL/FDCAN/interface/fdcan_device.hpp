/**
 * @file fdcan_device.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief FDCAN设备抽象接口
 * @version 0.0.1
 * @date 2026-09-15
 *
 * @copyright SZPU-RCIA (c) 2026
 *
 */

#pragma once
#include "fdcan.h"
#include <functional>

namespace HAL::FDCAN
{

// FDCAN消息ID类型
using ID_t = uint32_t;

// FDCAN数据帧结构体
struct Frame
{
    uint8_t data[8]{};  // 数据区
    ID_t id = 0;        // 帧ID（标准帧11位 / 扩展帧29位）
    uint8_t dlc = 0;    // 数据长度（字节数，0-8）
    uint32_t mailbox = 0; // 发送时作为Tx消息标记（MessageMarker），接收时无意义
    // 是否是扩展ID
    bool is_extended_id = false;
    // 是否是远程帧
    bool is_remote_frame = false;
};

// FDCAN接收回调函数类型
using RxCallback = std::function<void(const Frame &)>;

// FDCAN设备抽象接口
class IFdcanDevice
{
  public:
    virtual ~IFdcanDevice() = default;

    // 初始化FDCAN设备
    virtual void init() = 0;

    // 启动FDCAN设备
    virtual void start() = 0;

    // 发送FDCAN帧
    virtual bool send(const Frame &frame) = 0;

    // 接收FDCAN帧（非阻塞）
    virtual bool receive(Frame &frame) = 0;

    // 获取FDCAN句柄
    virtual FDCAN_HandleTypeDef *get_handle() const = 0;

    // 注册接收回调函数
    virtual void register_rx_callback(RxCallback callback) = 0;

    // 触发所有注册的回调函数
    virtual void trigger_rx_callbacks(const Frame &frame) = 0;

    // 从RX头提取FDCAN ID
    static ID_t extract_id(const FDCAN_RxHeaderTypeDef &rx_header);

    // 数据长度（字节数）转FDCAN的DLC编码
    static uint32_t to_dlc_code(uint8_t length);

    // FDCAN的DLC编码转数据长度（字节数）
    static uint8_t to_data_length(uint32_t dlc);
};

} // namespace HAL::FDCAN
