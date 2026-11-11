# FDCAN驱动接口说明

本FDCAN驱动封装了STM32 HAL库的FDCAN功能（STM32H723，经典CAN格式），提供了更加易用、封装性好的现代C++接口。接口和实现分离，支持更好的可扩展性和可测试性。设计遵循开闭原则，可以轻松添加新的FDCAN设备而无需修改现有接口。

> 本模块由原 `HAL/CAN` 驱动迁移而来：`HAL/CAN` 面向STM32F4的bxCAN外设，本板卡（STM32H723）只有FDCAN外设，因此统一使用 `HAL::FDCAN` 命名空间。

## 核心特性

- 面向对象设计，使用类和命名空间组织代码
- 接口和实现分离，遵循依赖倒置原则
- 设计符合开闭原则，可扩展而无需修改接口
- 单例模式管理FDCAN总线实例（懒汉模式，自动初始化）
- 支持标准帧和扩展帧
- 支持数据帧和远程帧
- 支持FDCAN1 / FDCAN2 / FDCAN3三路总线
- 简化的过滤器配置
- 提供可读性强的接口

## 文件结构

### 目录组织

- `fdcan_hal.hpp`: 主头文件，包含所有接口
- `README.md`: 说明文档
- `interface/`: 接口目录
  - `fdcan_device.hpp`: FDCAN设备接口定义
  - `fdcan_device.cpp`: FDCAN设备接口实现（静态转换方法）
  - `fdcan_bus.hpp`: FDCAN总线接口定义
  - `fdcan_bus.cpp`: FDCAN总线接口实现
- `impl/`: 实现目录
  - `fdcan_device_impl.hpp`: FDCAN设备实现类定义
  - `fdcan_device_impl.cpp`: FDCAN设备实现类实现
  - `fdcan_bus_impl.hpp`: FDCAN总线实现类定义
  - `fdcan_bus_impl.cpp`: FDCAN总线实现类实现

### 接口与实现分离

这种目录结构将接口与实现明确分离，带来以下好处：

1. 用户代码只需包含 `fdcan_hal.hpp` 即可使用所有功能
2. 实现细节被隐藏在 `impl` 目录中，用户不需要关注
3. 便于替换具体实现而不影响用户代码
4. 便于理解代码结构和职责划分

## 使用方法

### 初始化FDCAN总线

采用懒汉模式，在第一次获取实例时自动初始化（内部调用 `HAL_FDCAN_Start()` 并激活接收中断）：

```cpp
// 获取实例时自动初始化FDCAN总线
HAL::FDCAN::get_fdcan_bus_instance();
```

### 发送FDCAN帧

```cpp
// 创建FDCAN帧
HAL::FDCAN::Frame frame = {};
frame.id = 0x201;              // 设置ID
frame.dlc = 8;                 // 数据长度为8字节
frame.is_extended_id = false;  // 使用标准ID
frame.is_remote_frame = false; // 数据帧，非远程帧

// 设置数据
frame.data[0] = 0x12;
frame.data[1] = 0x34;
// ...其他数据...

// 获取FDCAN总线实例
auto &fdcan_bus = HAL::FDCAN::get_fdcan_bus_instance();

// 方法1：使用便捷方法
if (!fdcan_bus.get_fdcan1().send(frame))
{
    // 发送失败处理（发送FIFO满 / 总线错误）
}

// 方法2：通过ID获取设备
fdcan_bus.get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan2).send(frame);

// 方法3：在发送前检查设备是否可用
if (fdcan_bus.has_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3)) {
    fdcan_bus.get_device(HAL::FDCAN::FdcanDeviceId::HAL_Fdcan3).send(frame);
}
```

### 接收FDCAN帧

接收统一走中断回调，把数据分发到各个设备注册的回调函数里：

```cpp
// 注册回调（初始化时执行一次）
auto &fdcan1 = HAL::FDCAN::get_fdcan_bus_instance().get_fdcan1();

fdcan1.register_rx_callback([](const HAL::FDCAN::Frame &frame) {
    if (frame.id == 0x201) {
        // 处理ID为0x201的数据
    }
});

// 可以注册多个回调，它们会按注册顺序依次执行
```

对应的中断回调（工程中位于 `RtosTask/can_send_task.cpp`）：

```cpp
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t)
{
    HAL::FDCAN::Frame frame;
    HAL::FDCAN::IFdcanDevice *device = nullptr;

    if (hfdcan->Instance == FDCAN1)      device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan1();
    else if (hfdcan->Instance == FDCAN2) device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan2();
    else if (hfdcan->Instance == FDCAN3) device = &HAL::FDCAN::get_fdcan_bus_instance().get_fdcan3();

    // receive() 内部会自动触发所有注册的回调；while 用于一次中断清空整个FIFO
    if (device != nullptr)
    {
        while (device->receive(frame)) {}
    }
}
```

## 设计说明

### 开闭原则实现

1. 使用 `FdcanDeviceId` 枚举和 `get_device(id)` 方法代替具体的设备访问方法
2. 实现基于ID的设备注册和查询机制
3. 添加新设备只需在实现类中增加实例并注册，无需修改接口

### Frame结构体

`Frame` 结构体封装了FDCAN帧的所有属性：

- `data`: 8字节数据数组
- `id`: 帧ID（标准11位或扩展29位）
- `dlc`: 数据长度（字节数，0-8）
- `mailbox`: 发送时作为Tx消息标记（MessageMarker），接收时无意义
- `is_extended_id`: 是否使用扩展ID（29位）
- `is_remote_frame`: 是否为远程帧

### IFdcanDevice接口

- `init()`: 配置过滤器
- `start()`: 启动FDCAN并激活接收中断
- `send()`: 发送FDCAN帧
- `receive()`: 接收FDCAN帧（非阻塞，成功后自动触发回调）
- `get_handle()`: 获取HAL FDCAN句柄
- `register_rx_callback()`: 注册接收回调函数
- `trigger_rx_callbacks()`: 触发所有已注册的回调函数
- `extract_id()`: 从接收头中提取ID（静态方法）
- `to_dlc_code()` / `to_data_length()`: 字节数与FDCAN DLC编码互转（静态方法）

### IFdcanBus接口

- `get_device(id)`: 获取指定ID的FDCAN设备
- `has_device(id)`: 检查指定ID的设备是否存在
- `get_fdcan1()` / `get_fdcan2()` / `get_fdcan3()`: 便捷访问方法

## 过滤器配置

`FdcanDevice::configure_filter()` 使用掩码模式的标准过滤器，`FilterID1 = FilterID2 = 0`，
即接收总线上所有标准帧，非匹配帧同样送入接收FIFO，远程帧直接拒收。
`FDCAN1/2/3` 的 `StdFiltersNbr` 均为1，因此过滤器序号固定为0。

若后续需要按ID过滤，修改 `configure_filter()` 中的 `FilterID1`（期望ID）与
`FilterID2`（掩码）即可，接口无需改动。

## 注意事项

1. 初始化顺序：首次调用 `get_fdcan_bus_instance()` 时会自动初始化三路FDCAN
2. 回调注册：建议在系统初始化时注册回调函数，且必须在首次发送/接收之前完成（回调存放在 `std::vector` 中）
3. 中断处理：只需调用 `receive()` 接收数据，它会自动触发所有注册的回调
4. 上下文：回调在中断上下文中执行，应保持简短，耗时处理请用队列或信号量转移到任务中
5. 位速率：由CubeMX生成代码（`Core/Src/fdcan.c`）配置，当前为经典CAN格式，1Mbps
6. 抽象接口：代码应当依赖于抽象接口（`IFdcanDevice` 和 `IFdcanBus`），而不是具体实现类
