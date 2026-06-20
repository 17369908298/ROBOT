# Sentry Gimbal - 哨兵云台工程架构文档

> **平台:** STM32F4xx + FreeRTOS  
> **IDE:** Keil MDK-ARM / VSCode  
> **最后更新:** 2026-06-20

---

## 一、工程总览

```
sentry_Gimbal/
├── Core/                    # STM32 HAL 生成代码 (CAN, UART, GPIO, TIM, DMA, FreeRTOS)
├── Drivers/                 # CMSIS + STM32F4xx HAL 驱动库
├── Middlewares/             # FreeRTOS 内核源码
├── MDK-ARM/                 # Keil 工程文件、启动汇编、编译产物
└── User/                    # ⭐ 用户应用代码 (全部业务逻辑)
    ├── Task/                # FreeRTOS 任务入口 + 中断回调
    ├── core/
    │   ├── Alg/             # 控制算法库 (PID, ADRC, 滤波器, 功率控制, 底盘解算)
    │   ├── APP/             # 应用层 (裁判系统协议)
    │   ├── BSP/             # 板级支持包 (电机, IMU, 遥控器, 状态检测, 按键, FSM)
    │   └── HAL/             # 硬件抽象层 (CAN, UART, DWT, Logger, PWM, Assert)
    └── READ/                # 文档目录
```

---

## 二、分层架构

```
┌─────────────────────────────────────────────────────┐
│                   Task 任务层                         │
│  ControlTask · MotorTask · CommunicationTask         │
│  SerialTask · Callback                               │
├─────────────────────────────────────────────────────┤
│                   APP 应用层                          │
│  RM_RefereeSystem (裁判系统收发)                      │
├─────────────────────────────────────────────────────┤
│                   Alg 算法层                          │
│  PID · ADRC · Filter · Feedforward · SlopePlanning   │
│  PowerControl · ChassisCalculation (FK/ID/IK)        │
│  FSM                                                │
├─────────────────────────────────────────────────────┤
│                   BSP 板级支持层                       │
│  Motor (DJI/DM/LK) · IMU (HI12) · DT7 遥控器        │
│  StateWatch · BuzzerManager · SimpleKey · FSM        │
├─────────────────────────────────────────────────────┤
│                   HAL 硬件抽象层                       │
│  CAN (Bus/Device) · UART (Bus/Device)                │
│  DWT 计时器 · Logger (SEGGER RTT) · PWM              │
├─────────────────────────────────────────────────────┤
│              STM32 HAL + FreeRTOS                    │
└─────────────────────────────────────────────────────┘
```

---

## 三、核心模块详解

### 3.1 Task 任务层 (`User/Task/`)

| 文件 | 职责 |
|---|---|
| **ControlTask** | 云台 YAW 轴控制：三状态 FSM (STOP/VISION/MANUAL) + 角度环 PID + 掉线蜂鸣器报警，输出 yaw 电机指令 |
| **MotorTask** | DM J6006 电机驱动任务，周期性发送控制帧 + 解析反馈帧 |
| **CommunicationTask** | 板间 UART6 通信，收发自定义数据包 (含 CRC 校验) |
| **SerialTask** | UART 外设初始化：HI12 IMU (UART2) + DT7 遥控器 (UART3) |
| **Callback** | HAL 中断回调：CAN1 RX、UART2/3/6 RX/Error，分发给对应设备 |

**数据流:**
```
CAN RX IRQ ──→ Callback ──→ CanDevice ──→ Motor::Parse()
                                              ↓
UART2 RX ──→ Callback ──→ HI12_float::DataUpdate()    MotorBase::unit_data_[]
UART3 RX ──→ Callback ──→ DT7::Parse()                     ↓
UART6 RX ──→ Callback ──→ CommunicationTask            ControlTask 读取
                               ↓
                          ControlTask::MainLoopGimbal()
                            ├── 掉线检测 → BuzzerManagerSimple 入队
                            ├── BuzzerManagerSimple::update() 消费队列
                            ├── FSM 状态切换 → Yaw_SetTarget()
                            └── Yaw_Control() → out.yaw_torq
                               ↓
                          MotorTask → CAN TX
```

### 3.2 Alg 算法层 (`User/core/Alg/`)

#### PID (`Alg/PID/`)
- **类:** `ALG::PID::PID`
- **特性:** 积分分离、抗积分饱和、输出限幅
- **接口:** `UpDate(target, feedback)` → `output`

#### ADRC (`Alg/ADRC/`)
- **类:** `AdrcBase` → `FirstLADRC` (一阶) / `SecondLADRC` (二阶)
- **特性:** 自抗扰控制，含 TD 跟踪微分器、ESO 扩张状态观测器、NLSEF 非线性反馈

#### Filter (`Alg/Filter/`)
- **KalmanFilter** — 一维卡尔曼滤波
- **TD_Filter** — 跟踪微分器
- **LowPassFilter** — 一阶低通
- **LMF_Filter** — 最小均方自适应滤波

#### Feedforward (`Alg/Feedforward/`)
- **类:** `Alg::Feedforward::UphillFeedforward`
- **用途:** 上坡前馈补偿，基于重力分量计算额外力矩

#### SlopePlanning (`Alg/UtilityFunction/`)
- **类:** `Alg::Utility::SlopePlanning`
- **用途:** 斜坡规划器，平滑目标值变化速率

#### PowerControl (`Alg/PowerControl/`)
- **基类:** `PowerControlBase<N>`
- **策略:** `AttenuatedPower` (衰减功率)、`DecayingCurrent` (衰减电流)、`EnergyRing` (能量环)
- **用途:** 裁判系统功率限制下的电机功率分配

#### ChassisCalculation (`Alg/ChassisCalculation/`)
- **基类:** `ForwardKinematicsBase` / `InverseDynamicsBase` / `InverseKinematicsBase`
- **实现:** `Omni_FK/ID/IK` (全向轮)、`String_FK/ID/IK` (舵轮)

#### FSM (`Alg/FSM/`)
- **类:** `Class_FSM`
- **用途:** 通用有限状态机基类，含状态转移表、进入/退出回调

### 3.3 BSP 板级支持层 (`User/core/BSP/`)

#### Motor 电机驱动 (`BSP/Motor/`)

**类继承体系:**
```
MotorBase<N>                        ← 统一接口: getAngleDeg(), getVelocityRpm(), getTorque()...
├── DjiMotorBase<N>                 ← DJI CAN 协议 (大端, memcpy 直接解析)
│   ├── GM2006<N>                   ← C620 电调, 减速比 36:1
│   ├── GM3508<N>                   ← C620 电调, 减速比 268:17
│   └── GM6020<N>                   ← 内置驱动, 无减速
├── LkMotorBase<N>                  ← 灵明 CAN 协议
│   └── LK4005<N>
└── DMMotorBase<N>                  ← 达妙 CAN 协议 (小端定点, 需 uint_to_float 解算)
    ├── J4310<N>                    ← 量程 ±12.56 rad, 扭矩 ±18 Nm
    ├── S2325<N>                    ← 量程 ±12.5 rad,  扭矩 ±10 Nm
    ├── J4340<N>                    ← 量程 ±3.14 rad,  扭矩 ±9 Nm
    └── J6006<N>                    ← 量程 ±12.56 rad, 扭矩 ±12 Nm (带 6:1 减速)
```

**MotorBase 核心数据结构 `UnitData`:**
```cpp
struct UnitData {
    double angle_Deg;       // 角度 (°)
    double angle_Rad;       // 角度 (rad)
    double velocity_Rad;    // 角速度 (rad/s)
    double velocity_Rpm;    // 转速 (RPM)
    double current_A;       // 电流 (A)
    double torque_Nm;       // 扭矩 (N·m)
    double temperature_C;   // 温度 (°C)
    double last_angle;      // 上次角度 (用于多圈累加)
    double add_angle;       // 累计角度 (°)
};
```

**达妙电机数据流:**
```
CAN 帧 (8字节)
  ↓ Parse()
raw_angle / raw_velocity / raw_torque (uint16_t 定点数)
  ↓ uint_to_float()
feedback_.angle_Rad / velocity_Rad / torque_Nm (float 物理量)
  ↓ Configure()
unit_data_.angle_Rad / velocity_Rpm / torque_Nm / add_angle (基类标准数据)
```

#### IMU (`BSP/IMU/`)
- **基类:** `HI12Base` — 帧头校验 + CRC16 验证
- **实现:** `HI12_float` — 解析加速度、角速度、欧拉角、四元数

#### 遥控器 (`BSP/RemoteControl/`)
- **类:** `BSP::REMOTE_CONTROL::RemoteController` (DT7)
- **接口:** `getChannel(0-3)` 摇杆值、`getS1()/getS2()` 拨杆、`getMouse()` 鼠标、`getKey()` 键盘

#### 状态监测 (`BSP/Common/StateWatch/`)
- **StateWatch** — 在线/掉线超时检测，基于 `HAL_GetTick()` 计时
- **BuzzerManagerSimple** — 蜂鸣器队列管理 (单例)，支持电机/遥控器/IMU/板间通讯四种报警，已集成到 ControlTask 主循环

#### 按键 (`BSP/SimpleKey/`)
- **类:** `BSP::Key::SimpleKey`
- **特性:** 消抖、长按检测、切换模式 (Toggle)

#### FSM 状态机 (`BSP/Common/FiniteStateMachine/`)
- `Chassis_FSM` — 底盘状态机 (STOP / FOLLOW / GYRO / SMALL_GYRO)
- `Gimbal_FSM` — 云台状态机 (STOP / VISION / MANUAL)，掉线或双下拨杆→STOP，视觉辅助→VISION，其余→MANUAL
- `Launch_FSM`  — 发射状态机 (STOP / READY / AUTO / REVERSE)

### 3.4 HAL 硬件抽象层 (`User/core/HAL/`)

#### CAN (`HAL/CAN/`)
```
ICanBus (接口)          ICanDevice (接口)
   ↓                       ↓
CanBus (实现)           CanDevice (实现)
   └──── CanBusImpl ────┘
         每个 CAN 外设一个 CanBus 实例
         每个 CAN ID 一个 CanDevice 实例
```
- **单例获取:** `HAL::CAN::get_can_bus_instance().get_can1()` / `.get_can2()`
- **设备管理:** `registerDevice(id, callback)` / `unregisterDevice(id)`
- **发送:** `can_bus.send(frame)` — 阻塞等待空邮箱

#### UART (`HAL/UART/`)
```
IUartBus (接口)         IUartDevice (接口)
   ↓                       ↓
UartBus (实现)          UartDevice (实现)
   └──── UartBusImpl ────┘
```

#### DWT (`HAL/DWT/`)
- **类:** `HAL::DWTimer`
- **用途:** 基于 ARM DWT 周期计数器的高精度微秒级计时

#### Logger (`HAL/LOGGER/`)
- **类:** `HAL::LOGGER::Logger` (单例)
- **后端:** SEGGER RTT (无需额外串口，J-Link 直接读取)
- **接口:** `LOG_INFO()` / `LOG_WARN()` / `LOG_ERROR()` / `LOG_DEBUG()` 带颜色输出

### 3.5 APP 应用层 (`User/core/APP/`)

#### 裁判系统 (`APP/Referee/`)
- **协议:** RoboMaster 2024 裁判系统串口协议
- **功能:** 解析比赛状态、机器人血量、弹速/热量、自定义控制器数据
- **图形绘制:** `RM_DrawString()` / `RM_DrawLine()` / `RM_DrawRectangle()` / `RM_DrawCircle()`
- **CRC:** CRC8 帧头校验 + CRC16 数据校验

---

## 四、关键设计模式

### 4.1 模板化电机族
所有电机类以 `template <uint8_t N>` 参数化电机数量，编译期确定数组大小，零运行时开销。

### 4.2 统一接口多态
`MotorBase<N>` 提供统一的 `getAngleDeg()` / `getVelocityRpm()` / `getTorque()` 接口，上层任务无需关心电机型号。

### 4.3 CAN 设备注册制
CAN 收发采用 设备注册模式：每个电机/传感器在初始化时注册自己的 CAN ID 和回调，HAL 层收到帧后自动分发。

### 4.4 StateWatch 超时检测
每个电机绑定一个 `StateWatch` 实例，`Parse()` 成功时更新时间戳，周期检测超时则触发蜂鸣器报警。

### 4.5 FSM 状态机驱动
底盘、云台、发射器均采用独立 FSM 控制运行模式切换，任务主循环根据当前状态调用不同的控制策略。

---

## 五、Bug 报告

### 🔴 严重 (Critical)

| # | 文件 | 问题 | 影响 |
|---|---|---|---|
| 1 | `BSP/Common/StateWatch/state_watch.cpp` | `CheckStatus()` 将计算出的**已过时间**写回 `TimeThreshold_` 成员，永久破坏了构造时设定的**超时阈值** | 首次调用后在线/掉线检测完全失效 |
| ~~2~~ | ~~`Alg/PID/pid.cpp`~~ | ~~构造函数未初始化成员~~ | ✅ 已修复：构造函数末尾调用 `reset()` |
| ~~3~~ | ~~`Task/ControlTask.hpp` + `.cpp`~~ | ~~头文件声明 `extern ALG::ADRC::FirstLADRC yaw_adrc` 但未定义~~ | ✅ 已修复：`yaw_adrc` 已删除，仅保留 `yaw_angle_pid` |
| 4 | `BSP/IMU/HI12_imu.hpp` | `DataUpdate()` 搜索帧头后用 `frame_start` 偏移做 CRC 校验，但数据解析用固定的 `offset=6` | 帧头不在 byte 0 时，解析数据错位 |

### 🟡 中等 (Medium)

| # | 文件 | 问题 | 影响 |
|---|---|---|---|
| 5 | `Alg/UtilityFunction/SlopPlanning.cpp` | `Now_Planning` 初始值为 0，"真实值优先"逻辑比较的是旧规划值而非实际反馈 | 斜坡规划器首次响应可能大幅跳变 |
| 6 | `BSP/IMU/HI12Base.hpp` | `payload_len` 声明为 `int16_t`，高位字节 bit7 为 1 时变为负数 | CRC 校验循环次数异常巨大，可能死循环 |
| 7 | `BSP/Motor/Dji/DjiMotor.hpp` | `setCAN()` 中 `data << 8 >> 8` 对 `int16_t` 做移位，属于实现定义行为 | 可能产生符号扩展错误 |
| 8 | `Task/ControlTask.cpp` | `out.yaw_vel` 被无条件赋值为 `0.0f`，该字段从未使用 | 死代码，浪费结构体字段 |
| 9 | `APP/Referee/RM_RefereeSystem.cpp` | 多处 `while(HAL_UART_GetState() != READY)` 忙等待 | UART 异常时任务永久阻塞 |
| 10 | `APP/Referee/RM_RefereeSystem.cpp` | `rrrdata[20]` 缓冲区远小于 `RM_RefereeSystemData_t` 实际大小 | 数据截断丢失 |
| 11 | `BSP/Common/StateWatch/buzzer_manager.cpp` | `processRing()` 中调用 `osDelay()`，若从非任务上下文调用会崩溃 | 潜在 RTOS 堆栈异常 |

### 🟢 轻微 (Minor)

| # | 文件 | 问题 | 影响 |
|---|---|---|---|
| 12 | 多个文件 (`DjiMotor.hpp`, `Lk_motor.hpp`, `DT7.hpp`, `HI12Base.hpp`, `OmniCalculation.hpp`, `StringWheel.hpp`) | `#include` 路径使用小写 `../user/...` 而实际目录为 `User` | Windows 正常，Linux 交叉编译失败 |
| 13 | `Alg/Feedforward/Feedforward.hpp` | `#define PI` 和 `#define g` 污染全局宏命名空间 | `g` 作为单字母宏极易冲突 |
| 14 | `BSP/Motor/Dji/DjiMotor.hpp` | 重复 `#define PI`，与 `Feedforward.hpp` 冲突 | 编译警告 |
| 15 | `Alg/ChassisCalculation/StringWheel.hpp` | `#define M_PI` 可能与 `<cmath>` 冲突 | 编译警告或错误 |
| 16 | `BSP/Common/FiniteStateMachine/` | `Enum_Gimbal_States` 和 `Enum_Chassis_States` 在全局作用域定义 `STOP`、`VISION`、`MANUAL`、`STATUS_COUNT` 等枚举值 | 若两个 FSM 头文件被同一翻译单元包含则重定义 |
| 17 | `Alg/FSM/alg_fsm.hpp` + `FiniteStateMachine_chassis.hpp` | 两者都定义了 `struct Struct_Status`，成员不同 | 同上，全局命名冲突 |
| 18 | 三个 FSM 头文件 | 都定义 `#define STATUS_MAX (10)` | 值相同无害，但修改一处会影响全部 |
| 19 | `BSP/Motor/MotorBase.hpp` | `getAddAngleRad()` 返回 `add_angle`，但累加逻辑基于度数 | 函数名暗示返回弧度，实际返回度数 |
| 20 | `Alg/PowerControl-TestVersion/PowerControlTestVersion.hpp` | `SinExpected()` 第一行 `t = 0.001f` 直接覆盖参数 | 参数 `t` 永远无效 |
| 21 | `Alg/PowerControl-TestVersion/PowerControlTestVersion.hpp` | `P_cu`、`P_fe`、`P_mech` 等成员未在构造函数中初始化 | 首次使用可能为野值 |
| 22 | `HAL/LOGGER/logger.hpp` | 单例用 `new` 分配，无 `delete`；且 `inline` static 初始化顺序不确定 | 嵌入式环境可接受，但存在初始化顺序风险 |
| 23 | `HAL/DWT/DWT.hpp` | 闭合注释写 `} // namespace HAL::DWTimer`，实际是 `namespace HAL` | 仅注释误导 |
| 24 | `HAL/ASSERT/asster.hpp` | 文件名拼写错误 `asster` → 应为 `assert` | 困惑开发者 |
| 25 | `APP/Referee/RM_RefereeSystem.cpp` | CRC8 校验失败时 `State` 重置但 `RxIndex` 未重置，缓冲区残留脏数据 | 下次解析可能误判 |

---

## 六、编译与使用

### 6.1 Keil 编译
1. 打开 `MDK-ARM/sentry_Gimbal.uvprojx`
2. 选择目标芯片 (STM32F4xx)
3. Build (F7)

### 6.2 代码导航
- **电机驱动:** `User/core/BSP/Motor/` — 按厂商分子目录
- **控制算法:** `User/core/Alg/` — 每个算法独立目录
- **任务入口:** `User/Task/` — FreeRTOS 任务函数
- **硬件抽象:** `User/core/HAL/` — CAN/UART/DWT/Logger

### 6.3 添加新电机
1. 在 `BSP/Motor/` 下新建目录
2. 继承 `MotorBase<N>`，实现 `Parse()` 和 `Configure()`
3. 在 `Configure()` 中将厂商反馈数据转换为 `UnitData` 国际单位

---

## 七、待办事项 (TODO)

- [ ] **BUG-1:** 修复 `StateWatch::CheckStatus()` 中 `TimeThreshold_` 被覆盖的问题，改用局部变量 `elapsed`
- [x] **BUG-2:** PID 构造函数末尾调用 `reset()` 初始化所有状态变量 ✅
- [x] **BUG-3:** 删除 `yaw_adrc` 死代码，统一使用 `yaw_angle_pid` ✅
- [ ] **BUG-4:** 修复 IMU `DataUpdate()` 使用 `frame_start` 而非固定 `offset` 解析数据
- [ ] **BUG-6:** `HI12Base.hpp` 中 `payload_len` 改为 `uint16_t`
- [ ] **BUG-12:** 所有 `#include` 路径统一为大写 `User`
- [ ] **BUG-13/14/15:** 消除全局 `#define PI` / `#define g` / `#define M_PI`，改用 `constexpr` 或 `M_PI` 标准常量
- [ ] **BUG-16/17:** FSM 枚举值和结构体移入独立命名空间
- [ ] **BUG-19:** `getAddAngleRad()` 返回值除以 `rad_to_deg`，或改名为 `getAddAngleDeg()`
