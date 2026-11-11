# GM6020 力矩激励采集工程

这是独立的电机台架辨识固件，目标板沿用本目录的 STM32F427II、CubeMX 引脚和 RTOS 工程。新增代码位于 `app/`，**直接复用 `user/core` 的外设驱动**；没有另写 DT7、HI12、DJI 或 DM 协议解析器。工程不接入视觉、发射机构或原来的整车任务。

## 文件入口

| 文件 | 用途 |
| --- | --- |
| `app/yaw_id_config.hpp` | 电机、激励、行程、时效和辅助轴参数 |
| `app/yaw_id_core.hpp` | 激励波形、运行状态机、软限位和力矩限制 |
| `app/yaw_id_app.cpp` | 驱动接入、1 kHz 控制任务、接收时间戳、200 Hz 遥测 |
| `app/omni_reference_config.hpp` | 同目录兄弟工程 `SENTRY/GIMBAL/RtosTask/gimbal_config.hpp` 的配置快照 |
| `tools/decode_yaw.py` | 仅离线解码已保存的二进制文件，不打开串口 |
| `tools/identify_yaw_6020.m` | 配套 MATLAB 拟合入口，默认首尾裁剪各 3 秒，零时间偏移 |
| `MDK-ARM/Hero_gimbal.uvprojx` | Keil 工程，目标名称 `YawId` |

此前提到的 `config/user/onmi/gimbal.yaml` 在本工作区不存在，因此 roll 使用当前 SENTRY/GIMBAL 的 Omni 配置快照及本地 `user/core/Alg/Omni/roll_control.hpp` 控制器。编译不依赖兄弟工程目录。若后续修改 Omni 参数，需要同步此快照。

## 硬件与默认连接

| 功能 | 接口 | 参数 |
| --- | --- | --- |
| GM6020 小 yaw | CAN2，PB12 RX / PB13 TX，1 Mbps | 反馈 `0x206`；电流指令 `0x1FE` 第二槽；对应电机 ID 2 |
| roll J4310_2EC | CAN1，PD0 RX / PD1 TX，1 Mbps | 反馈 `0x07`，发送 `0x06`，MIT 模式 |
| pitch J4310_2EC | CAN1 | 反馈 `0x05`，发送 `0x04`，MIT 模式 |
| 大 yaw | CAN1，发送 `0x02` | 本采集程序始终请求失能，需要基座固定 |
| DT7 | USART1，PB7 RX / PB6 TX | 100000 baud，8 位有效数据、偶校验、1 停止位；HAL 配置为 9B + EVEN |
| HI12 主 IMU | UART7，PE7 RX / PE8 TX | 921600 baud，8N1；沿用现有 82 字节浮点帧驱动 |
| VOFA / Python | UART8，PE1 TX / PE0 RX | **115200 baud，8N1，JustFloat，10 通道，200 Hz** |

UART8 应用波特率由 `vofa_baud` 配置，当前与 CubeMX 默认值一样为 115200。TIM6 原初始化保持不变，应用启动后覆盖为 1 MHz 计数、1000 周期中断。UART7 在应用中改为循环 DMA，DT7 使用 ReceiveToIdle 普通 DMA。重新生成 CubeMX 时仍需保留上述应用接入；本次未修改 `.ioc`。

GM6020 必须支持本工程使用的 **电流控制指令 `0x1FE`**。力矩换算沿用已有 `TorqueCurrentConverter`，默认 `Kt=0.741 N·m/A`，`±16384 ↔ ±3 A`。I15 是反馈电流乘 Kt 的力矩估计，不是独立力矩传感器读数；Kt 未校准会同比影响辨识结果的绝对尺度。

## 启动与运行

1. 先检查电机 ID、方向、IMU 轴和可用行程。在机械中位开始，固定大 yaw 基座。默认行程以**开始采集时的位置为零**，不替代绝对机械限位；必须让配置范围小于实际剩余行程。
2. 上电后先让 DT7 至少一个拨杆离开中位，再将 **S1/S2 都置中位（编码 3/3）**。上电即处于 3/3 不会启动。
3. roll 先通过已有 Omni 阻抗控制器及 S 曲线到达 `2.98 rad`；稳定 0.3 秒后开始 yaw 激励。准备超过 20 秒仍不满足条件则故障锁定。
4. 实验运行期间，DT7 右 Y 可调整 pitch。正式采样时保持 pitch 角度固定，避免改变等效惯量。
5. 正常结束或故障后不会自动重启。有效反馈和正常任务时序下，yaw 可保留有界软 PD 回中制动；退出 3/3 或反馈失效则置零，辅助轴退出运行。重新拨杆退出、进入才能开始下一轮。

`roll_enabled/pitch_enabled` 默认均为 `true`。没有安装相应电机时，应关闭对应项并固定该机械自由度，否则 roll 到位条件会阻止开始。大 yaw 失能不等于机械固定。

roll 参数与当前配置快照一致：刚度 `32 N·m/rad`、阻尼 `2.3 N·m·s/rad`、速度上限 `30 rad/s`、力矩上限 `12.5 N·m`，轨迹速度/加速度/jerk 为 `10/35/400`，重力幅值 `1.3 N·m`、相位 `-1.457349925 rad`。pitch 仅用于手动设置姿态，其参数集中在配置末尾，没有将整套 Omni 模式逻辑搬入采集程序。

状态变量 `yaw_id_state`：`0=未启动`、`1=准备`、`2=激励`、`3=完成`、`4=故障`。`yaw_id_fault` 是应用故障位：`1=初始化失败`、`2=控制等待超时`、`4=8 ms 看门狗`、`8=CAN 提交失败`；位 4 在有效遥控退出运行档后清除，其余位保留诊断记录。

数据输出不需要 Python、上位机启动指令或拨杆使能。正常运行时 UART8 主动以 200 Hz 发送 JustFloat；若控制任务超过 20 ms 没有提供数据，发送任务会尝试约 50 Hz 的无效状态帧，I4=0、I5 包含 8192 位，便于在 VOFA 看到启动异常，且不启动任何电机。这个机制要求 RTOS 与 UART8 发送仍能运行，不能覆盖 MCU HardFault 或中断完全阻塞的情况。

无数据时可在调试器观察：`yaw_id_init_stage`（0=未开始，1=UART8，2=CAN，3=DT7，4=HI12，5=TIM6，6=初始化完成）、`yaw_id_vofa_tx_frames`（成功提交 DMA 次数）、`yaw_id_vofa_status_frames`（缺少控制数据时的状态帧次数）、`yaw_id_tx_drops`。DMA 提交次数增长不代表 PC 已收到数据；此时应继续核对 TX/RX 接线、串口号、115200 波特率和 JustFloat 协议。

## 激励与参数调整

默认 `Mode::SteppedSine`：频点 `0.2/0.5/1/2/3/4/6 Hz`，每个频点依次使用 `0.5/0.75/1.0` 三档幅值，每档 8 个周期，首尾各 1 个周期渐入渐出。每档相位独立且在零力矩处切换，总时长约 **222 秒**。

| 频率 Hz | 每档时长 s | 三档总时长 s | 最大档激励幅值 N·m |
| --- | ---: | ---: | ---: |
| 0.2 | 40 | 120 | 0.250 |
| 0.5 | 16 | 48 | 0.366 |
| 1 | 8 | 24 | 0.558 |
| 2 | 4 | 12 | 0.944 |
| 3 | 2.667 | 8 | 1.329 |
| 4 | 2 | 6 | 1.715 |
| 6 | 1.333 | 4 | 2.100 |

最终命令为 `软PD + 激励`，随后经过预测行程调节、总力矩限幅和变化率限制。拟合必须使用 **I1 总力矩**，不能只用 I5 激励，也不能再扣一次软 PD。

| 参数 | 默认值 | 调整方法 |
| --- | ---: | --- |
| `hard_angle_rad` | 0.45 rad | 以本轮初始角为中心的停止阈值；先按机械剩余行程缩小 |
| `governor_start_rad / governor_stop_rad` | 0.34 / 0.42 rad | 预测接近边界时削弱向外的力矩，应始终小于硬阈值 |
| `governor_prediction_s` | 0.10 s | 增大可更早削弱向外命令，但会减少边界附近激励 |
| `soft_kp / soft_kd` | 1.0 / 0.35 | 维持中心与阻尼；角度容易走偏时先少量增大 Kp，速度过大时增大 Kd |
| `soft_limit_nm` | 1.8 N·m | 软 PD 回中最大力矩 |
| `torque_limit_nm` | 2.2 N·m | 总命令上限，不得超过电流换算上限 |
| `torque_slew_nm_s` | 160 N·m/s | 过小会使高频激励严重变形；VOFA I5 会标记斜率限制 |
| `amplitude_low_nm / amplitude_high_nm` | 0.25 / 2.1 N·m | 低频与高频幅值包络；先减小幅值确认方向和行程，再逐步扩大 |
| `step_cycles` | 8 | 低频数据不足时增加；每档有效稳定部分为 6 周期 |
| `step_amplitude_scale` | 0.5 / 0.75 / 1.0 | 同频不同幅值有助于区分惯性、粘性和摩擦项 |

可将 `mode` 改为 Sine、Chirp 或 Prbs。Chirp 默认使用时间幂次 2 的频率分布，以增加低频停留；PRBS 使用固定幅值、随机正负号，切换间隔默认 0.1 秒。改变激励后需要重新编译。有限行程内无法任意同时提高低频速度和激励力矩，应以实测 I2/I3 的覆盖及保护介入比例为准。

反馈最大龄期为 GM/IMU 50 ms、DT7 100 ms、roll 300 ms。运行周期超过 2.5 ms 会终止本轮；独立 TIM6 检查控制任务超过 8 ms 未完成时请求零力矩并失能辅助轴。故障处理是软件请求，未经过实机停机响应验证。

## VOFA 精简数据通道（schema 5，当前默认）

VOFA 从 24 路缩减到 **10 路**。每帧 10 个 little-endian float32，后接 JustFloat 尾 `00 00 80 7F`，总计 **44 字节**。200 Hz 对应 8800 字节/s，8N1 占用 88000 bit/s，约为 115200 带宽的 76.4%，单帧发送约 3.82 ms，小于 5 ms 周期。

| 通道 | CSV 列名 | 含义 / 单位 |
| --- | --- | --- |
| I0 | time_s | 控制采样时间戳，s，以本轮开始激励时刻为零 |
| I1 | tau_cmd_nm | 限幅、斜率调节和电流量化后的最终总力矩，N·m |
| I2 | theta_rad | HI12 IMU 原始 yaw 角反馈，rad，按配置选轴和方向，不减控制器中心角 |
| I3 | omega_rad_s | 原始 IMU yaw 角速度，rad/s；拟合加速度由此求导 |
| I4 | mode | 0=无激励，1=正弦，2=chirp，3=PRBS，4=分段正弦 |
| I5 | flags | 运行有效性及保护标记，位含义见下表 |
| I6 | frame_sequence | 遥测生成序号，24 位循环，用于识别缺帧 |
| I7 | imu_time_s | I2/I3 对应有效 IMU 帧的接收时间戳，s，与 I0 使用同一本轮起点 |
| I8 | schema_version | 固定为 5，防止旧通道布局混用 |
| I9 | frame_crc24 | I0～I8 原始 36 字节 CRC32/ISO-HDLC 的低 24 位 |

I2 和 I3 直接取同一份完整、校验通过的 HI12 IMU 快照，单位分别从驱动的 deg/deg·s⁻¹ 转为 rad/rad·s⁻¹；不使用电机编码器角度或转速。IMU yaw 本身可能跨 ±π，检查角位移时需考虑回绕；内部控制器的相对角保护逻辑保持不变。

I0 和 I7 都来自同一扩展 DWT 微秒时钟，以本轮激励起点为零后换算为秒，因此 `I0-I7` 表示控制采样时 IMU 帧已接收多久。本轮刚开始时，最近一帧可能来自起点之前，I7 可为小负数。尚未开始本轮或遥控退出后，两路时间为 0；角度和速度继续输出最后有效 IMU 反馈。

I7 在有效 IMU 完整帧解析时记录，并按 DMA 字节位置估计帧结束时刻、扣除 IDLE 检测等待。它是**主控接收时间估计，不是 IMU 内部采样时刻**；不包含独立测得的 IMU 内部滤波延迟。重复使用同一帧 IMU 时，I7 保持不变。拟合用 I7 的实际间隔计算加速度，并剔除重复帧；力矩仍使用原始同帧 I1，不作自动延迟平移。必须使用同步更新的 schema-5 采集、拟合脚本。

### 调试器完整诊断数组（schema 3，不通过 UART8 发送）

`yaw_id_probe_channels[24]` 保留原布局，供 EmberProbe 或 Keil 排查力矩电流跟随、接收龄期等问题。下表的 I 编号仅指此诊断数组和旧版 24 通道文件，**不是上面的新版 VOFA 编号**。

| 通道 | CSV 列名 | 含义 / 单位 |
| --- | --- | --- |
| I0 | time_s | 本轮激励设备时间，s；未运行时为 0 |
| I1 | tau_cmd_nm | 总力矩经过限制和电流量化后的命令，N·m；提交失败帧带错误标记 |
| I2 | theta_rad | 相对本轮开始位置的 IMU yaw 角，rad；有效运行时使用 |
| I3 | omega_rad_s | 原始 IMU yaw 角速度，rad/s，未软件低通 |
| I4 | tau_base_nm | 限制前软 PD 分量，N·m |
| I5 | tau_id_nm | 限制前激励分量，N·m |
| I6 | mode | 0=无激励，1=正弦，2=chirp，3=PRBS，4=分段正弦 |
| I7 | flags | 状态和保护位，见下表 |
| I8 | motor_feedback_age_ms | 打包时距离最近 CAN 反馈接收的时间，ms |
| I9 | imu_age_ms | 打包时距离最近有效 IMU 完整帧接收的时间，ms |
| I10 | frame_sequence | 遥测生成序号，24 位循环 |
| I11 | motor_sequence | 有效 CAN 反馈序号，24 位循环 |
| I12 | imu_sequence | 有效 IMU 帧序号，24 位循环 |
| I13 | tx_drops | 队列满 / UART 忙 / 启动发送失败的累计次数，24 位循环 |
| I14 | can_submit_age_ms | 打包时距最近成功提交 CAN 邮箱的时间，ms |
| I15 | tau_meas_nm | 电机反馈电流 × Kt，N·m |
| I16 | can_submit_torque_nm | 最近成功提交 CAN 邮箱的量化力矩，N·m |
| I17 | excitation_frequency_hz | 当前激励频率，Hz；PRBS 为切换频率 |
| I18 | excitation_amplitude_nm | 当前设定的激励幅值，N·m，不含渐入渐出 |
| I19 | excitation_stage | 分段编号，从 0 开始 |
| I20 | motor_temperature_c | GM6020 温度，℃ |
| I21 | pitch_rad | IMU pitch 姿态，rad |
| I22 | schema_version | 固定为 3 |
| I23 | frame_crc24 | I0～I22 原始 92 字节 CRC32/ISO-HDLC 的低 24 位 |

状态标记使用位或组合（新版 VOFA I5 / 完整诊断数组 I7）：

| 值 | 含义 |
| ---: | --- |
| 1 | 未运行 / 遥控未使能 |
| 4 | 打包时遇到非有限数并置零 |
| 8 | 完成 / 故障锁定，无辨识激励 |
| 16 | 硬行程或硬速度阈值 |
| 64 | 力矩斜率调节 |
| 128 | 总力矩限幅 |
| 256 | 激励渐入渐出 |
| 512 | 反馈无效或超时；可与 4 组合 |
| 1024 | 等待辅助轴稳定 |
| 2048 | 预测行程调节 |
| 8192 | 时序或输入有效性不满足运行条件 |
| 16384 | 本周期 CAN 命令提交失败 |

所有接收时刻、控制和打包使用同一扩展 DWT 微秒时钟；UART 帧结束时刻按 DMA 位置及已传输字节估计，减掉 IDLE 检测等待。没有对原始角速度作固定样本移动。快照反映该控制周期最新已收到的反馈，**不代表 IMU 采样与电机命令在物理上同时发生**；IMU 内部滤波 / 传感器采样时刻、电流环响应和 CAN 线上 ACK 时间均未单独测得。

**诊断数组 I8/I9/I14 是接收或提交龄期，不是命令到运动的延迟；不能直接填入 2 ms 或 10 ms 拟合平移。** 当前精简串口 I7 是 IMU 接收时间戳，已不再是旧 schema 4 的龄期字段。诊断数组 I16 提交成功仅表示进入 CAN 邮箱，不能当作电机已执行证明。需要时通过调试器检查 I15 与 I16 的幅值和相位关系，再分析剩余动态误差。

EmberProbe 可观察 `yaw_id_probe_channels[0]`～`[23]`、`yaw_id_state`、`yaw_id_fault`、`yaw_id_tx_drops`、`yaw_id_can_errors`。数组在临界区内统一更新，但调试器跨次读取仍可能混帧，应同时检查该数组 I22/I23/I10；高频拟合采集优先用串口完整帧。当前未修改插件配置或连接目标板。

## VOFA 直接采集、离线回放与拟合

在线采集完全由 VOFA 完成，已移除 `capture_yaw.py` 的串口采集功能。构建和离线测试工具不打开 COM 口。

在 VOFA 选择实际在线的串口，配置 **115200、8N1、无流控、JustFloat**，打开后直接接收 10 通道。I8 应为 5，I6 帧序号应增长；画角度、速度时选择 I2/I3，避免让 I9 校验值的大量程挤压显示。采样周期由板端设备时间决定，不能用 VOFA 绘图的显示间隔替代。

保存或导出 CSV 时保留 I0～I9 全部通道，列名可使用 `I0`～`I9`（也支持 `CH0`、`Channel0` 等）；额外的主机 `Time` 列会忽略。MATLAB 只使用板端时间与 IMU 时间戳。发送丢包累计数保留为调试器全局变量，精简串口通过 I6 序号识别丢帧。

如果之前已经保存了原始 JustFloat 二进制文件，可使用**离线**工具转换，它没有 `--port` 参数、不包含串口打开逻辑：

```powershell
python tools/decode_yaw.py --decode data/run01.bin --out data/run01_replay.csv
# 回放之前 24 通道的原始文件，显式指定旧版：
python tools/decode_yaw.py --schema 3 --decode data/old.raw.bin --out data/old_replay.csv
# 回放此前 I7 为龄期的 10 通道版本：
python tools/decode_yaw.py --schema 4 --decode data/old_compact.raw.bin --out data/old_compact_replay.csv
```

MATLAB 中从本工程根目录执行：

```matlab
addpath('tools');
r = identify_yaw_6020("data/vofa_run01.csv", VerifyCRC=false);
% 联合多个文件：
% r = identify_yaw_6020(["data/vofa_run01.csv", "data/vofa_run02.csv"], VerifyCRC=false);
% 对保留原 float32 精度的离线解码 CSV，使用默认 CRC 校验：
% r = identify_yaw_6020("data/run01_replay.csv");
```

`VerifyCRC=false` 是针对 VOFA 十进制 CSV 可能舍入小数、无法重建原始 float32 字节的情况，脚本会明确警告。此时仍检查版本、有限数、状态标记、反馈龄期和时间连续性，但不提供原始字节级完整性验证。原始二进制离线转换得到的 CSV 默认验证 CRC。

拟合模型为 `tau_cmd = J*alpha + B*omega + Fc*tanh(omega/omega_s)`。入口兼容当前 schema 5 以及旧 schema 4/3 CSV，验证对应布局的帧 CRC，对每轮激励首尾各裁剪 3 秒。schema 5 会先剔除重复 IMU 时间戳，使用 IMU 接收时间间隔求导；旧版数据继续使用原设备时间。在缺帧、失效、时间回退处先分块，再进行对称滤波和加速度求导，避免跨缺口求导。默认平滑窗口为 5 点，两侧信号使用相同窗口。原始文件始终不改。

脚本保留限幅和软 PD 样本的真实总命令，不自动移动样本、不根据龄期补偿，也不强制 B 为正。若 B<0、Fc<0、J≤0 或 omega_s 达到搜索边界，会明确警告。**负 B 不能直接当作物理阻尼填入控制器**；先比较分频段 / 分幅值结果、力矩实际跟随情况、速度覆盖和残差，再判定时序或模型问题。

## 构建与验证

本机已使用 Arm Compiler 6.21 实际编译、链接通过：

```powershell
python tools/build.py --toolchain D:/ARM/ARMCLANG
```

产物为 `MDK-ARM/YawId/yaw_id.axf`、`yaw_id.hex`、`yaw_id.bin`。构建日志在 `build/armclang/`。也可用 Keil 工程目标 `YawId`，要求 Arm Compiler 6 和 C++17。工程移除了指向缺失旧任务的引用，保留现有 HAL / BSP 源文件。已有驱动的未使用字段、构造初始化顺序等编译警告仍存在。

```powershell
python tools/run_host_tests.py
python tests/make_fit_fixture.py
matlab -batch "run('tests/test_fit.m')"
```

主机验证覆盖启动联锁、超时和故障停机门控、行程限制、力矩量化、21 个激励阶段、完整运行、帧错位 / 损坏恢复、CSV 回放和序号回卷。MATLAB 合成数据验证 J/B/Fc 求解链路。它们不替代电机方向、实际行程、停机响应或实机拟合验证。**本次未烧录或运行电机。**

复用驱动只增加必要返回值 / 诊断：DJI `sendCAN()`、DM `ctrl_Mit/On/Off()` 返回发送结果；HI12 增加有效帧序号；DT7 `parseData()` 增加有效性校验与 bool 返回，线协议不变。CAN 发送修复了读取未初始化 mailbox 字段的问题；RTOS 断言和异常入口先请求停机。集成前核心工程文件备份位于 `tools/original_project_backup.zip`，精简通道前备份位于 `tools/backups/vofa_schema3_before_compact.zip`，添加 IMU 时间戳前备份位于 `tools/backups/vofa_schema4_before_imu_timestamps.zip`，不建议用整包覆盖方式回退其他工作。
