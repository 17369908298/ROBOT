
# 全向云台

## 云台电机（can2）

| 电机型号   |   ID（上位机查看/设置）    |    位置    |  控制模式     |    闭环方式  |   量纲    |
|   :---:        |   :---:            |  :---:          | :---:    |   :---: | :---:  |     
|DM-J4310        |      2/0X01        |      pitch_2    |   MIT    | 外置陀螺仪角度环pid+内置速度环给kd |   上位机可查/-7/7    |

|DM-J4310        |      6/0X05        |        yaw      |   MIT    |  陀螺仪角速度（rad/s）单环adrc          | 上位机/ |
|DM-J4340        |      4/0x03         |     pitch_1    |  MIT     |   内置角度环（传入目标角度（rad），给定kp） |  上位机/-9~9     |
|LK4005          |       0X140/0X01     |    拨盘电机     |   扭矩模式0xA1 |  角度+速度双环PID |  力矩控（0xA1）/-2048~2048   |

|                |                |
|    :---:        |   :---:        |
|电机手册         |      [查看手册](DM-J4310-2EC V1.2减速电机说明书初稿最新.pdf)         |

### DM电机控制代码

```cpp


/**
 * @brief 达妙 (DM) 电机状态机处理函数
 * * @tparam MotorType 模板参数，支持不同类型的电机驱动对象
 * @param motor         电机对象引用
 * @param id            电机ID(非Can ID)
 * @param step          状态机当前步骤变量（需在外部持久保存，如静态变量或类成员）
 * @param target_enable 云台状态机是否在STOP状态 ：true 为使能，false 为失能
 * @param pos           MIT模式：目标位置 (rad)
 * @param vel           MIT模式：目标速度 (rad/s)
 * @param kp            MIT模式：位置增益
 * @param kd            MIT模式：速度增益
 * @param torq          MIT模式：馈通扭矩 (Nm)
 * * @note 关键逻辑：
 * 1. 状态跳转依赖于 motor.getError(id) 的返回值。通常 0 为失能/待机，1 为正常使能，>1 为各类硬件故障。
 * 2. 只有在满足“已使能”且“无错误”的情况下，才会执行真正的 ctrl_Mit 控制帧发送。
 */
template <typename MotorType>
static void handle_dm_motor_state_machine(MotorType& motor, uint8_t id, uint8_t &step, bool target_enable, float pos, float vel, float kp, float kd, float torq)
{
    // --- 情况 A：目标是使能电机并运行 ---
    if (target_enable)
    {
        /* 快速路径：如果当前电机已经处于使能状态且没有错误，直接执行控制 */
        if (motor.getIsenable(id) && motor.getError(id) == 1)
        {
            step = 0; // 重置步骤记录，确保下次状态切换从头开始
            motor.ctrl_Mit(id, pos, vel, kp, kd, torq); // 发送控制指令
            return;
        }

        /* 状态机：逐步执行使能流程 (ClearErr -> On -> Verify) */
        switch (step)
        {
            case 0: // 第一步：发送清除错误指令
                motor.ClearErr(id, BSP::Motor::DM::MIT);
                step = 1;
                break;

            case 1: // 第二步：等待错误清除，确认进入失能/待机状态 (0)
                if (motor.getError(id) == 0) 
                {
                    motor.On(id, BSP::Motor::DM::MIT); // 发送使能指令
                    step = 2;
                }
                break;

            case 2: // 第三步：确认进入使能状态 (1)
                if (motor.getError(id) == 1) 
                {
                    motor.setIsenable(id, true); // 更新内部软件标志位
                    step = 0; // 完成流程
                }
                else if (motor.getError(id) > 1) // 如果在此过程中发生故障
                {
                    step = 0; // 重启使能流程
                }
                break;

            default: 
                step = 0; 
                break;
        }
    }
    // --- 情况 B：目标是失能电机（安全停止） ---
    else
    {
        /* 快速路径：如果已经处于失能状态且无报错 */
        if (!motor.getIsenable(id) && motor.getError(id) == 0)
        {
            step = 0;
            // 依然发送空控制帧，目的是为了触发电机的CAN回传，以便持续获取位置、速度等反馈数据
            motor.ctrl_Mit(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); 
            return;
        }

        /* 状态机：逐步执行失能流程 (Off -> ClearErr -> Verify) */
        switch (step)
        {
            case 0: // 第一步：发送失能指令
                motor.Off(id, BSP::Motor::DM::MIT);
                step = 1;
                break;

            case 1: // 第二步：发送清错指令，确保彻底停止并清除潜在锁死
                motor.ClearErr(id, BSP::Motor::DM::MIT);
                step = 2;
                break;

            case 2: // 第三步：确认电机已回到失能状态 (0)
                if (motor.getError(id) == 0) 
                {
                    motor.setIsenable(id, false); // 更新内部软件标志位
                    step = 0;
                }
                else if (motor.getError(id) > 1) // 依然存在错误则尝试重置
                {
                    step = 0; 
                }
                break;

            default: 
                step = 0; 
                break;
        }
    }
}

```
**控制示例代码**
```cpp
/**
 * @brief 电机控制函数
 * 
 * 根据控制系统的输出结果向各电机发送控制指令
 */
static void motor_control()
{
    static uint8_t send_seq = 0;
    send_seq++;


    static uint8_t re_enable_step_pitch = 0;
    static uint8_t re_enable_step_yaw = 0;
    static uint8_t re_enable_step_pitch1 = 0;
 

    bool is_gimbal_active = (gimbal_fsm.Get_Now_State() != STOP);

    if (send_seq % 3 == 0) // 第三次
    {
        // can2 (Pitch2)
        handle_dm_motor_state_machine(MotorJ4310, 
                                        1, 
                                        re_enable_step_pitch, 
                                        is_gimbal_active,
                                        0.0f, 
                                        gimbal_output.out_pitch2, 
                                        0.0f, 
			                            0.5f, 
                                        0.0f);

    }
}

```





### 陀螺仪
**超核电子HI12** 
数据即拿即用，内置滤波。



# 全向底盘

## 底盘电机（can1）

| 电机型号 |  ID  |    控制方式          |  量纲  |
| :---:          |   :---:  | :---:    | :---: |
| 4x DJI3508     |    0X200/1/2/3/4    |     速度环pid(最好加点前馈)    | -16384~16384  |



## 底盘状态


### 不跟随

**底盘坐标系以云台坐标系为基准**

<iframe src="./航向转换.html" width="100%" height="600px" frameborder="0"></iframe>


1. 获取yaw轴电机角度，经过下面计算 ，**STANDARD基准角度（为云台坐标系与底盘坐标系重合时的yaw电机角度）**
```cpp
/**
 * @brief 底盘相对云台坐标系的平移解算（二维旋转矩阵）
 * * @param theta 云台当前反馈的 Yaw 轴角度（单位：弧度）
 * @param vx    笛卡尔坐标系下的 X 方向输入（通常对应遥控器的左右摇杆）
 * @param vy    笛卡尔坐标系下的 Y 方向输入（通常对应遥控器的前后摇杆）
 * @param phi   角度偏差补偿值（单位：弧度，用于微调或者机械安装误差补偿）
 * @param out_vx [输出指针] 计算旋转矩阵后，底盘坐标系下的 X 方向速度（前后）
 * @param out_vy [输出指针] 计算旋转矩阵后，底盘坐标系下的 Y 方向速度（左右）
 
 * @param psi   额外的相位角补偿（单位：弧度，用于特殊机动如小陀螺等）
 */
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi)
{
    // 将云台反馈角减去正前方的零位基准角，再叠加额外的补偿角
    theta = theta - STANDARD + phi;

    // 过零处理 (将角度归一化限制在 -PI 到 PI 之间)
    theta = fmod(theta, 2 * PI_);
    if (theta > PI_) theta -= 2 * PI_;
    else if (theta < -PI_) theta += 2 * PI_;

    // 预计算正弦和余弦值，包含额外的 psi 相位补偿
    float s = sinf(theta + psi);
    float c = cosf(theta + psi);
    
    // 遥控器控制量输入 (云台坐标系下)
    // 放大系数 4.5f，且注意遥控器的映射：vy(前后摇杆)对应机器人的X轴(前后)
    float raw_vx = 1.5f * vy;  // raw_vx是机器人云台坐标系下的前后方向
    float raw_vy = -1.5f * vx; // 横移方向取反，修正在云台坐标系下左右与遥控输入相反的问题
    
    // 核心：二维旋转矩阵计算
    // 将云台坐标系下的期望速度，通过旋转矩阵投影到底盘自身的局部坐标系下
    // [ cos(θ)  -sin(θ) ] * [ raw_vx ]
    // [ sin(θ)   cos(θ) ]   [ raw_vy ]
    *out_vx = raw_vx * c - raw_vy * s; 
    *out_vy = raw_vx * s + raw_vy * c;
}
```


### 跟随
**底盘坐标系始终跟随云台坐标系（云台旋转底盘跟随旋转）**
```cpp
/**
 * @brief 底盘跟随云台的角速度(w)规划
 * * 计算底盘偏航角(Yaw)与云台相对基准角度(STANDARD)之间的误差，
 * 并通过 PID 算法输出一个用于底盘旋转的角速度，使底盘始终朝向云台的正面。
 */
void CalculateFollow()
{
    // 计算跟随误差：标准基准角度(云台相对底盘的正前方编码器值) - 当前云台的实际反馈偏航角
    float follow_error = STANDARD - Cboard.GetYawAngle(); 
    
    // 角度归一化处理 (将误差限制在 -PI/2 到 PI/2，即 -90度 到 +90度之间)
    // 注意：这里的 1.5707963267f 就是 PI / 2
    // 如果误差超过 90 度，代码将其反转，这通常意味着底盘可以通过反向转动更近地到达目标，
    // 或者机械结构上允许底盘正反都能算作“跟随”（比如轮式步兵底盘前后对称）。
    while (follow_error > 1.5707963267f) follow_error -= 2 * 1.5707963267f;
    while (follow_error < -1.5707963267f) follow_error += 2 * 1.5707963267f;

    // 死区处理 (防止极小误差引起的底盘微小高频震荡，通常称为“点头”或“发抖”)
    // 0.01 弧度大约等于 0.57 度
    if(fabs(follow_error) < 0.01f) follow_error = 0.0f;
    
    // 将处理后的误差送入跟随 PID 控制器计算
    // 目标值为 0.0f（即期望误差为0），当前值为算出的误差量 follow_error
    // 返回值通常会赋值给底盘的期望角速度 target_vw
    follow_pid.UpDate(0.0f, follow_error);
}
```
获取旋转值（w），传入底盘解算

### 小陀螺



# 板间通信（串口）
|板子   |   uart    |
| :---:  | :---:    |
|  C板      |  uart6    |
|   A板     |   uart8   |
基础功能只需从云台获取遥控器和yaw轴电机角度
### 发送端
```cpp

#include "CommunicationTask.hpp"
 #include "../User/Task/MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp" // 为了使用 DT7Rx_buffer


// 强制 1 字节对齐，防止 float 产生空隙
#pragma pack(1)
typedef struct {
    uint8_t header;        // 帧头 0x5A
    uint8_t dt7_raw[18];   // 遥控器 18 字节
    float motor_angle;     // 电机角度 4 字节
    uint8_t checksum;      // 校验和 1 字节
} BoardPacket_t;
#pragma pack()
BoardPacket_t TxPacket;

void BoardCommunicationTX()
{
    // 1. 获取硬件实例 (UART6)
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    
    // 2. 获取数据：电机角度（假设从你的 Motor 库获取）
    float angle = MotorJ4310.getAngleRad(2); 

    // 3. 填充结构体
    TxPacket.header = 0x5A;
    memcpy(TxPacket.dt7_raw, DT7Rx_buffer, 18);
    TxPacket.motor_angle = angle;

    // 4. 计算和校验
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t *)&TxPacket;
    for (size_t i = 0; i < sizeof(BoardPacket_t) - 1; i++) {
        sum += ptr[i];
    }
    TxPacket.checksum = sum;

    // 5. 调用发送
    HAL::UART::Data uart6_tx_buffer{(uint8_t*)&TxPacket, sizeof(BoardPacket_t)};
    uart6.transmit_dma(uart6_tx_buffer);
}

```

### 接收端

```cpp
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include <cstdint>
#include <cstring>

// 协议总长度：1(帧头) + 18(DT7) + 4(电机角度) + 1(校验和) = 24字节
#define BOARD_PACKET_SIZE 24

// 开辟双倍缓冲区，必定能容纳至少一帧完整的错位包
extern uint8_t BoardRx[BOARD_PACKET_SIZE * 2];

class BoardCommunication
{
public:
    BoardCommunication() : yaw_angle(0.0f) 
    {
        memset(dt7_raw_data, 0, 18);
    }

    // 【核心修复2】：滑动窗口扫描解析
    void ParseProtocol(const uint8_t* buffer, uint16_t size) 
    {
        // 扫包：从头开始找 0x5A，并且保证后面有足够的长度
        for(int i = 0; i <= size - BOARD_PACKET_SIZE; i++)
        {
            if(buffer[i] == 0x5A) // 找到了潜在的包头
            {
                uint8_t sum = 0;
                // 计算除了校验位之外的 23 个字节的和
                for(int j = 0; j < BOARD_PACKET_SIZE - 1; j++) {
                    sum += buffer[i + j];
                }

                // 比对校验位
                if(sum == buffer[i + BOARD_PACKET_SIZE - 1]) 
                {
                    // 校验成功！提取数据
                    memcpy(dt7_raw_data, &buffer[i + 1], 18);
                    memcpy(&yaw_angle, &buffer[i + 19], sizeof(float));
                    
                    //可选：如果你需要直接把遥控器数据喂给底层的 DT7 解析器
                    extern BSP::REMOTE_CONTROL::RemoteController DT7;
                    DT7.parseData(dt7_raw_data);

                    return; // 成功解析一包，直接退出
                }
            }
        }
    }

    float GetYawAngle() const { return yaw_angle; }//获取yaw电机角度
    const uint8_t* GetRemoteDT7Data() const { return dt7_raw_data; }

private:
    float yaw_angle;     
    uint8_t dt7_raw_data[18];   
};

extern BoardCommunication Cboard;

```
