#pragma once
#include "main.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace Sentry::Time
{
/** @brief 返回 HAL 毫秒时钟，用于反馈新鲜度判断。 */
inline uint32_t now_ms()
{
    return HAL_GetTick();
}
inline uint32_t elapsed_ms(uint32_t since)
{
    return now_ms() - since;
}
inline bool expired(uint32_t since, uint32_t timeout)
{
    return elapsed_ms(since) >= timeout;
}
inline bool fresh(uint32_t stamp, uint32_t timeout)
{
    return stamp != 0U && !expired(stamp, timeout);
}
} // namespace Sentry::Time

namespace Sentry
{
// 角度、角速度、转矩和电流按成员后缀标注单位。
// 底盘 DJI 轮速为输出轴转速；摩擦轮转速为电机转子转速。
/** @brief 电机原始反馈及换算后的转子轴、输出轴状态。 */
struct MotorFeedback
{

    int16_t angle_enc;
    int16_t speed_rpm_raw;
    int16_t torque_raw;
    uint8_t temperature;
    uint8_t padding[2];
    uint8_t online;
    uint32_t timestamp;

    float rotor_angle_deg;
    float rotor_angle_rad;
    float rotor_speed_radps;
    float rotor_total_rad;

    float output_angle_deg;
    float output_angle_rad;
    float output_speed_rpm;
    float output_speed_radps;
    float total_angle_deg;
    float total_angle_rad;
    float out_torque_Nm;
    float motor_current_A;

    float angle_deg() const { return rotor_angle_deg; }

    float angle_deg_signed() const
    {
        return rotor_angle_deg > 180.0f ? rotor_angle_deg - 360.0f : rotor_angle_deg;
    }

    float angle_rad() const { return rotor_angle_rad; }

    float angle_rad_signed() const { return angle_deg_signed() * 0.01745329252f; }

    float speed_rpm() const { return output_speed_rpm; }

    float speed_radps() const { return output_speed_radps; }

    float speed_degps() const { return output_speed_rpm * 6.0f; }

    float torque_Nm() const { return out_torque_Nm; }

    float current_A() const { return motor_current_A; }

    float total_deg() const { return total_angle_deg; }

    uint8_t temp() const { return temperature; }

    static constexpr uint32_t DEFAULT_ONLINE_TIMEOUT_MS = 100U;

    bool is_online(uint32_t timeout_ms = DEFAULT_ONLINE_TIMEOUT_MS) const
    {
        return online != 0 && Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool is_offline(uint32_t timeout_ms = DEFAULT_ONLINE_TIMEOUT_MS) const
    {
        return !is_online(timeout_ms);
    }
};
/** @brief 电机电流命令；current 为 BSP 对应驱动的原始命令量。 */
struct MotorCommand
{
    int16_t current = 0;
    bool enable = true;
    bool reinitialize = false;
};
/** @brief DM 电机 MIT 命令；角度用 rad、速度用 rad/s、转矩用 N*m。 */
struct DMMITCommand
{
    float pos_rad;
    float vel_radps;
    float kp;
    float kd;
    float torque_nm;
    bool enable = true;
    bool reinitialize = false;
    uint32_t timestamp;
};
/** @brief DM 电机握手状态及最近一帧反馈信息。 */
struct DMMotorStatus
{
    uint8_t state = 0;
    uint8_t error_code = 0;
    uint16_t send_id = 0;
    uint16_t fb_id = 0;
    uint32_t fb_ts = 0;
    uint32_t tx_drop_count = 0;
    bool enabled = false;
    uint32_t timestamp = 0;
};
// HI12 原始角度与角速度使用 deg、deg/s；控制器读取后统一换算为 rad、rad/s。
/** @brief IMU 测量值；角度和陀螺仪数据保留传感器原生单位。 */
struct IMUData
{
    float acc_mss[3];
    float gyro_dps[3];
    float angle_deg[3];
    float quaternion[4];
    float temperature_c;
    uint32_t sensor_timestamp_ms;
    uint32_t timestamp;

    static constexpr uint32_t DEFAULT_ONLINE_TIMEOUT_MS = 100U;

    uint32_t age_ms() const
    {
        return timestamp == 0U ? 0xFFFFFFFFUL : Sentry::Time::elapsed_ms(timestamp);
    }

    bool is_online(uint32_t timeout_ms = DEFAULT_ONLINE_TIMEOUT_MS) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool is_offline(uint32_t timeout_ms = DEFAULT_ONLINE_TIMEOUT_MS) const
    {
        return !is_online(timeout_ms);
    }
};
#pragma pack(push, 1)
/** @brief DT7 遥控器通道、拨杆、鼠标和键盘数据。 */
struct RemoteData
{
    int16_t ch0, ch1, ch2, ch3;
    int16_t scroll;
    uint8_t s1, s2;
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
    float left_x, left_y, right_x, right_y;
    float scroll_norm;
    int16_t left_coord_x, left_coord_y;
    int16_t right_coord_x, right_coord_y;
    int16_t scroll_coord;
    uint32_t timestamp;

    static constexpr uint32_t CONNECTION_TIMEOUT_MS = 100;

    bool isConnected() const { return Sentry::Time::fresh(timestamp, CONNECTION_TIMEOUT_MS); }

    uint32_t age_ms() const
    {
        return timestamp == 0U ? 0xFFFFFFFFUL : Sentry::Time::elapsed_ms(timestamp);
    }
};
namespace RefereeSystem
{
struct RobotStatus
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    float bullet_speed_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};
struct PowerHeatData
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};
struct RadarSingleTargetV0
{
    uint8_t version;
    uint8_t msg_type;
    uint8_t seq;
    uint8_t enemy_robot_id;
    int16_t x_cm;
    int16_t y_cm;
    uint16_t age_ms;
    uint8_t confidence;
    uint8_t flags;
    uint32_t radar_time_ms;
};
inline constexpr uint8_t RADAR_SINGLE_TARGET_VALID = 1U;
inline constexpr uint8_t RADAR_SINGLE_TARGET_LOST = 2U;
} // namespace RefereeSystem
/** @brief 裁判系统状态、热量数据及接收统计。 */
struct RefereeData
{
    RefereeSystem::RobotStatus status;
    RefereeSystem::PowerHeatData power;
    bool is_online;
    uint32_t timestamp;
    uint32_t rx_frame_count;
    uint32_t robot_status_count;
    uint32_t power_heat_count;
    uint16_t last_cmd_id;
    uint16_t last_cmd_len;

    RefereeData()
        : status{}, power{}, is_online(false), timestamp(0), rx_frame_count(0),
          robot_status_count(0), power_heat_count(0), last_cmd_id(0), last_cmd_len(0)
    {
    }

    uint32_t age_ms() const
    {
        return timestamp == 0U ? 0xFFFFFFFFUL : Sentry::Time::elapsed_ms(timestamp);
    }

    bool online(uint32_t timeout_ms = 1000U) const
    {
        return is_online && Sentry::Time::fresh(timestamp, timeout_ms);
    }
};
/** @brief 裁判系统转发的雷达目标和收件信息。 */
struct RefereeRadarTarget
{
    RefereeSystem::RadarSingleTargetV0 target{};
    uint16_t sender_id{0};
    uint16_t receiver_id{0};
    uint32_t timestamp{0};
    uint32_t rx_count{0};

    uint32_t age_ms() const
    {
        return timestamp == 0U ? 0xFFFFFFFFUL : Sentry::Time::elapsed_ms(timestamp);
    }

    bool online(uint32_t timeout_ms = 500U) const
    {
        return timestamp != 0U && Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool has_ref() const { return (target.flags & RefereeSystem::RADAR_SINGLE_TARGET_VALID) != 0U; }
};
#pragma pack(pop)
} // namespace Sentry
static_assert(sizeof(Sentry::RemoteData) == 56);
static_assert(sizeof(Sentry::RefereeData) == 52);
static_assert(sizeof(Sentry::RefereeRadarTarget) == 28);

namespace App
{
namespace Chassis
{

/** @brief 主机下发的底盘速度及云台模式请求。 */
struct HostMotionRequest
{
    float vx_mps{0.0f};
    float vy_mps{0.0f};
    float wz_radps{0.0f};
    std::uint32_t timestamp{0U};

    std::uint8_t gimbal_mode{0U};
    std::uint8_t reserved_byte{0U};
    std::uint16_t reserved{0U};

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }
};

static_assert(sizeof(HostMotionRequest) == 20U,
              "HostMotionRequest is a 20-byte UARTBridge payload");
static_assert(offsetof(HostMotionRequest, timestamp) == 12U,
              "HostMotionRequest bridge field layout changed");
static_assert(offsetof(HostMotionRequest, gimbal_mode) == 16U,
              "HostMotionRequest mode offset changed");

enum class ChassisMotionSource : std::uint8_t
{
    Stop = 0U,
    Manual = 1U,
    Navigation = 2U,
    Keyboard = 3U,
};

enum ChassisMotionCommandFlag : std::uint8_t
{
    CHASSIS_CMD_REMOTE_FRESH = 1U << 0U,
    CHASSIS_CMD_HOST_REQUEST_FRESH = 1U << 1U,
    CHASSIS_CMD_NAVIGATION_SELECTED = 1U << 2U,
    CHASSIS_CMD_VALUE_LIMITED = 1U << 3U,
    CHASSIS_CMD_KEYBOARD_SELECTED = 1U << 4U,

    CHASSIS_CMD_GIMBAL_TRANSITION_LOCK = 1U << 5U,
};

enum class GimbalMotionGateState : std::uint8_t
{
    NORMAL = 0U,
    TO_TRANSFORM = 1U,
    TRANSFORM = 2U,
    TO_NORMAL = 3U,
    FAULT = 4U,
};

/** @brief 云台变形状态对底盘运动的许可信息。 */
struct GimbalMotionGate
{
    GimbalMotionGateState state{GimbalMotionGateState::NORMAL};
    std::uint8_t chassis_lock{0U};
    std::uint8_t tf_complete{0U};
    std::uint8_t reserved{0U};
    std::uint32_t timestamp{0U};

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool locks_chassis() const { return chassis_lock != 0U; }
};

static_assert(sizeof(GimbalMotionGate) == 8U,
              "GimbalMotionGate must remain one UART bridge payload");
static_assert(offsetof(GimbalMotionGate, timestamp) == 4U,
              "GimbalMotionGate timestamp offset changed");

/** @brief 底盘最终速度目标、来源和有效性标志。 */
struct ChassisMotionCommand
{
    float vx_mps{0.0f};
    float vy_mps{0.0f};
    float wz_radps{0.0f};
    std::uint32_t timestamp{0U};
    ChassisMotionSource source{ChassisMotionSource::Stop};
    std::uint8_t flags{0U};
    std::uint16_t reserved{0U};

    bool is_online(std::uint32_t timeout_ms = 50U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool has_known_source() const
    {
        return static_cast<std::uint8_t>(source) <=
               static_cast<std::uint8_t>(ChassisMotionSource::Keyboard);
    }
};

/** @brief 云台发送给底盘的航向角，单位 rad。 */
struct HeadingFeedback
{
    float yaw_rad{0.0f};
    std::uint32_t timestamp{0U};

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }
};

/** @brief 云台相对底盘 yaw 关节角反馈，单位 rad。 */
struct GimbalYawJointFeedback
{
    float yaw_angle_rad{0.0f};
    std::uint32_t timestamp{0U};

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }
};

static_assert(sizeof(HeadingFeedback) == 8U,
              "HeadingFeedback must remain one bridge payload (8 bytes)");
static_assert(offsetof(HeadingFeedback, timestamp) == 4U,
              "HeadingFeedback bridge field layout changed");
static_assert(sizeof(GimbalYawJointFeedback) == 8U,
              "GimbalYawJointFeedback must remain one bridge payload (8 bytes)");
static_assert(offsetof(GimbalYawJointFeedback, timestamp) == 4U,
              "GimbalYawJointFeedback bridge field layout changed");

enum ChassisOdometryFlag : std::uint8_t
{
    ODOM_INITIALIZED = 1U << 0U,
    ODOM_HEADING_FRESH = 1U << 1U,
    ODOM_WHEELS_FRESH = 1U << 2U,
    ODOM_KINEMATICS_VALID = 1U << 3U,
    ODOM_DT_VALID = 1U << 4U,
    ODOM_POSITION_INTEGRATED = 1U << 5U,
    ODOM_HEADING_JUMP = 1U << 6U,
    ODOM_STATIONARY_GATED = 1U << 7U,
};

/** @brief 底盘平面速度、位置和航向里程计。 */
struct ChassisOdometry
{
    float x_m{0.0f};
    float y_m{0.0f};
    float yaw_rad{0.0f};
    float vx_body_mps{0.0f};
    float vy_body_mps{0.0f};
    float wz_radps{0.0f};
    std::uint32_t timestamp{0U};
    std::uint32_t heading_timestamp{0U};
    std::uint8_t valid_wheel_count{0U};
    std::uint8_t flags{0U};

    std::uint8_t gimbal_motion_lock{0U};
    std::uint8_t reserved{0U};

    bool has_flag(ChassisOdometryFlag flag) const
    {
        return (flags & static_cast<std::uint8_t>(flag)) != 0U;
    }

    bool is_valid() const
    {
        constexpr std::uint8_t required =
            ODOM_INITIALIZED | ODOM_HEADING_FRESH | ODOM_WHEELS_FRESH | ODOM_KINEMATICS_VALID;
        return (flags & required) == required;
    }

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }
};

static_assert(sizeof(ChassisOdometry) == 36U, "ChassisOdometry bridge payload size changed");
static_assert(offsetof(ChassisOdometry, gimbal_motion_lock) == 34U,
              "ChassisOdometry gimbal lock offset changed");

} // namespace Chassis
} // namespace App

namespace App
{
namespace Vision
{

/** @brief 视觉主机下发的 yaw、pitch 参考角及速度、加速度。 */
struct VisionControlRequest
{
    float pitch_angle_rad{0.0f};
    float yaw_angle_rad{0.0f};
    float yaw_speed_radps{0.0f};
    float yaw_accel_radps2{0.0f};
    float pitch_speed_radps{0.0f};
    float pitch_accel_radps2{0.0f};
    std::uint8_t control_mode{0U};
    std::uint8_t vis_ready{0U};
    std::uint8_t fire{0U};
    std::uint16_t reserved{0U};
    std::uint32_t vis_time_ms{0U};
    std::uint8_t aim_x{0U};
    std::uint8_t aim_y{0U};
    std::uint16_t reserved2{0U};
    std::uint32_t timestamp{0U};

    bool is_online(std::uint32_t timeout_ms = 100U) const
    {
        return Sentry::Time::fresh(timestamp, timeout_ms);
    }

    bool has_ref() const { return vis_ready != 0U; }

    bool should_fire() const { return has_ref() && fire != 0U; }
};

} // namespace Vision
} // namespace App

namespace Sentry
{

/** @brief 通过短时关中断保护控制任务与中断间的结构体快照。 */
template <class T> class Snapshot
{
    T value_{};

  public:
    T read() const
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        T copy = value_;
        __set_PRIMASK(mask);
        return copy;
    }
    void publish(const T &value)
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        value_ = value;
        __set_PRIMASK(mask);
    }
};
struct RobotState
{
    Snapshot<Sentry::RemoteData> remote;
    Snapshot<App::Chassis::HostMotionRequest> host;
    Snapshot<App::Chassis::ChassisOdometry> odometry;
    Snapshot<App::Vision::VisionControlRequest> vision;
    Snapshot<Sentry::IMUData> main_imu;
    Snapshot<Sentry::IMUData> big_imu;
    Snapshot<App::Chassis::HeadingFeedback> heading;
    Snapshot<App::Chassis::GimbalYawJointFeedback> yaw_joint;
    Snapshot<App::Chassis::GimbalMotionGate> motion_gate;
    Snapshot<App::Chassis::ChassisMotionCommand> motion_command;
    Snapshot<Sentry::RefereeData> referee;
    Snapshot<Sentry::RefereeRadarTarget> radar;
    Snapshot<Sentry::MotorFeedback> base_fb;
    Snapshot<Sentry::MotorCommand> base_cmd;
    Snapshot<Sentry::DMMotorStatus> base_status;
    Snapshot<Sentry::DMMITCommand> base_mit;
    Snapshot<Sentry::MotorFeedback> yaw_fb;
    Snapshot<Sentry::MotorCommand> yaw_cmd;
    Snapshot<Sentry::MotorFeedback> pitch_fb;
    Snapshot<Sentry::MotorCommand> pitch_cmd;
    Snapshot<Sentry::DMMotorStatus> pitch_status;
    Snapshot<Sentry::DMMITCommand> pitch_mit;
    Snapshot<Sentry::MotorFeedback> roll_fb;
    Snapshot<Sentry::MotorCommand> roll_cmd;
    Snapshot<Sentry::DMMotorStatus> roll_status;
    Snapshot<Sentry::DMMITCommand> roll_mit;
    Snapshot<Sentry::MotorFeedback> fric_left_fb;
    Snapshot<Sentry::MotorCommand> fric_left_cmd;
    Snapshot<Sentry::MotorFeedback> fric_right_fb;
    Snapshot<Sentry::MotorCommand> fric_right_cmd;
    Snapshot<Sentry::MotorFeedback> fric_third_fb;
    Snapshot<Sentry::MotorCommand> fric_third_cmd;
    Snapshot<Sentry::MotorFeedback> dial_fb;
    Snapshot<Sentry::MotorCommand> dial_cmd;
    Snapshot<MotorFeedback> wheel_fb[4];
    Snapshot<MotorCommand> wheel_cmd[4];
};
} // namespace Sentry
extern Sentry::RobotState sentry_data;
