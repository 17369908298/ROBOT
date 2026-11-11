#pragma once

#include <cstdint>

namespace App::Onmi
{

/** @brief 云台进入变形形态及恢复过程中的各阶段。 */
enum class TransformState : std::uint8_t
{
    // 普通模式与变形模式之间必须经过对齐和 roll 轴切换阶段。
    NORMAL = 0U,

    // 先对齐底盘、yaw、pitch，再切换 roll 轴。
    TO_TRANSFORM_ALIGN,

    TO_TRANSFORM_ROLL,

    TRANSFORM,

    // 恢复时先同步参考角，再沿轨迹对齐并切回 roll 轴。
    TO_NORMAL_SYNC,

    TO_NORMAL_ALIGN,

    TO_NORMAL_ROLL,

    FAULT,
};

/** @brief 管理变形对齐、roll 轴切换、恢复同步和故障锁存。 */
class TransformFSM final
{
  public:
    constexpr void reset()
    {

        state_ = TransformState::NORMAL;
        previous_state_ = state_;
    }

    constexpr void force_fault()
    {

        previous_state_ = state_;
        state_ = TransformState::FAULT;
    }

    /** @brief 根据请求及轴到位条件推进一个状态。 */
    constexpr void update(bool tf_requested, bool normal_recovery_requested, bool tf_axes_reached,
                          bool tf_roll_reached, bool normal_sync_done, bool normal_recovery_ready,
                          bool normal_roll_reached)
    {
        previous_state_ = state_;

        switch (state_)
        {
        case TransformState::NORMAL:

            if (tf_requested)
            {
                state_ = TransformState::TO_TRANSFORM_ALIGN;
            }
            else if (normal_recovery_requested)
            {
                state_ = TransformState::TO_NORMAL_SYNC;
            }
            break;

        case TransformState::TO_TRANSFORM_ALIGN:

            if (!tf_requested)
            {
                state_ = TransformState::TO_NORMAL_SYNC;
            }
            else if (tf_axes_reached)
            {
                state_ = TransformState::TO_TRANSFORM_ROLL;
            }
            break;

        case TransformState::TO_TRANSFORM_ROLL:

            if (!tf_requested)
            {
                state_ = TransformState::TO_NORMAL_SYNC;
            }
            else if (tf_roll_reached)
            {
                state_ = TransformState::TRANSFORM;
            }
            break;

        case TransformState::TRANSFORM:

            if (!tf_requested)
            {
                state_ = TransformState::TO_NORMAL_SYNC;
            }
            break;

        case TransformState::TO_NORMAL_SYNC:

            if (normal_sync_done)
            {
                state_ = TransformState::TO_NORMAL_ALIGN;
            }
            break;

        case TransformState::TO_NORMAL_ALIGN:

            if (normal_recovery_ready)
            {
                state_ = TransformState::TO_NORMAL_ROLL;
            }
            break;

        case TransformState::TO_NORMAL_ROLL:

            if (normal_roll_reached)
            {
                state_ = TransformState::NORMAL;
            }
            break;

        case TransformState::FAULT:

            break;
        }
    }

    constexpr TransformState state() const { return state_; }
    constexpr TransformState previous_state() const { return previous_state_; }
    constexpr bool changed() const { return state_ != previous_state_; }
    constexpr bool external_control_allowed() const { return state_ == TransformState::NORMAL; }
    constexpr bool owns_control() const { return !external_control_allowed(); }
    constexpr bool faulted() const { return state_ == TransformState::FAULT; }

  private:
    TransformState state_{TransformState::NORMAL};
    TransformState previous_state_{TransformState::NORMAL};
};

} // namespace App::Onmi

#include <cstdint>

namespace App::Onmi
{

/** @brief 记录变形控制权及其来源。 */
class GimbalModeRequest final
{
  public:
    constexpr bool update(bool remote_tf, bool automatic_selected, bool host_fresh,
                          std::uint8_t host_mode)
    {
        if (remote_tf || !automatic_selected)
        {
            host_ready_ = false;
            host_mode_ = 0U;
            return remote_tf;
        }
        if (host_fresh && host_mode <= 1U)
        {
            host_mode_ = host_mode;
            host_ready_ = true;
        }
        return host_ready_ && host_mode_ == 1U;
    }

  private:
    std::uint8_t host_mode_{0U};
    bool host_ready_{false};
};

} // namespace App::Onmi
