#pragma once

#include <cstdint>
#include <type_traits>

namespace ALG
{
namespace Omni
{

/** @brief 记录当前、上一状态及本周期状态变化的通用有限状态机。 */
template <typename State, uint8_t MaxStates = 16> class FSM
{

    static_assert(std::is_enum<State>::value, "FSM State must be an enum type");

    static_assert(MaxStates > 0U, "FSM MaxStates must be greater than zero");

  public:
    void init(State initial, uint32_t now_ms = 0U)
    {
        uint8_t index = 0U;
        if (!state_index(initial, index))
        {
            return;
        }

        reset_statistics();
        current_state_ = initial;
        previous_state_ = initial;
        entered_at_ms_ = now_ms;
        enter_counts_[index] = 1U;
        changed_ = false;
        ready_ = true;
    }

    void update(State next, uint32_t now_ms)
    {
        changed_ = false;

        uint8_t index = 0U;
        if (!state_index(next, index))
        {
            return;
        }
        if (!ready_)
        {
            init(next, now_ms);
            return;
        }
        if (next == current_state_)
        {
            return;
        }

        previous_state_ = current_state_;
        current_state_ = next;
        entered_at_ms_ = now_ms;
        ++enter_counts_[index];
        changed_ = true;
    }

    void force(State next, uint32_t now_ms = 0U)
    {
        uint8_t index = 0U;
        if (!state_index(next, index))
        {
            changed_ = false;
            return;
        }
        if (!ready_)
        {
            init(next, now_ms);
            return;
        }

        previous_state_ = current_state_;
        current_state_ = next;
        entered_at_ms_ = now_ms;
        ++enter_counts_[index];
        changed_ = true;
    }

    State state() const { return current_state_; }

    State previous_state() const { return previous_state_; }

    bool changed() const { return changed_; }

    bool just_entered(State expected) const { return changed_ && current_state_ == expected; }

    uint32_t elapsed_ms(uint32_t now_ms) const { return ready_ ? (now_ms - entered_at_ms_) : 0U; }

    uint32_t enter_count(State state_value) const
    {
        uint8_t index = 0U;
        return state_index(state_value, index) ? enter_counts_[index] : 0U;
    }

    void reset_statistics()
    {
        for (uint8_t i = 0U; i < MaxStates; ++i)
        {
            enter_counts_[i] = 0U;
        }
    }

  private:
    using Underlying = typename std::underlying_type<State>::type;

    static bool state_index(State state_value, uint8_t &index)
    {
        const Underlying raw = static_cast<Underlying>(state_value);
        if constexpr (std::is_signed<Underlying>::value)
        {
            if (raw < 0)
            {
                return false;
            }
        }

        const uint64_t value = static_cast<uint64_t>(raw);
        if (value >= MaxStates)
        {
            return false;
        }

        index = static_cast<uint8_t>(value);
        return true;
    }

    State current_state_{};
    State previous_state_{};
    uint32_t entered_at_ms_ = 0U;
    uint32_t enter_counts_[MaxStates]{};
    bool changed_ = false;
    bool ready_ = false;
};

} // namespace Omni
} // namespace ALG
