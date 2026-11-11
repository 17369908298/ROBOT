#pragma once
#include "communication/robot_data.hpp"
#include "user/core/BSP/Motor/DM/DmMotor.hpp"

/** @brief DM 电机掉线、清错、使能和重试状态。 */
enum class DMMotorState : uint8_t
{
    DISARMED = 0,
    DISABLING,
    IDLE,
    CLEARING_ERROR,
    WAITING_ENABLE,
    WAITING_CONFIRM,
    ENABLED,
    ERROR
};
/** @brief 通过电机 BSP 命令管理恢复，每轮最多发送一帧。 */
template <class Motor> class Class_Motor_Recovery_FSM
{
    Motor &motor_;
    bool enable_requested_ = false, has_received_fb_ = false, sent_ = false;
    DMMotorState dm_state_ = DMMotorState::DISARMED;
    struct
    {
        uint8_t error_code = 0;
    } dm_fb_;
    uint32_t state_enter_time_ = 0, retry_count_ = 0, clear_error_retry_count_ = 0,
             last_clear_error_tx_ms_ = 0, last_fb_time_ = 0, last_clear_error_diag_ms_ = 0;
    static constexpr uint32_t FEEDBACK_TIMEOUT_MS = 100, CLEAR_ERROR_RETRY_INTERVAL_MS = 50,
                              CLEAR_ERROR_MAX_RETRIES = 100;
    uint32_t now_ms() const { return HAL_GetTick(); }
    void switch_dm_state(DMMotorState next)
    {
        if (next != dm_state_)
        {
            retry_count_ = 0;
            state_enter_time_ = now_ms();
            if (next == DMMotorState::CLEARING_ERROR)
            {
                clear_error_retry_count_ = 0;
                last_clear_error_tx_ms_ = 0;
            }
        }
        dm_state_ = next;
    }
    void disable()
    {
        if (!sent_)
        {
            motor_.Off(1, BSP::Motor::DM::MIT);
            sent_ = true;
        }
    }
    void enable()
    {
        if (!sent_)
        {
            motor_.On(1, BSP::Motor::DM::MIT);
            sent_ = true;
        }
    }
    void clear_error()
    {
        if (!sent_)
        {
            motor_.ClearErr(1, BSP::Motor::DM::MIT);
            sent_ = true;
        }
    }
    bool request_clear_error(uint32_t now)
    {
        if (sent_ || (last_clear_error_tx_ms_ != 0 &&
                      uint32_t(now - last_clear_error_tx_ms_) < CLEAR_ERROR_RETRY_INTERVAL_MS))
            return false;
        clear_error();
        last_clear_error_tx_ms_ = now;
        ++clear_error_retry_count_;
        return true;
    }
    void run_state_machine()
    {
        uint32_t now = now_ms();
        uint32_t elapsed = now - state_enter_time_;

        uint32_t fb_age = now - last_fb_time_;
        bool fb_valid = fb_age < FEEDBACK_TIMEOUT_MS;

        if (!fb_valid)
        {
            has_received_fb_ = false;
        }

        bool enable_req = enable_requested_;
        if (!enable_req && dm_state_ != DMMotorState::DISARMED &&
            dm_state_ != DMMotorState::DISABLING)
        {

            disable();
            switch_dm_state(DMMotorState::DISABLING);
        }

        switch (dm_state_)
        {
        case DMMotorState::DISARMED:

            if (enable_req)
            {
                switch_dm_state(DMMotorState::CLEARING_ERROR);
                request_clear_error(now);
            }
            break;

        case DMMotorState::DISABLING:
            if (has_received_fb_)
            {
                if (dm_fb_.error_code == 0)
                {

                    switch_dm_state(DMMotorState::DISARMED);
                }
                else if (elapsed > 10)
                {

                    if (dm_fb_.error_code > 1)
                    {

                        clear_error();
                        disable();
                    }
                    else if (dm_fb_.error_code == 1)
                    {

                        disable();
                    }
                    retry_count_++;
                    state_enter_time_ = now;
                }
            }
            else if (elapsed > 50)
            {

                disable();
                retry_count_++;
                state_enter_time_ = now;
            }

            if (retry_count_ > 500)
            {

                switch_dm_state(DMMotorState::DISARMED);
            }
            break;

        case DMMotorState::IDLE:
            switch_dm_state(DMMotorState::CLEARING_ERROR);
            request_clear_error(now);
            break;

        case DMMotorState::CLEARING_ERROR:
            if (fb_valid && has_received_fb_)
            {
                if (dm_fb_.error_code == 0)
                {
                    enable();
                    switch_dm_state(DMMotorState::WAITING_CONFIRM);
                }
                else if (dm_fb_.error_code == 1)
                {

                    switch_dm_state(DMMotorState::WAITING_CONFIRM);
                }
            }

            if ((!fb_valid || !has_received_fb_) || (dm_fb_.error_code > 1U))
            {
                if (request_clear_error(now) &&
                    (last_clear_error_diag_ms_ == 0U || (now - last_clear_error_diag_ms_) >= 500U))
                {
                    last_clear_error_diag_ms_ = now;
                }

                if (clear_error_retry_count_ >= CLEAR_ERROR_MAX_RETRIES)
                {
                    const uint32_t failed_retries = clear_error_retry_count_;
                    (void)failed_retries;
                    switch_dm_state(DMMotorState::ERROR);
                }
            }
            break;

        case DMMotorState::WAITING_CONFIRM:
            if (fb_valid && has_received_fb_ && dm_fb_.error_code == 1)
            {
                switch_dm_state(DMMotorState::ENABLED);
            }
            else if (fb_valid && has_received_fb_ && dm_fb_.error_code > 1)
            {
                switch_dm_state(DMMotorState::CLEARING_ERROR);
                request_clear_error(now);
            }
            else if (elapsed > 100)
            {
                retry_count_++;
                if (retry_count_ <= 3)
                {
                    enable();
                }
                else
                {
                    switch_dm_state(DMMotorState::CLEARING_ERROR);
                    request_clear_error(now);
                }
                state_enter_time_ = now;
            }
            break;

        case DMMotorState::WAITING_ENABLE:
            break;

        case DMMotorState::ENABLED:

            if (!fb_valid)
            {

                switch_dm_state(DMMotorState::DISABLING);
                disable();
            }
            else if (has_received_fb_ && dm_fb_.error_code != 1)
            {
                switch_dm_state(DMMotorState::CLEARING_ERROR);
                request_clear_error(now);
            }
            break;

        case DMMotorState::ERROR:

            if (!enable_req)
            {
                switch_dm_state(DMMotorState::DISARMED);
            }
            else if (fb_valid && has_received_fb_)
            {

                switch_dm_state(DMMotorState::CLEARING_ERROR);
                request_clear_error(now);
            }
            break;
        }
    }

  public:
    explicit Class_Motor_Recovery_FSM(Motor &motor) : motor_(motor) {}
    Sentry::DMMotorStatus Update(const Sentry::DMMITCommand &command, uint16_t send_id,
                                 uint16_t fb_id)
    {
        const auto now = now_ms();
        sent_ = false;
        enable_requested_ = command.enable && Sentry::Time::fresh(command.timestamp, 50);
        last_fb_time_ = motor_.getFeedbackTick(1);
        has_received_fb_ = motor_.hasFeedback(1);
        dm_fb_.error_code = motor_.getError(1);
        if (command.reinitialize && dm_state_ == DMMotorState::ENABLED)
        {
            disable();
            switch_dm_state(DMMotorState::DISABLING);
        }
        run_state_machine();
        const bool enabled = dm_state_ == DMMotorState::ENABLED && has_received_fb_ &&
                             Sentry::Time::fresh(last_fb_time_, FEEDBACK_TIMEOUT_MS) &&
                             dm_fb_.error_code == 1;
        if (!sent_)
        {
            if (enabled && enable_requested_)
                motor_.ctrl_Mit(1, command.pos_rad, command.vel_radps, command.kp, command.kd,
                                command.torque_nm);
            else
                motor_.ctrl_Mit(1, 0, 0, 0, 0, 0);
        }
        Sentry::DMMotorStatus status{};
        status.state = uint8_t(dm_state_);
        status.error_code = dm_fb_.error_code;
        status.send_id = send_id;
        status.fb_id = fb_id;
        status.fb_ts = last_fb_time_;
        status.enabled = enabled;
        status.timestamp = now;
        return status;
    }
};
