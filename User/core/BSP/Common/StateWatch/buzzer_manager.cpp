#include "buzzer_manager.hpp"

namespace BSP::WATCH_STATE
{
    // 获取单例实例
    BuzzerManagerSimple& BuzzerManagerSimple::getInstance()
    {
        static BuzzerManagerSimple instance;
        return instance;
    }
    
    BuzzerManagerSimple::BuzzerManagerSimple() {}
    
    void BuzzerManagerSimple::init()
    {
        ring_index_ = 0;
        last_ring_time_ = 0;
        is_ringing_ = false;
        current_melody_ = nullptr;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    }
    
    // 底层 PWM 频率控制
    void BuzzerManagerSimple::controlBuzzer(uint16_t freq_hz)
    {
        if (freq_hz == 0) 
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 占空比 0，静音
        } 
        else 
        {
            // 计算动态 ARR 值以改变音调
            uint32_t arr_value = TIM_BASE_CLOCK / freq_hz - 1;
            __HAL_TIM_SET_AUTORELOAD(&htim4, arr_value);
            // 保持 50% 占空比以获得最大音量和最佳波形
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arr_value / 5); 
        }
    }

    // ================== 阻塞式上电逻辑 ==================
    void BuzzerManagerSimple::playStartupMusic()
    {
        playStartupMusic(StartupPrompt::PowerOn);
    }

    void BuzzerManagerSimple::playStartupMusic(StartupPrompt prompt)
    {
        static const BuzzerNote power_on_melody[] = {
            {2093, 150}, {0, 50},  // Do (C7)
            {2349, 150}, {0, 50},  // Re (D7)
            {2637, 500}            // Mi (E7)
        };

        static const BuzzerNote ready_melody[] = {
            {NOTE_LOW, 80}, {0, 30},
            {NOTE_MID, 80}, {0, 30},
            {NOTE_HIGH, 180}
        };

        static const BuzzerNote confirm_melody[] = {
            {NOTE_MID, 80}, {0, 40},
            {NOTE_HIGH, 120}
        };

        static const BuzzerNote attention_melody[] = {
            {NOTE_HIGH, 120}, {0, 60},
            {NOTE_LOW, 180}
        };

        static const BuzzerNote error_melody[] = {
            {NOTE_MAX, 80}, {0, 40},
            {NOTE_MAX, 80}, {0, 40},
            {NOTE_LOW, 250}
        };

        const BuzzerNote* melody = power_on_melody;
        uint16_t length = sizeof(power_on_melody) / sizeof(BuzzerNote);
        uint16_t tail_delay_ms = 800;

        switch (prompt)
        {
            case StartupPrompt::Ready:
                melody = ready_melody;
                length = sizeof(ready_melody) / sizeof(BuzzerNote);
                tail_delay_ms = 300;
                break;

            case StartupPrompt::Confirm:
                melody = confirm_melody;
                length = sizeof(confirm_melody) / sizeof(BuzzerNote);
                tail_delay_ms = 200;
                break;

            case StartupPrompt::Attention:
                melody = attention_melody;
                length = sizeof(attention_melody) / sizeof(BuzzerNote);
                tail_delay_ms = 300;
                break;

            case StartupPrompt::Error:
                melody = error_melody;
                length = sizeof(error_melody) / sizeof(BuzzerNote);
                tail_delay_ms = 400;
                break;

            case StartupPrompt::PowerOn:
            default:
                break;
        }

        playBlockingMelody(melody, length, tail_delay_ms);
    }

    void BuzzerManagerSimple::playBlockingMelody(const BuzzerNote* melody, uint16_t length, uint16_t tail_delay_ms)
    {
        if (melody == nullptr || length == 0)
        {
            controlBuzzer(0);
            return;
        }

        for (uint16_t i = 0; i < length; i++) 
        {
            controlBuzzer(melody[i].frequency);
            osDelay(melody[i].duration); // 阻塞当前 RTOS 任务
        }
        controlBuzzer(0); // 彻底关闭

        if (tail_delay_ms > 0)
        {
            osDelay(tail_delay_ms);
        }
    }

    // ================== 请求入队逻辑 ==================
    void BuzzerManagerSimple::requestMotorRing(uint8_t motor_id) 
    {
        if (motor_id < 1 || motor_id > 8 || ring_index_ >= MAX_QUEUE_SIZE) return;
        // 防抖/防重复判断
        for (uint8_t i = 0; i < ring_index_; i++) {
            if (ring_queue_[i] == motor_id) return; 
        }
        ring_queue_[ring_index_++] = motor_id;
    }
    
    void BuzzerManagerSimple::requestRemoteRing() 
    {
        if (ring_index_ >= MAX_QUEUE_SIZE) return;
        for (uint8_t i = 0; i < ring_index_; i++) {
            if (ring_queue_[i] == 0xFF) return;
        }
        ring_queue_[ring_index_++] = 0xFF;
    }

    void BuzzerManagerSimple::requestCommunicationRing() 
    {
        if (ring_index_ >= MAX_QUEUE_SIZE) return;
        for (uint8_t i = 0; i < ring_index_; i++) {
            if (ring_queue_[i] == 0xFE) return;
        }
        ring_queue_[ring_index_++] = 0xFE;
    }

    void BuzzerManagerSimple::requestIMURing() 
    {
        if (ring_index_ >= MAX_QUEUE_SIZE) return;
        for (uint8_t i = 0; i < ring_index_; i++) {
            if (ring_queue_[i] == 0xFD) return;
        }
        ring_queue_[ring_index_++] = 0xFD;
    }

    // ================== 非阻塞状态机处理 ==================
    void BuzzerManagerSimple::startMelody(const BuzzerNote* melody, uint16_t length)
    {
        current_melody_ = melody;
        melody_length_ = length;
        current_note_index_ = 0;
        note_start_time_ = HAL_GetTick(); // 记录起播时间
        is_ringing_ = true;
        
        controlBuzzer(current_melody_[0].frequency); // 播放第一声
    }

    void BuzzerManagerSimple::update()
    {
        uint32_t current_time = HAL_GetTick();
        
        // 1. 处理正在播放的音符序列 (时间流逝检测)
        if (is_ringing_ && current_melody_ != nullptr)
        {
            // 如果当前音符播放时间达到设定值
            if (current_time - note_start_time_ >= current_melody_[current_note_index_].duration)
            {
                current_note_index_++; // 准备切换下一个音符
                
                if (current_note_index_ >= melody_length_) 
                {
                    // 旋律彻底结束
                    controlBuzzer(0); 
                    is_ringing_ = false;
                    current_melody_ = nullptr;
                    last_ring_time_ = current_time; // 记录结束时间，用于冷却计算
                } 
                else 
                {
                    // 播放下一个音符
                    controlBuzzer(current_melody_[current_note_index_].frequency); 
                    note_start_time_ = current_time;
                }
            }
            return; // 当前正在响铃时，拦截所有新请求的处理
        }
            
        // 2. 响铃间隔冷却 (防止不同报警音粘连)
        if (current_time - last_ring_time_ < RING_INTERVAL_MS) 
            return;
            
        // 3. 提取队列中最前方的请求并处理
        if (ring_index_ > 0)
        {
            uint8_t id = ring_queue_[0];
            // 队列前移
            for (uint8_t i = 1; i < ring_index_; i++) {
                ring_queue_[i-1] = ring_queue_[i];
            }
            ring_index_--;
            
            processRing(id); // 装载旋律
        }
    }

    void BuzzerManagerSimple::processRing(uint8_t id)
    {
        // 注意：静态局部数组，防止函数退出后指针悬挂
        // 遥控器报警音：低频升调
        static const BuzzerNote remote_melody[] = {
            {NOTE_LOW, 150}, {NOTE_MID, 150}, {0, 100}, {NOTE_LOW, 150}, {NOTE_MID, 150}
        };
        
        // 板间通信报警音：低频慢节奏双音
        static const BuzzerNote comm_melody[] = {
            {NOTE_LOW, 200}, {0, 100}, {NOTE_LOW, 200}
        };
        
        // IMU报警音：4000Hz 致命三连响 (声音最尖锐刺耳)
        static const BuzzerNote imu_melody[] = {
            {NOTE_MAX, 100}, {0, 50}, {NOTE_MAX, 100}, {0, 50}, {NOTE_MAX, 100}
        };
        
        if (id == 0xFF) 
        {
            startMelody(remote_melody, sizeof(remote_melody)/sizeof(BuzzerNote));
        }
        else if (id == 0xFE) 
        {
            startMelody(comm_melody, sizeof(comm_melody)/sizeof(BuzzerNote));
        }
        else if (id == 0xFD) 
        {
            startMelody(imu_melody, sizeof(imu_melody)/sizeof(BuzzerNote));
        }
        else if (id >= 1 && id <= 8) 
        { 
            // 动态生成电机响铃次数存入全局 buffer
            melody_length_ = id * 2; 
            for (uint8_t i = 0; i < id; i++) {
                dynamic_melody_buffer_[i * 2] = {NOTE_HIGH, 100};  // 滴
                dynamic_melody_buffer_[i * 2 + 1] = {0, 100};      // 停顿
            }
            startMelody(dynamic_melody_buffer_, melody_length_);
        }
    }
}
