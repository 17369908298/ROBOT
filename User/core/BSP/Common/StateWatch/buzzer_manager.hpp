#ifndef BUZZER_MANAGER_HPP
#define BUZZER_MANAGER_HPP

#include "cmsis_os.h"
#include "tim.h"      // 包含 htim4 句柄
#include <stdint.h>

namespace BSP::WATCH_STATE
{
    /**
     * @brief 蜂鸣器管理器类 (单例模式)
     * * 针对无源贴片蜂鸣器设计。
     * 支持上电时阻塞播放开机音效，以及在主循环中非阻塞异步播放队列中的报警提示音。
     */
    class BuzzerManagerSimple
    {
    public:
        static BuzzerManagerSimple& getInstance();

        enum class StartupPrompt : uint8_t
        {
            PowerOn = 0,    // 上电提示，默认开机三音
            Ready,          // 自检完成 / 系统就绪
            Confirm,        // 启动步骤确认
            Attention,      // 需要注意但非严重错误
            Error           // 启动失败 / 严重异常
        };
        
        // ---------------- 基础与阻塞功能 ----------------
        /**
         * @brief 初始化蜂鸣器管理器状态
         */
        void init();
        
        /**
         * @brief 播放默认上电启动提示音（阻塞式，兼容旧接口）
         * @note 必须在进入 RTOS 任务的 while(1) 主控制循环之前调用
         */
        void playStartupMusic();

        /**
         * @brief 播放指定启动提示音（阻塞式）
         * @param prompt 启动提示音类型
         * @note 必须在进入 RTOS 任务的 while(1) 主控制循环之前调用
         */
        void playStartupMusic(StartupPrompt prompt);

        // ---------------- 非阻塞报警功能 ----------------
        /**
         * @brief 请求电机响铃 (滴滴声，次数与 ID 一致)
         * @param motor_id 电机ID，有效值为 1-8
         */
        void requestMotorRing(uint8_t motor_id);
        
        /**
         * @brief 请求遥控器报警响铃 (升调)
         */
        void requestRemoteRing();
        
        /**
         * @brief 请求板间通讯报警响铃 (低沉双音)
         */
        void requestCommunicationRing();
        
        /**
         * @brief 请求陀螺仪异常报警响铃 (急促最高频)
         */
        void requestIMURing();
        
        /**
         * @brief 状态机更新函数
         * @note 必须在主循环 while(1) 中高频调用，绝对非阻塞
         */
        void update();
        
    private:
        BuzzerManagerSimple(); // 私有构造，确保单例
        
        // 音符结构体
        struct BuzzerNote {
            uint16_t frequency; // 频率 (Hz)，0 表示静音
            uint16_t duration;  // 持续时间 (ms)
        };
        
        // C型板常用高频音符 (2000Hz - 4000Hz 范围内音量最大，4000Hz为谐振点)
        static constexpr uint16_t NOTE_LOW  = 2000;
        static constexpr uint16_t NOTE_MID  = 2500;
        static constexpr uint16_t NOTE_HIGH = 3000;
        static constexpr uint16_t NOTE_MAX  = 4000; // 额定频率，声音最刺耳

        // 定时器基础时钟 (假设 APB1 Timer=84MHz, PSC=84-1)
        static constexpr uint32_t TIM_BASE_CLOCK = 500000; 
        
        // 底层控制
        void controlBuzzer(uint16_t freq_hz);
        void playBlockingMelody(const BuzzerNote* melody, uint16_t length, uint16_t tail_delay_ms);
        void startMelody(const BuzzerNote* melody, uint16_t length);
        void processRing(uint8_t id);
        
        // 队列相关
        static constexpr uint8_t MAX_QUEUE_SIZE = 12;
        uint8_t ring_queue_[MAX_QUEUE_SIZE];
        uint8_t ring_index_ = 0;
        
        // 非阻塞状态机相关
        const BuzzerNote* current_melody_ = nullptr;   // 当前正在播放的旋律指针
        uint16_t melody_length_ = 0;                   // 当前旋律的音符数量
        uint16_t current_note_index_ = 0;              // 当前播放到第几个音符
        uint32_t note_start_time_ = 0;                 // 当前音符开始播放的时间戳
        
        uint32_t last_ring_time_ = 0;                  // 上次一段旋律播放完毕的时间
        bool is_ringing_ = false;                      // 蜂鸣器工作状态标志位
        static constexpr uint32_t RING_INTERVAL_MS = 500; // 连续报警之间的冷却间隔
        
        // 用于动态生成电机响铃序列的缓冲数组 (最大支持响铃 8 次，即 16 个音符状态)
        // 保证内存安全，防止局部变量指针悬挂
        BuzzerNote dynamic_melody_buffer_[16]; 
    };
}

#endif
