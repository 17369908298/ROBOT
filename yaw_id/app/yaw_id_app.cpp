#include "app/yaw_id_app.h"
#include "app/yaw_id_core.hpp"
#include "can.h"
#include "usart.h"
#include "tim.h"
#include "cmsis_os2.h"
#include "user/core/BSP/RemoteControl/DT7.hpp"
#include "user/core/BSP/IMU/HI12_imu.hpp"
#include "user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "user/core/BSP/Motor/DM/DmMotor.hpp"
#include "user/core/HAL/UART/uart_hal.hpp"

extern "C" {
volatile float yaw_id_probe_channels[24]{};
volatile unsigned yaw_id_state{}, yaw_id_fault{}, yaw_id_tx_drops{}, yaw_id_can_errors{};
volatile unsigned yaw_id_init_stage{}, yaw_id_vofa_tx_frames{}, yaw_id_vofa_status_frames{};
}

namespace {
using namespace YawId;
struct Critical {
    std::uint32_t saved{__get_PRIMASK()};
    Critical() {__disable_irq();} ~Critical() {__set_PRIMASK(saved);}
};
using Packet=TelemetryPacket;
struct Snapshot {Remote remote;Imu imu;Motor gm,roll,pitch;};
Snapshot sensors{};
Controller controller;
ALG::Omni::RollImpedanceController roll_ctrl;
BSP::REMOTE_CONTROL::RemoteController remote_driver;
BSP::IMU::HI12_float imu_driver;
using Bus = HAL::CAN::CanDeviceId;
BSP::Motor::Dji::GM6020<1> yaw_driver(0x204,{std::uint8_t(Config::yaw_feedback_id-0x204)},0x1fe,Bus::HAL_Can2);
BSP::Motor::DM::J4310_2EC<1> roll_driver(0,{std::uint8_t(Config::roll_rx_id)},{Config::roll_tx_id},Bus::HAL_Can1);
BSP::Motor::DM::J4310_2EC<1> pitch_driver(0,{std::uint8_t(Config::pitch_rx_id)},{Config::pitch_tx_id},Bus::HAL_Can1);
BSP::Motor::DM::J6006_2EC<1> base_driver(0,{3},{Config::big_yaw_tx_id},Bus::HAL_Can1);
HAL::CAN::ICanDevice *can1_device{}, *can2_device{};
void on_can1(const HAL::CAN::Frame& frame);
void on_can2(const HAL::CAN::Frame& frame);
osThreadId_t control_thread{};
osMessageQueueId_t packet_queue{};
alignas(4) std::uint8_t imu_dma[164]{},dt7_dma[36]{},tx_dma[sizeof(Packet)+4]{};
unsigned imu_position{},uart_recover{},frame_sequence{};
volatile bool initialized{},watchdog_trip{};
bool roll_initialized{},pitch_initialized{},aux_was_enabled{};
std::uint64_t last_complete_us{},last_control_us{},last_aux_us{},last_special_us{};
std::uint64_t clock_ms{},clock_cycles{};
float pitch_target{};
unsigned can_submit_sequence{};
std::uint64_t can_submit_us{};
std::int16_t can_submitted_command{};

std::uint64_t now_us() {
    Critical lock;
    const auto coarse=std::uint32_t(HAL_GetTick()-std::uint32_t(clock_ms));
    const std::uint64_t low=std::uint32_t(DWT->CYCCNT-std::uint32_t(clock_cycles));
    const std::uint64_t wrap=std::uint64_t(1)<<32;
    const auto expected=std::uint64_t(coarse)*SystemCoreClock/1000U;
    const auto wraps=expected+wrap/2>=low?(expected+wrap/2-low)/wrap:0;
    const auto cycles=low+wraps*wrap;
    return clock_ms*1000+(cycles/SystemCoreClock)*1000000+
           (cycles%SystemCoreClock)*1000000/SystemCoreClock;
}
bool send_yaw(std::int16_t command) {
    Critical lock;
    yaw_driver.setCAN(command,Config::yaw_feedback_id-0x204);
    const bool ok=yaw_driver.sendCAN();
    if(ok) {can_submit_us=now_us();can_submitted_command=command;++can_submit_sequence;}
    else ++yaw_id_can_errors;
    return ok;
}
bool special(unsigned id,std::uint8_t code) {
    Critical lock;
    const bool enable=code==0xfc;
    using BSP::Motor::DM::MIT;
    bool ok=false;
    if(id==Config::roll_tx_id) ok=enable?roll_driver.On(1,MIT):roll_driver.Off(1,MIT);
    if(id==Config::pitch_tx_id) ok=enable?pitch_driver.On(1,MIT):pitch_driver.Off(1,MIT);
    if(id==Config::big_yaw_tx_id) ok=base_driver.Off(1,MIT);
    if(!ok) ++yaw_id_can_errors;
    return ok;
}
bool send_mit(unsigned id,float speed,float kd,float torque) {
    Critical lock;
    bool ok=false;
    if(id==Config::roll_tx_id) ok=roll_driver.ctrl_Mit(1,0,speed,0,kd,torque);
    if(id==Config::pitch_tx_id) ok=pitch_driver.ctrl_Mit(1,0,speed,0,kd,torque);
    if(!ok) ++yaw_id_can_errors;
    return ok;
}
// Queueing to a CAN mailbox is not wire ACK nor measured applied torque.
void auxiliary(const Snapshot& s,std::uint64_t now,float dt,bool enable) {
    const bool roll_ok=fresh(now,s.roll.received_us,Config::roll_feedback_timeout_ms) && s.roll.state<=1;
    const bool pitch_ok=fresh(now,s.pitch.received_us,Config::roll_feedback_timeout_ms) && s.pitch.state<=1 &&
                        fresh(now,s.imu.received_us,Config::sensor_timeout_ms);
    const bool roll_on=enable && Config::roll_enabled && roll_ok;
    const bool pitch_on=enable && Config::pitch_enabled && pitch_ok;
    if(!enable && aux_was_enabled) {last_special_us=0;last_aux_us=0;}
    aux_was_enabled=enable;
    if(now-last_special_us>=50000) {
        last_special_us=now;
        // Do not clear motor fault codes automatically; the cause must be inspected.
        if(!roll_on || s.roll.state==0) special(Config::roll_tx_id,roll_on?0xfc:0xfd);
        if(!pitch_on || s.pitch.state==0) special(Config::pitch_tx_id,pitch_on?0xfc:0xfd);
        special(Config::big_yaw_tx_id,0xfd);
    }
    if(!roll_on) {roll_initialized=false;roll_ctrl.reset();}
    else {
        if(!roll_initialized) {roll_ctrl.reset(s.roll.angle);roll_initialized=true;}
        roll_ctrl.update(Config::roll_target_rad,s.roll.angle,s.roll.speed,
                         std::uint32_t(s.roll.received_us/1000),dt,true);
    }
    if(!pitch_on) pitch_initialized=false;
    else {
        if(!pitch_initialized) {
            pitch_target=clamp(s.imu.angle[1],Config::pitch_min_rad,Config::pitch_max_rad);
            pitch_initialized=true;
        }
        const float stick=std::fabs(s.remote.right_y)<.05f?0:s.remote.right_y;
        pitch_target=clamp(pitch_target+stick*Config::pitch_manual_rate*dt,
                           Config::pitch_min_rad,Config::pitch_max_rad);
    }
    if(now-last_aux_us<2000) return;
    last_aux_us=now;
    const auto& roll=roll_ctrl.output();
    const bool apply_roll=roll_on && s.roll.state==1 && roll.valid;
    const float rv=apply_roll?roll.reference_speed_radps:0;
    const float rd=apply_roll?roll.damping_nms_rad:0;
    const float rt=apply_roll?roll.mit_ff_nm:0;
    const bool accepted=send_mit(Config::roll_tx_id,rv,rd,rt);
    if(accepted) roll_ctrl.set_applied_command(rv,rd,rt,roll.gravity_torque_nm,apply_roll);
    if(pitch_on && s.pitch.state==1) {
        const float velocity=clamp(Config::pitch_kp*angle_error(pitch_target,s.imu.angle[1]),
                                    -Config::pitch_speed_limit,Config::pitch_speed_limit);
        const float torque=clamp(Config::pitch_gravity_nm*std::cos(s.imu.angle[1]+Config::pitch_gravity_offset_rad),
                                  -Config::pitch_torque_limit,Config::pitch_torque_limit);
        send_mit(Config::pitch_tx_id,velocity,Config::pitch_kd,torque);
    } else send_mit(Config::pitch_tx_id,0,0,0);
}
bool start_rx(UART_HandleTypeDef& uart,std::uint8_t* buffer,unsigned count,bool circular) {
    HAL_UART_AbortReceive(&uart);
    uart.hdmarx->Init.Mode=circular?DMA_CIRCULAR:DMA_NORMAL;
    if(HAL_DMA_Init(uart.hdmarx)!=HAL_OK) return false;
    __HAL_UART_CLEAR_OREFLAG(&uart);
    HAL::UART::Data data{buffer,std::uint16_t(count)};
    auto& bus=HAL::UART::get_uart_bus_instance();
    auto& device=uart.Instance==USART1?bus.get_uart1():bus.get_uart7();
    if(!device.receive_dma_idle(data)) return false;
    if(!circular) __HAL_DMA_DISABLE_IT(uart.hdmarx,DMA_IT_HT);
    return true;
}
bool initialize() {
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    clock_ms=HAL_GetTick();clock_cycles=DWT->CYCCNT;
    // App runtime override leaves the original CubeMX pin/DMA assignments intact.
    yaw_id_init_stage=1;
    huart8.Init.BaudRate=Config::vofa_baud;
    if(HAL_UART_Init(&huart8)!=HAL_OK) return false;
    yaw_id_init_stage=2;
    {
        Critical lock;
        auto& bus=HAL::CAN::get_can_bus_instance();
        can1_device=&bus.get_can1();can2_device=&bus.get_can2();
        can1_device->register_rx_callback(on_can1);can2_device->register_rx_callback(on_can2);
    }
    if(hcan1.State!=HAL_CAN_STATE_LISTENING || hcan2.State!=HAL_CAN_STATE_LISTENING) return false;
    yaw_id_init_stage=3;
    if(!start_rx(huart1,dt7_dma,sizeof(dt7_dma),false)) return false;
    yaw_id_init_stage=4;
    if(!start_rx(huart7,imu_dma,sizeof(imu_dma),true)) return false;
    roll_ctrl.configure(Config::roll_controller);roll_ctrl.reset();
    const auto timer_hz=HAL_RCC_GetPCLK1Freq()*((RCC->CFGR&RCC_CFGR_PPRE1)?2U:1U);
    htim6.Init.Prescaler=timer_hz/1000000U-1;
    htim6.Init.Period=1000U-1;
    yaw_id_init_stage=5;
    if(HAL_TIM_Base_Init(&htim6)!=HAL_OK) return false;
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn,6,0);HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    last_complete_us=now_us();initialized=true;
    if(HAL_TIM_Base_Start_IT(&htim6)!=HAL_OK) return false;
    yaw_id_init_stage=6;
    return true;
}
template<class Driver> Motor read_motor(Driver& driver,const Motor& previous,std::uint64_t stamp) {
    Motor m=previous;
    m.angle=normalize(driver.getAngleRad(1));m.speed=driver.getVelocityRads(1);
    m.torque=driver.getTorque(1);m.temperature=driver.getTemperature(1);
    if(!std::isfinite(m.angle) || !std::isfinite(m.speed) || !std::isfinite(m.torque)) return previous;
    m.received_us=stamp;++m.sequence;return m;
}
void on_can1(const HAL::CAN::Frame& frame) {
    if(frame.dlc!=8 || frame.is_extended_id || frame.is_remote_frame) return;
    const auto time=now_us();
    if(frame.id==Config::roll_rx_id) {
        roll_driver.Parse(frame);sensors.roll=read_motor(roll_driver,sensors.roll,time);
        sensors.roll.state=roll_driver.getError(1);
    }
    if(frame.id==Config::pitch_rx_id) {
        pitch_driver.Parse(frame);sensors.pitch=read_motor(pitch_driver,sensors.pitch,time);
        sensors.pitch.state=pitch_driver.getError(1);
    }
}
void on_can2(const HAL::CAN::Frame& frame) {
    if(frame.dlc!=8 || frame.is_extended_id || frame.is_remote_frame || frame.id!=Config::yaw_feedback_id) return;
    yaw_driver.Parse(frame);sensors.gm=read_motor(yaw_driver,sensors.gm,now_us());
    // Existing GM6020 driver uses Kt=.7; the identification calibration uses configurable Kt=.741.
    sensors.gm.torque=yaw_driver.getCurrent(1)*Config::torque_constant;
}
void receive_can(CAN_HandleTypeDef* bus,unsigned) {
    auto* device=bus->Instance==CAN1?can1_device:can2_device;
    if(!device) return;
    HAL::CAN::Frame frame{};
    while(device->receive(frame)) {} // existing HAL driver dispatches registered callbacks
}
void export_packet(const Snapshot& s,const ControlOutput& out,std::uint64_t time) {
    float diagnostic[24]{};auto* v=diagnostic;
    const auto packed_at=now_us();
    const auto age=[packed_at](std::uint64_t stamp) {return stamp && packed_at>=stamp?float(packed_at-stamp)*.001f:1000.f;};
    v[0]=controller.elapsed(time);v[1]=command_torque(torque_command(out.torque));v[2]=out.relative_angle;
    v[3]=Config::imu_yaw_sign*s.imu.gyro[Config::imu_yaw_axis];
    v[4]=out.base;v[5]=out.excitation;
    v[6]=controller.state==State::Running?float(Config::mode):0;
    v[7]=float(out.flags);v[8]=age(s.gm.received_us);v[9]=age(s.imu.received_us);
    // Additional data are diagnostic; old I0-I9 retain their meaning.
    v[10]=float((++frame_sequence)&0xffffffU);v[11]=float(s.gm.sequence&0xffffffU);
    v[12]=float(s.imu.sequence&0xffffffU);v[13]=float(yaw_id_tx_drops&0xffffffU);
    {Critical lock;v[14]=age(can_submit_us);v[16]=command_torque(can_submitted_command);}
    v[15]=s.gm.torque;
    v[17]=out.id.frequency;v[18]=out.id.amplitude;v[19]=float(out.id.stage);
    v[20]=s.gm.temperature;v[21]=s.imu.angle[1];v[22]=3; // schema version 3
    for(unsigned i=0;i<23;++i) {
        if(!std::isfinite(v[i])) {v[i]=0;v[7]=float(out.flags|4U);}
        if(v[i]==0) v[i]=0; // canonical +0 for decimal CSV round trips
    }
    v[23]=float(crc32(v,23*sizeof(float))&0xffffffU);
    {Critical lock;for(unsigned i=0;i<24;++i) yaw_id_probe_channels[i]=v[i];}
    // Serial schema 5 carries the raw IMU angle/rate and that frame's receive timestamp.
    // Keep the full schema-3 diagnostic array available through the debugger.
    const Packet packet=compact_packet(diagnostic,s.imu,controller.sample_time(s.imu.received_us));
    if(osMessageQueuePut(packet_queue,&packet,0,0)!=osOK) ++yaw_id_tx_drops;
}
void control_task(void*) {
    if(!initialize()) {
        yaw_id_fault|=1;
        for(;;) {YawId_FaultStop();osDelay(10);}
    }
    unsigned divider=0;
    for(;;) {
        const auto flags=osThreadFlagsWait(1,osFlagsWaitAny,10);
        if(flags&osFlagsError) {YawId_FaultStop();yaw_id_fault|=2;continue;}
        Snapshot s;unsigned recover;
        {Critical lock;s=sensors;recover=uart_recover;uart_recover=0;}
        if(recover&1U) start_rx(huart1,dt7_dma,sizeof(dt7_dma),false);
        if(recover&2U) {Critical lock;imu_position=0;imu_driver.ResetReceiveBuffer();start_rx(huart7,imu_dma,sizeof(imu_dma),true);}
        const auto now=now_us();
        const float dt=last_control_us && now>last_control_us?float(now-last_control_us)*1e-6f:.001f;
        last_control_us=now;
        if(fresh(now,s.remote.received_us,Config::remote_timeout_ms) && !s.remote.armed(now)) {
            watchdog_trip=false;yaw_id_fault&=~4U;
        }
        const bool active=s.remote.armed(now) && controller.ready_to_arm() && !watchdog_trip &&
            controller.state!=State::Fault && controller.state!=State::Done;
        auxiliary(s,now,clamp(dt,.00001f,.005f),active);
        const bool roll_fresh=fresh(now,s.roll.received_us,Config::roll_feedback_timeout_ms) && s.roll.state==1;
        const bool roll_ready=!Config::roll_enabled || !Config::wait_roll_settled ||
            (roll_fresh && roll_ctrl.output().valid && roll_ctrl.trajectory().done() &&
             std::fabs(angle_error(Config::roll_target_rad,s.roll.angle))<.02f && std::fabs(s.roll.speed)<.08f);
        const bool timing_ok=!watchdog_trip && dt>0 && dt<=.0025f &&
            (controller.state!=State::Running || !Config::roll_enabled || roll_fresh);
        auto out=controller.update(now,s.remote,s.imu,s.gm,roll_ready,timing_ok);
        // Transmit the quantized current corresponding to the recorded final command.
        const bool submitted=send_yaw(torque_command(out.torque));
        if(!submitted) {out.flags|=16384;YawId_FaultStop();watchdog_trip=true;yaw_id_fault|=8;}
        yaw_id_state=unsigned(controller.state);
        if(++divider>=Config::telemetry_divider) {divider=0;export_packet(s,out,now);}
        {Critical lock;last_complete_us=now_us();}
    }
}
void telemetry_task(void*) {
    Packet packet{};
    for(;;) {
        const auto result=osMessageQueueGet(packet_queue,&packet,nullptr,std::max(1U,osKernelGetTickFreq()/50U));
        if(result==osErrorTimeout || result==osErrorResource) {
            // A failed control initialization must not leave VOFA completely silent.
            // This fallback never enables an axis or starts excitation.
            Critical lock;
            packet=status_packet(sensors.imu,++frame_sequence,command_torque(can_submitted_command));
            ++yaw_id_vofa_status_frames;
        } else if(result!=osOK) {osDelay(1);continue;}
        if(huart8.gState!=HAL_UART_STATE_READY) {++yaw_id_tx_drops;continue;}
        std::memcpy(tx_dma,packet.v,sizeof(packet));
        constexpr unsigned tail=sizeof(packet);
        tx_dma[tail]=0;tx_dma[tail+1]=0;tx_dma[tail+2]=0x80;tx_dma[tail+3]=0x7f;
        HAL::UART::Data data{tx_dma,sizeof(tx_dma)};
        if(HAL::UART::get_uart_bus_instance().get_uart8().transmit_dma(data)) ++yaw_id_vofa_tx_frames;
        else ++yaw_id_tx_drops;
    }
}
}

extern "C" void YawId_CreateTasks(void) {
    packet_queue=osMessageQueueNew(8,sizeof(Packet),nullptr);
    const osThreadAttr_t control_attr={"yaw_id_control",0,nullptr,0,nullptr,6144,osPriorityHigh,0,0};
    const osThreadAttr_t tx_attr={"yaw_id_vofa",0,nullptr,0,nullptr,2048,osPriorityNormal,0,0};
    control_thread=osThreadNew(control_task,nullptr,&control_attr);
    const auto tx=osThreadNew(telemetry_task,nullptr,&tx_attr);
    if(!packet_queue || !control_thread || !tx) Error_Handler();
}
// Keep the generated USB CDC endpoint linkable; this bench firmware has no USB command protocol.
extern "C" void Sentry_UsbRx(const std::uint8_t*, std::uint32_t) {}
extern "C" void YawId_TimerIrq(void) {
    if(!initialized) return;
    if(now_us()-last_complete_us>YawId::Config::watchdog_us) {
        watchdog_trip=true;yaw_id_fault|=4;YawId_FaultStop();
    }
    osThreadFlagsSet(control_thread,1);
}
extern "C" void YawId_FaultStop(void) {
    if(!initialized) return;
    HAL_CAN_AbortTxRequest(&hcan2,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
    (void)send_yaw(0);
    special(YawId::Config::roll_tx_id,0xfd);special(YawId::Config::pitch_tx_id,0xfd);
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* bus) {receive_can(bus,CAN_RX_FIFO0);}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* bus) {receive_can(bus,CAN_RX_FIFO1);}
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef*) {++yaw_id_can_errors;}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart,std::uint16_t size) {
    const bool idle=HAL_UARTEx_GetRxEventType(uart)==HAL_UART_RXEVENT_IDLE;
    if(uart->Instance==USART1) {
        const auto end=now_us()-(idle?110:0);
        std::uint8_t local[36];const unsigned length=std::min<unsigned>(size,sizeof(local));
        std::memcpy(local,dt7_dma,length);
        // Rearm before decoding; the normal DMA transfer has already ended.
        HAL::UART::Data data{dt7_dma,sizeof(dt7_dma)};
        if(!HAL::UART::get_uart_bus_instance().get_uart1().receive_dma_idle(data)) uart_recover|=1;
        __HAL_DMA_DISABLE_IT(uart->hdmarx,DMA_IT_HT);
        if(length%18==0) for(unsigned i=0;i<length;i+=18) {
            if(!remote_driver.parseData(local+i)) continue;
            if(remote_driver.get_s1()<1 || remote_driver.get_s1()>3 ||
               remote_driver.get_s2()<1 || remote_driver.get_s2()>3) continue;
            sensors.remote.s1=remote_driver.get_s1();sensors.remote.s2=remote_driver.get_s2();
            sensors.remote.right_y=remote_driver.get_right_y();
            sensors.remote.received_us=end-std::uint64_t(length-i-18)*110;++sensors.remote.sequence;
        }
    }
    if(uart->Instance==UART7) {
        Critical lock;
        // Read the live cursor: stale HT/TC notifications must not repeat IDLE bytes.
        const unsigned position=(sizeof(imu_dma)-__HAL_DMA_GET_COUNTER(uart->hdmarx))%sizeof(imu_dma);
        const unsigned count=(position+sizeof(imu_dma)-imu_position)%sizeof(imu_dma);
        const auto endpoint=now_us()-(idle?10000000U/uart->Init.BaudRate:0);
        for(unsigned i=0;i<count;++i) {
            const auto stamp=endpoint-std::uint64_t(count-i-1)*10000000U/uart->Init.BaudRate;
            const auto before=imu_driver.GetFeedbackSequence();
            imu_driver.DataUpdate(imu_dma+imu_position,1);
            if(imu_driver.GetFeedbackSequence()!=before) {
                Imu value=sensors.imu;bool finite=true;
                for(unsigned axis=0;axis<3;++axis) {
                    value.angle[axis]=imu_driver.GetAngle(axis)*pi/180.f;
                    value.gyro[axis]=imu_driver.GetGyro(axis)*pi/180.f;
                    finite=finite && std::isfinite(value.angle[axis]) && std::isfinite(value.gyro[axis]);
                }
                if(finite) {value.received_us=stamp;++value.sequence;sensors.imu=value;}
            }
            imu_position=(imu_position+1)%sizeof(imu_dma);
        }
    }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
    if(uart->Instance==USART1) {uart_recover|=1;sensors.remote.received_us=0;}
    if(uart->Instance==UART7) {uart_recover|=2;sensors.imu.received_us=0;}
}
