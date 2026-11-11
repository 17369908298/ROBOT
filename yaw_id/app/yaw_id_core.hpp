#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "app/yaw_id_config.hpp"
#include "user/core/Alg/Omni/torque_current_converter.hpp"

namespace YawId {
inline constexpr float pi = 3.14159265358979323846f;
inline float clamp(float x, float lo, float hi) { return std::max(lo, std::min(x, hi)); }
inline float angle_error(float a, float b) { return std::remainder(a-b, 2*pi); }
inline float normalize(float x) { x=std::fmod(x,2*pi); return x<0 ? x+2*pi : x; }
inline bool fresh(std::uint64_t now, std::uint64_t stamp, unsigned age_ms) {
    return stamp && now>=stamp && now-stamp < std::uint64_t(age_ms)*1000;
}
inline std::uint32_t crc32(const void* data, unsigned length) {
    const auto* p=static_cast<const std::uint8_t*>(data);
    std::uint32_t crc=0xffffffffU;
    while(length--) {
        crc^=*p++;
        for(unsigned b=0;b<8;++b) crc=(crc>>1)^(0xedb88320U & (0U-(crc&1U)));
    }
    return ~crc;
}
inline std::int16_t torque_command(float torque) {
    const ALG::Omni::TorqueCurrentConverter converter({Config::torque_constant,Config::current_max_a,16384,16384});
    return converter.torque_to_command(torque);
}
inline float command_torque(std::int16_t raw) {
    const ALG::Omni::TorqueCurrentConverter converter({Config::torque_constant,Config::current_max_a,16384,16384});
    return converter.command_to_torque_Nm(raw);
}
struct TelemetryPacket {float v[Config::vofa_channels];};
static_assert(sizeof(TelemetryPacket)==40, "Schema 5 carries ten float32 values");
struct Remote {
    std::uint64_t received_us{};
    std::uint32_t sequence{};
    float right_y{};
    std::uint8_t s1{},s2{};
    bool armed(std::uint64_t now) const {
        return fresh(now,received_us,Config::remote_timeout_ms) && s1==3 && s2==3;
    }
};
struct Imu {
    float angle[3]{}, gyro[3]{}; // rad and rad/s, gyro unfiltered
    std::uint32_t device_ms{},sequence{};
    std::uint64_t received_us{};
};
inline TelemetryPacket compact_packet(const float (&diagnostic)[24],const Imu& imu,float imu_time_s) {
    const auto* v=diagnostic;
    // Both measurements come directly from the same validated HI12 frame snapshot.
    TelemetryPacket packet{{v[0],v[1],Config::imu_yaw_sign*imu.angle[Config::imu_yaw_axis],
                            Config::imu_yaw_sign*imu.gyro[Config::imu_yaw_axis],
                            v[6],v[7],v[10],imu_time_s,float(Config::vofa_schema),0}};
    if(!imu.received_us) packet.v[5]=float(unsigned(packet.v[5])|516U);
    for(unsigned i=0;i<9;++i) {
        if(!std::isfinite(packet.v[i])) {packet.v[i]=0;packet.v[5]=float(unsigned(packet.v[5])|4U);}
        if(packet.v[i]==0) packet.v[i]=0; // canonical +0 for CSV CRC round trips
    }
    packet.v[9]=float(crc32(packet.v,9*sizeof(float))&0xffffffU);
    return packet;
}
inline TelemetryPacket status_packet(const Imu& imu,std::uint32_t sequence,float last_submitted_torque) {
    float diagnostic[24]{};
    diagnostic[1]=last_submitted_torque;
    diagnostic[7]=8193; // idle + missing control-cycle telemetry; never a fit sample
    diagnostic[10]=float(sequence&0xffffffU);
    return compact_packet(diagnostic,imu,0);
}
struct Motor {
    float angle{},speed{},torque{},temperature{};
    std::uint32_t sequence{};
    std::uint64_t received_us{};
    std::uint8_t state{};
};
struct Excitation { float torque{},frequency{},amplitude{};unsigned stage{};bool done{},ramp{}; };
class Exciter {
public:
    void reset() { rng_=Config::prbs_seed;next_=0;sign_=1;last_time_=-1; }
    Excitation update(float time) {
        Excitation r{};float phase=0,fade=1;
        if(!std::isfinite(time) || time<0 || time<last_time_) {r.done=true;return r;}
        last_time_=time;
        using namespace Config;
        if constexpr(mode==Mode::SteppedSine) {
            float local=time;bool found=false;
            for(unsigned f=0;f<sizeof(step_hz)/sizeof(float) && !found;++f) {
                for(unsigned a=0;a<sizeof(step_amplitude_scale)/sizeof(float);++a) {
                    const float duration=step_cycles/step_hz[f];
                    if(local<duration) {
                        r.frequency=step_hz[f];r.stage=f*(sizeof(step_amplitude_scale)/sizeof(float))+a;
                        r.amplitude=envelope(r.frequency)*step_amplitude_scale[a];
                        const float edge=step_fade_cycles/r.frequency;
                        fade=clamp(std::min(local,duration-local)/edge,0,1);
                        phase=2*pi*r.frequency*local;found=true;break;
                    }
                    local-=duration;
                }
            }
            if(!found) {r.done=true;return r;}
        } else {
            if(time>=duration_s) {r.done=true;return r;}
            fade=ramp_s>0?clamp(std::min(time,duration_s-time)/ramp_s,0,1):1;
            if constexpr(mode==Mode::Sine) {r.frequency=sine_hz;r.amplitude=sine_amplitude_nm;phase=2*pi*sine_hz*time;}
            if constexpr(mode==Mode::Chirp) {
                const float s=time/duration_s;
                r.frequency=chirp_start_hz+(chirp_end_hz-chirp_start_hz)*std::pow(s,chirp_exponent);
                phase=2*pi*(chirp_start_hz*time+(chirp_end_hz-chirp_start_hz)*duration_s*
                             std::pow(s,chirp_exponent+1)/(chirp_exponent+1));
                r.amplitude=envelope(r.frequency);
            }
            if constexpr(mode==Mode::Prbs) {
                if(time>=next_) {
                    rng_^=rng_<<13;rng_^=rng_>>17;rng_^=rng_<<5;
                    sign_=(rng_&1)?1:-1;next_=time+prbs_switch_s;
                }
                r.amplitude=prbs_amplitude_nm;r.frequency=1/prbs_switch_s;
                r.torque=r.amplitude*fade*float(sign_);r.ramp=fade<.999f;return r;
            }
        }
        r.ramp=fade<.999f;r.torque=r.amplitude*fade*std::sin(phase);return r;
    }
    static float envelope(float hz) {
        const float mix=clamp((hz-Config::chirp_start_hz)/(Config::amplitude_full_hz-Config::chirp_start_hz),0,1);
        return Config::amplitude_low_nm+(Config::amplitude_high_nm-Config::amplitude_low_nm)*mix;
    }
private:
    std::uint32_t rng_{Config::prbs_seed};float next_{},last_time_{-1};int sign_{1};
};
enum class State : unsigned { Disarmed=0, Preparing=1, Running=2, Done=3, Fault=4 };
struct ControlOutput {float torque{},base{},excitation{},relative_angle{};Excitation id{};unsigned flags{};};
// Platform-independent state machine, used by firmware and host regression.
class Controller {
public:
    State state{State::Disarmed};
    ControlOutput update(std::uint64_t now,const Remote& remote,const Imu& imu,const Motor& gm,
                         bool roll_settled,bool timing_ok=true) {
        ControlOutput out{};
        const bool remote_ok=fresh(now,remote.received_us,Config::remote_timeout_ms);
        const bool sensors=fresh(now,imu.received_us,Config::sensor_timeout_ms) &&
                           fresh(now,gm.received_us,Config::sensor_timeout_ms);
        const float angle=Config::imu_yaw_sign*imu.angle[Config::imu_yaw_axis];
        const float speed=Config::imu_yaw_sign*imu.gyro[Config::imu_yaw_axis];
        const float dt=previous_us_?(now>previous_us_?float(now-previous_us_)*1e-6f:0):.001f;
        previous_us_=now;
        if(remote_ok && !remote.armed(now)) release_seen_=true;
        if(!remote.armed(now)) {
            state=State::Disarmed;last_torque_=0;ready_since_=0;start_us_=0;
            out.flags=1;return out;
        }
        if(state==State::Done || state==State::Fault) {
            out.flags=8;
            // Valid feedback permits restoring braking; loss of feedback stays zero.
            if(timing_ok && dt>0 && dt<=.005f && sensors && std::isfinite(angle) && std::isfinite(speed)) {
                out.relative_angle=angle_error(angle,center_);
                const float brake=clamp(-Config::soft_kp*out.relative_angle-Config::soft_kd*speed,
                                         -Config::soft_limit_nm,Config::soft_limit_nm);
                const float step=60.f*clamp(dt,0,.005f);
                out.torque=clamp(brake,last_torque_-step,last_torque_+step);
                last_torque_=out.torque;
            } else last_torque_=0;
            return out;
        }
        if(!release_seen_) {out.flags=1;return out;}
        if(!timing_ok || dt<=0 || dt>.005f || !sensors ||
           !std::isfinite(angle) || !std::isfinite(speed)) {
            if(state==State::Running) {state=State::Fault;release_seen_=false;}
            last_torque_=0;ready_since_=0;out.flags=(!sensors?516U:0U)|8192U;return out;
        }
        if(state==State::Disarmed) {state=State::Preparing;prepare_us_=now;center_=angle;}
        if(state==State::Preparing) {
            if(now-prepare_us_>std::uint64_t(Config::preparing_timeout_s*1e6f)) {
                state=State::Fault;release_seen_=false;out.flags=8;return out;
            }
            if(!roll_settled) {ready_since_=0;out.flags=1024;return out;}
            if(!ready_since_) ready_since_=now;
            if(now-ready_since_<std::uint64_t(Config::startup_settle_s*1e6f)) {out.flags=1024;return out;}
            center_=angle;start_us_=now;state=State::Running;exciter_.reset();
        }
        out.relative_angle=angle_error(angle,center_);
        if(std::fabs(out.relative_angle)>=Config::hard_angle_rad || std::fabs(speed)>=Config::hard_speed_rad_s) {
            state=State::Fault;release_seen_=false;last_torque_=0;out.flags=8|16;return out;
        }
        out.id=exciter_.update(float(now-start_us_)*1e-6f);
        if(out.id.done) {state=State::Done;release_seen_=false;last_torque_=0;out.flags=8;return out;}
        out.base=clamp(-Config::soft_kp*out.relative_angle-Config::soft_kd*speed,
                       -Config::soft_limit_nm,Config::soft_limit_nm);
        out.excitation=out.id.torque;
        float requested=out.base+out.excitation;
        const float predicted=std::fabs(out.relative_angle+speed*Config::governor_prediction_s);
        if(out.relative_angle*requested>0 && predicted>Config::governor_start_rad) {
            requested*=clamp((Config::governor_stop_rad-predicted)/
                (Config::governor_stop_rad-Config::governor_start_rad),0,1);out.flags|=2048;
        }
        const float limited=clamp(requested,-Config::torque_limit_nm,Config::torque_limit_nm);
        if(limited!=requested) out.flags|=128;
        out.torque=clamp(limited,last_torque_-Config::torque_slew_nm_s*dt,
                        last_torque_+Config::torque_slew_nm_s*dt);
        if(out.torque!=limited) out.flags|=64;
        if(out.id.ramp) out.flags|=256;
        last_torque_=out.torque;return out;
    }
    float elapsed(std::uint64_t now) const {return start_us_ && now>=start_us_?float(now-start_us_)*1e-6f:0;}
    float sample_time(std::uint64_t stamp) const {
        if(!start_us_ || !stamp) return 0;
        return stamp>=start_us_?float(stamp-start_us_)*1e-6f:-float(start_us_-stamp)*1e-6f;
    }
    bool ready_to_arm() const {return release_seen_;}
private:
    bool release_seen_{};std::uint64_t previous_us_{},start_us_{},prepare_us_{},ready_since_{};
    float center_{},last_torque_{};Exciter exciter_{};
};
}
