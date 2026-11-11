#include "app/yaw_id_core.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
#include <array>
using namespace YawId;

struct Rig {
    Controller c;
    Remote remote;
    Imu imu;
    Motor motor;
    std::uint64_t t=1000000;
    ControlOutput tick(bool arm=true,bool roll=true,bool timing=true) {
        t+=1000;
        remote.received_us=imu.received_us=motor.received_us=t;
        remote.s1=remote.s2=arm?3:2;
        return c.update(t,remote,imu,motor,roll,timing);
    }
    void start() {
        tick(false);
        for(unsigned i=0;i<310;++i) tick();
        assert(c.state==State::Running);
    }
};

int main() {
    assert(crc32("123456789",9)==0xcbf43926U);
    float diagnostic[24];
    for(unsigned i=0;i<24;++i) diagnostic[i]=float(i);
    Imu measured{};measured.angle[2]=1.25f;measured.gyro[2]=-.75f;measured.received_us=12345;
    const auto compact=compact_packet(diagnostic,measured,.125f);
    const float expected[]={0,1,1.25f,-.75f,6,7,10,.125f,5};
    for(unsigned i=0;i<9;++i) assert(compact.v[i]==expected[i]);
    assert(compact.v[9]==float(crc32(compact.v,36)&0xffffffU));
    assert((sizeof(compact)+4)*200*10<Config::vofa_baud);
    diagnostic[7]=0;
    measured.angle[2]=std::numeric_limits<float>::quiet_NaN();
    auto invalid=compact_packet(diagnostic,measured,.125f);
    assert(invalid.v[2]==0 && (unsigned(invalid.v[5])&4));
    measured.angle[2]=1.25f;
    const auto status=status_packet(measured,77,.3f);
    assert(status.v[1]==.3f && status.v[2]==1.25f && status.v[4]==0);
    assert((unsigned(status.v[5])&8193U)==8193U && status.v[6]==77);
    assert(status.v[9]==float(crc32(status.v,36)&0xffffffU));
    for(float tau=-2.2f;tau<2.2f;tau+=.013f)
        assert(std::fabs(command_torque(torque_command(tau))-tau)<.00008f);
    assert(torque_command(std::numeric_limits<float>::quiet_NaN())==0);
    assert(torque_command(100)==16384 && torque_command(-100)==-16384);

    // A receiver already in the run position at boot must not start a sweep.
    Rig boot;
    for(unsigned i=0;i<500;++i) assert(boot.tick().torque==0);
    assert(boot.c.state==State::Disarmed);
    boot.start();
    assert(std::fabs(boot.c.sample_time(boot.t)-boot.c.elapsed(boot.t))<1e-7f);
    assert(std::fabs(boot.c.sample_time(boot.t-2000)-(boot.c.elapsed(boot.t)-.002f))<1e-7f);
    assert(boot.tick(false).torque==0 && boot.c.state==State::Disarmed);

    Rig waiting;
    waiting.tick(false);
    for(unsigned i=0;i<500;++i) waiting.tick(true,false);
    assert(waiting.c.state==State::Preparing);
    for(unsigned i=0;i<20010;++i) waiting.tick(true,false);
    assert(waiting.c.state==State::Fault);

    Rig limit;limit.start();limit.imu.angle[2]=Config::hard_angle_rad+.01f;
    assert(limit.tick().torque==0 && limit.c.state==State::Fault);
    // Timing/watchdog faults must never be followed by a restoring torque.
    for(unsigned i=0;i<30;++i) assert(limit.tick(true,true,false).torque==0);
    assert(limit.tick(false).torque==0 && limit.c.state==State::Disarmed);

    Rig stale;stale.start();stale.t+=60000;stale.remote.received_us=stale.t;
    assert(stale.c.update(stale.t,stale.remote,stale.imu,stale.motor,true).torque==0);
    assert(stale.c.state==State::Fault);
    Rig lost;lost.start();lost.t+=101000;
    assert(lost.c.update(lost.t,lost.remote,lost.imu,lost.motor,true).torque==0);
    assert(lost.c.state==State::Disarmed);
    Rig reverse;reverse.start();
    reverse.remote.received_us=reverse.imu.received_us=reverse.motor.received_us=reverse.t-1;
    assert(reverse.c.update(reverse.t-1,reverse.remote,reverse.imu,reverse.motor,true).torque==0);
    assert(reverse.c.state==State::Fault);
    Rig bad;bad.start();bad.imu.gyro[2]=std::numeric_limits<float>::quiet_NaN();
    assert(bad.tick().torque==0 && bad.c.state==State::Fault);

    Exciter exciter;exciter.reset();
    constexpr unsigned nfreq=sizeof(Config::step_hz)/sizeof(float);
    constexpr unsigned namp=sizeof(Config::step_amplitude_scale)/sizeof(float);
    std::array<unsigned,nfreq*namp> counts{};
    float duration=0;
    for(float f:Config::step_hz) duration+=namp*Config::step_cycles/f;
    assert(std::fabs(duration-222.f)<.001f);
    for(unsigned ms=0;ms<unsigned(duration*1000);++ms) {
        auto ex=exciter.update(float(ms)*.001f);
        if(ex.done) break;
        assert(ex.stage<counts.size());++counts[ex.stage];
        assert(std::isfinite(ex.torque) && std::fabs(ex.torque)<=ex.amplitude+.00001f);
        assert(ex.amplitude<=Config::amplitude_high_nm);
    }
    assert(exciter.update(duration+.01f).done);
    for(unsigned f=0;f<nfreq;++f) for(unsigned a=0;a<namp;++a)
        assert(std::fabs(float(counts[f*namp+a])-8000.f/Config::step_hz[f])<=2);
    exciter.reset();assert(exciter.update(-1).done);
    exciter.reset();exciter.update(1);assert(exciter.update(.9f).done);

    // Traverse a whole run, checking slew and end latch without a physical plant.
    Rig run;run.start();float previous=0;
    for(unsigned ms=0;ms<223000;++ms) {
        auto out=run.tick();
        assert(std::isfinite(out.torque) && std::fabs(out.torque)<=Config::torque_limit_nm+.00001f);
        if(run.c.state==State::Running) assert(std::fabs(out.torque-previous)<=.1601f);
        previous=out.torque;
    }
    assert(run.c.state==State::Done);
    run.tick();assert(run.c.state==State::Done);
    run.tick(false);assert(run.c.state==State::Disarmed);
    assert(Config::roll_target_rad==Config::omni.roll.reference.normal_rad);
    std::puts("PASS: CRC, conversion, arming, timeouts, limits, watchdog gate, 21 excitation stages, full run");
}
