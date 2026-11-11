#include "communication/wire_protocol.hpp"
#include "feeder_fsm/transform_fsm.hpp"
using namespace Communication;
constexpr uint8_t digits[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
static_assert(Crc16Ccitt(digits, 9) == 0x29b1, "board CRC");
static_assert(Crc16Reflected(digits, 9, 0xa001) == 0x4b37, "navigation CRC");
static_assert(Crc16Reflected(digits, 9, 0x8408) == 0x6f91, "vision CRC");
constexpr uint8_t heading[] = {0xa5, 0x0f, 0, 8, 0, 0, 0x80, 0x3f, 0xd2, 4, 0, 0, 9, 0xc8};
constexpr uint8_t heading_cobs[] = {3, 0xa5, 0x0f, 2, 8, 1, 5, 0x80, 0x3f, 0xd2, 4, 1, 3, 9, 0xc8};
static_assert(Crc16Ccitt(heading, 12) == ReadU16(heading + 12), "heading sample");
constexpr uint8_t nav[] = {0xa5, 2, 0, 0,    0x80, 0x3f, 0,    0,   0,
                           0xbf, 0, 0, 0x80, 0x3e, 0,    0xb4, 0x4c};
static_assert(Crc16Reflected(nav, 15, 0xa001) == ReadU16(nav + 15), "nav sample");
constexpr uint8_t vision[] = {0x53, 0x50, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0,   0,
                              0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x6f, 0x79};
static_assert(Crc16Reflected(vision, 27, 0x8408) == ReadU16(vision + 27), "vision sample");
constexpr bool CheckCobs()
{
    uint8_t encoded[32]{}, decoded[32]{};
    const auto n = CobsEncode(heading, sizeof(heading), encoded, sizeof(encoded));
    if (n != sizeof(heading_cobs))
        return false;
    for (size_t i = 0; i < n; ++i)
        if (encoded[i] != heading_cobs[i])
            return false;
    const auto m = CobsDecode(encoded, n, decoded, sizeof(decoded));
    if (m != sizeof(heading))
        return false;
    for (size_t i = 0; i < m; ++i)
        if (decoded[i] != heading[i])
            return false;
    const uint8_t bad[] = {4, 1, 2};
    if (CobsDecode(bad, 3, decoded, sizeof(decoded)) != 0)
        return false;
    if (CobsEncode(heading, sizeof(heading), encoded, 4) != 0)
        return false;
    return true;
}
static_assert(CheckCobs(), "COBS sample, malformed input and capacity");
using App::Onmi::TransformFSM;
using App::Onmi::TransformState;
constexpr bool CheckTransform()
{
    TransformFSM f;
    if (f.owns_control())
        return false;
    f.update(true, false, false, false, false, false, false);
    if (f.state() != TransformState::TO_TRANSFORM_ALIGN || !f.owns_control())
        return false;
    f.update(true, false, false, true, false, false, false);
    if (f.state() != TransformState::TO_TRANSFORM_ALIGN)
        return false; // roll cannot skip axis alignment
    f.update(true, false, true, false, false, false, false);
    if (f.state() != TransformState::TO_TRANSFORM_ROLL)
        return false;
    f.update(true, false, true, true, false, false, false);
    if (f.state() != TransformState::TRANSFORM)
        return false;
    f.update(false, false, false, false, false, false, false);
    if (f.state() != TransformState::TO_NORMAL_SYNC)
        return false;
    f.update(true, false, true, true, false, true, true);
    if (f.state() != TransformState::TO_NORMAL_SYNC)
        return false; // new commands cannot take ownership during exit
    f.update(false, false, false, false, true, false, false);
    if (f.state() != TransformState::TO_NORMAL_ALIGN)
        return false;
    f.update(false, false, false, false, true, true, false);
    if (f.state() != TransformState::TO_NORMAL_ROLL)
        return false;
    f.update(false, false, false, false, true, true, true);
    if (f.state() != TransformState::NORMAL || f.owns_control())
        return false;
    f.force_fault();
    f.update(true, true, true, true, true, true, true);
    if (!f.faulted() || !f.owns_control())
        return false;
    f.reset();
    return !f.faulted() && !f.owns_control();
}
static_assert(CheckTransform(), "transform stages, ownership and latched fault");
constexpr bool CheckModeRequest()
{
    App::Onmi::GimbalModeRequest mode;
    if (!mode.update(false, true, true, 1))
        return false;
    if (!mode.update(false, true, false, 0))
        return false; // disconnect preserves the selected host mode
    if (mode.update(false, false, false, 0))
        return false;
    if (mode.update(false, true, false, 1))
        return false; // leaving AUTO clears host ownership
    return mode.update(true, false, false, 0);
}
static_assert(CheckModeRequest(), "host mode latching and ownership");
