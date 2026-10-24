// A Wii Remote with a Classic Controller, emulated for a gamepad. Covers what the
// game's WPAD/KPAD use: status, memory reads and writes (EEPROM calibration and
// the extension registers), LEDs, rumble, IR and speaker switches (acknowledged,
// never active) and every data reporting mode a Classic Controller can use.
// References: WiiBrew's Wiimote and Classic Controller pages, Dolphin's
// emulated Wii Remote.

#include "gamepad_remote.h"

#include <algorithm>
#include <cmath>

namespace Wiimote {
namespace GamepadRemoteDetail {

using Bluetooth::LinkState;

constexpr uint8_t kReportLeds = 0x11;
constexpr uint8_t kReportMode = 0x12;
constexpr uint8_t kReportIr = 0x13;
constexpr uint8_t kReportSpeaker = 0x14;
constexpr uint8_t kReportStatusRequest = 0x15;
constexpr uint8_t kReportWriteMemory = 0x16;
constexpr uint8_t kReportReadMemory = 0x17;
constexpr uint8_t kReportSpeakerMute = 0x19;
constexpr uint8_t kReportIr2 = 0x1A;
constexpr uint8_t kReportStatus = 0x20;
constexpr uint8_t kReportReadReply = 0x21;
constexpr uint8_t kReportAck = 0x22;

constexpr uint8_t kErrorNoDevice = 0x07;
constexpr uint8_t kErrorBadAddress = 0x08;

// Dolphin's rate for its emulated remote.
constexpr uint64_t kReportIntervalUs = 5000;
constexpr uint32_t kEepromSize = 0x1700;
constexpr uint8_t kBatteryFull = 0xC8;
constexpr uint8_t kAccelZero = 0x80;
constexpr uint8_t kAccelOneG = 0x9A;
constexpr uint8_t kSlaveSpeaker = 0xA2;
constexpr uint8_t kSlaveExtension = 0xA4;
constexpr uint8_t kSlaveCamera = 0xB0;
constexpr uint8_t kRegisterCalibration = 0x20;
constexpr uint8_t kRegisterKey = 0x40;
constexpr uint8_t kRegisterEncryption = 0xF0;
constexpr uint8_t kRegisterIdentifier = 0xFA;
constexpr uint8_t kEncryptionOn = 0xAA;
constexpr std::array<uint8_t, 6> kClassicId = {0x00, 0x00, 0xA4, 0x20, 0x01, 0x01};

// Calibration blocks end in checksums: the sum of the data plus 0x55, then 0x55 more for each further one.
template <size_t N>
constexpr std::array<uint8_t, N> WithChecksum(std::array<uint8_t, N> data, size_t checksumBytes) {
    uint8_t sum = 0x55;
    for (size_t i = 0; i < N - checksumBytes; ++i) sum = static_cast<uint8_t>(sum + data[i]);
    for (size_t i = N - checksumBytes; i < N; ++i, sum = static_cast<uint8_t>(sum + 0x55)) data[i] = sum;
    return data;
}

// Zero and one g on every axis; stored at 0x16 and again at 0x20.
constexpr std::array<uint8_t, 10> kAccelCalibration = WithChecksum<10>(
    {kAccelZero, kAccelZero, kAccelZero, 0, kAccelOneG, kAccelOneG, kAccelOneG, 0, 0, 0}, 1);

// Max, min and centre of each stick axis (left 6 bits, right 5 bits, scaled to 8), then the triggers at rest.
constexpr std::array<uint8_t, 16> kClassicCalibration = WithChecksum<16>(
    {0xFC, 0x04, 0x80, 0xFC, 0x04, 0x80, 0xF8, 0x08, 0x80, 0xF8, 0x08, 0x80, 0x00, 0x00, 0, 0}, 2);

struct Layout {
    size_t size;
    bool buttons;
    bool accel;
    size_t ir;
    size_t extension;
};

std::optional<Layout> LayoutFor(uint8_t mode) {
    switch (mode) {
    case 0x30: return Layout{3, true, false, 0, 0};
    case 0x31: return Layout{6, true, true, 0, 0};
    case 0x32: return Layout{11, true, false, 0, 8};
    case 0x33: return Layout{18, true, true, 12, 0};
    case 0x34: return Layout{22, true, false, 0, 19};
    case 0x35: return Layout{22, true, true, 0, 16};
    case 0x36: return Layout{22, true, false, 10, 9};
    case 0x37: return Layout{22, true, true, 10, 6};
    case 0x3D: return Layout{22, false, false, 0, 21};
    default: return std::nullopt;
    }
}

uint8_t StickValue(float value, int centre) {
    const long raw = std::lround(static_cast<float>(centre) + std::clamp(value, -1.0f, 1.0f) * (centre - 1));
    return static_cast<uint8_t>(std::clamp<long>(raw, 0, 2 * centre - 1));
}

uint8_t TriggerValue(float value) {
    return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 31.0f));
}

// The Classic Controller's six data bytes (format 1).
std::array<uint8_t, 6> ClassicData(const ClassicState& state) {
    const uint8_t lx = StickValue(state.leftX, 32);
    const uint8_t ly = StickValue(state.leftY, 32);
    const uint8_t rx = StickValue(state.rightX, 16);
    const uint8_t ry = StickValue(state.rightY, 16);
    const uint8_t lt = TriggerValue(state.triggerL);
    const uint8_t rt = TriggerValue(state.triggerR);
    const uint16_t buttons = static_cast<uint16_t>(~state.buttons); // active low
    return {
        static_cast<uint8_t>(((rx >> 3) & 0x03) << 6 | lx),
        static_cast<uint8_t>(((rx >> 1) & 0x03) << 6 | ly),
        static_cast<uint8_t>((rx & 0x01) << 7 | ((lt >> 3) & 0x03) << 5 | ry),
        static_cast<uint8_t>((lt & 0x07) << 5 | rt),
        static_cast<uint8_t>(buttons),
        static_cast<uint8_t>(buttons >> 8),
    };
}

uint8_t EepromByte(uint32_t address) {
    for (const uint32_t base : {0x16u, 0x20u}) {
        if (address >= base && address < base + kAccelCalibration.size()) return kAccelCalibration[address - base];
    }
    return 0;
}

} // namespace GamepadRemoteDetail

using namespace GamepadRemoteDetail;

GamepadRemoteHost::GamepadRemoteHost(Bluetooth::RemoteHost& remotes, GamepadSource& gamepads)
    : m_remotes(remotes), m_gamepads(gamepads) {
    for (size_t slot = 0; slot < m_emulated.size(); ++slot) ResetEmulated(slot);
}

void GamepadRemoteHost::ResetEmulated(size_t slot) {
    Emulated& remote = m_emulated[slot];
    if (remote.rumble) m_gamepads.Rumble(slot, false);
    const bool present = remote.present;
    const uint16_t buttons = remote.buttons;
    remote = Emulated{};
    remote.present = present;
    remote.buttons = buttons;
    std::copy(kClassicCalibration.begin(), kClassicCalibration.end(), remote.extension.begin() + kRegisterCalibration);
    std::copy(kClassicCalibration.begin(), kClassicCalibration.end(),
              remote.extension.begin() + kRegisterCalibration + kClassicCalibration.size());
    std::copy(kClassicId.begin(), kClassicId.end(), remote.extension.begin() + kRegisterIdentifier);
}

GamepadRemoteHost::Source GamepadRemoteHost::SourceFor(size_t slot) const {
    if (m_remotes.IsAttached(slot)) return Source::Remote;
    ClassicState state;
    return m_gamepads.Read(slot, state) ? Source::Gamepad : Source::None;
}

// A connection stays with what it was made with; the other one takes the slot after a reconnect.
bool GamepadRemoteHost::Physical(size_t slot) const {
    const Source linked = m_emulated[slot].linkedTo;
    return linked != Source::None ? linked == Source::Remote : m_remotes.IsAttached(slot);
}

bool GamepadRemoteHost::IsAttached(size_t slot) const {
    const Source source = SourceFor(slot);
    const Source linked = m_emulated[slot].linkedTo;
    return source != Source::None && (linked == Source::None || linked == source);
}

bool GamepadRemoteHost::ConsumeWake(size_t slot) {
    Emulated& remote = m_emulated[slot];
    ClassicState state;
    const bool present = m_gamepads.Read(slot, state);
    const bool assigned = present && !remote.present;
    const bool pressed = present && (state.buttons & ~remote.buttons) != 0;
    remote.present = present;
    remote.buttons = present ? state.buttons : 0;
    if (Physical(slot)) return m_remotes.ConsumeWake(slot);
    return assigned || (pressed && remote.link == LinkState::Down);
}

bool GamepadRemoteHost::PopInputReport(size_t slot, std::vector<uint8_t>& report) {
    if (Physical(slot)) return m_remotes.PopInputReport(slot, report);
    Emulated& remote = m_emulated[slot];
    if (!remote.replies.empty()) {
        report = std::move(remote.replies.front());
        remote.replies.pop_front();
        return true;
    }
    const uint64_t now = m_gamepads.NowUs();
    if (remote.link != LinkState::Ready || !LayoutFor(remote.mode) || now < remote.nextReportUs) return false;
    remote.nextReportUs = now + kReportIntervalUs;
    report = DataReport(slot);
    return true;
}

std::vector<uint8_t> GamepadRemoteHost::DataReport(size_t slot) {
    Emulated& remote = m_emulated[slot];
    ClassicState state;
    if (m_gamepads.Read(slot, state)) {
        remote.buttons = state.buttons;
    } else {
        state = {};
    }
    const Layout layout = *LayoutFor(remote.mode);
    std::vector<uint8_t> report(layout.size, 0);
    report[0] = remote.mode;
    size_t at = layout.buttons ? 3 : 1;
    if (layout.accel) {
        // Lying flat: gravity on Z only.
        report[at] = kAccelZero;
        report[at + 1] = kAccelZero;
        report[at + 2] = kAccelOneG;
        at += 3;
    }
    std::fill_n(report.begin() + static_cast<std::ptrdiff_t>(at), layout.ir, 0xFF); // no IR objects
    at += layout.ir;
    if (layout.extension != 0) {
        const auto data = ClassicData(state);
        std::copy(data.begin(), data.end(), report.begin() + static_cast<std::ptrdiff_t>(at));
        if (remote.extension[kRegisterEncryption] == kEncryptionOn && remote.tables) {
            remote.tables->Encrypt(report.data() + at, 0, layout.extension);
        }
    }
    return report;
}

void GamepadRemoteHost::SendOutputReport(size_t slot, const uint8_t* report, size_t size) {
    if (Physical(slot)) {
        m_remotes.SendOutputReport(slot, report, size);
        return;
    }
    if (size < 2) return;
    Emulated& remote = m_emulated[slot];
    const bool rumble = (report[1] & 0x01) != 0;
    if (rumble != remote.rumble) {
        remote.rumble = rumble;
        m_gamepads.Rumble(slot, rumble);
    }
    switch (report[0]) {
    case kReportLeds: remote.leds = report[1] >> 4; break;
    case kReportMode:
        if (size >= 3 && LayoutFor(report[2])) remote.mode = report[2];
        break;
    case kReportIr: remote.ir = (report[1] & 0x04) != 0; break;
    case kReportSpeaker: remote.speaker = (report[1] & 0x04) != 0; break;
    case kReportSpeakerMute:
    case kReportIr2: break;
    case kReportStatusRequest: QueueStatus(remote); return;
    case kReportWriteMemory: WriteMemory(remote, report, size); return;
    case kReportReadMemory:
        if (size >= 7) ReadMemory(remote, report);
        return;
    default: return; // rumble and speaker data are never acknowledged
    }
    if ((report[1] & 0x02) != 0) remote.replies.push_back({kReportAck, 0, 0, report[0], 0x00});
}

void GamepadRemoteHost::QueueStatus(Emulated& remote) {
    const uint8_t flags = static_cast<uint8_t>(remote.leds << 4 | (remote.ir ? 0x08 : 0) |
                                               (remote.speaker ? 0x04 : 0) | 0x02); // extension attached
    remote.replies.push_back({kReportStatus, 0, 0, flags, 0, 0, kBatteryFull});
}

void GamepadRemoteHost::WriteMemory(Emulated& remote, const uint8_t* report, size_t size) {
    if (size < 7) return;
    const uint32_t address = (report[2] << 16) | (report[3] << 8) | report[4];
    const size_t length = std::min<size_t>({report[5], 16, size - 6});
    // EEPROM, speaker and camera writes are accepted and ignored.
    if ((report[1] & 0x04) != 0 && ((address >> 16) & 0xFE) == kSlaveExtension) {
        const uint8_t first = static_cast<uint8_t>(address);
        for (size_t i = 0; i < length; ++i) remote.extension[(first + i) & 0xFF] = report[6 + i];
        if (first < kRegisterKey + 16 && first + length > kRegisterKey) {
            std::array<uint8_t, 16> key{};
            std::copy_n(remote.extension.begin() + kRegisterKey, key.size(), key.begin());
            remote.tables = ExtCrypto::FromKey(key);
        }
    }
    remote.replies.push_back({kReportAck, 0, 0, kReportWriteMemory, 0x00});
}

void GamepadRemoteHost::ReadMemory(Emulated& remote, const uint8_t* report) {
    const bool registers = (report[1] & 0x04) != 0;
    uint32_t address = (report[2] << 16) | (report[3] << 8) | report[4];
    uint32_t length = (report[5] << 8) | report[6];
    const uint8_t slave = static_cast<uint8_t>((address >> 16) & 0xFE);
    uint8_t error = 0;
    if (registers && slave != kSlaveSpeaker && slave != kSlaveExtension && slave != kSlaveCamera) {
        error = kErrorNoDevice; // no Motion Plus
    } else if (!registers && address + length > kEepromSize) {
        error = kErrorBadAddress;
    }
    if (error != 0) {
        std::vector<uint8_t> reply(22, 0);
        reply[0] = kReportReadReply;
        reply[3] = error;
        reply[4] = static_cast<uint8_t>(address >> 8);
        reply[5] = static_cast<uint8_t>(address);
        remote.replies.push_back(std::move(reply));
        return;
    }
    const bool encrypted = registers && slave == kSlaveExtension &&
                           remote.extension[kRegisterEncryption] == kEncryptionOn && remote.tables;
    while (length != 0) {
        const uint32_t chunk = std::min<uint32_t>(length, 16);
        std::vector<uint8_t> reply(22, 0);
        reply[0] = kReportReadReply;
        reply[3] = static_cast<uint8_t>((chunk - 1) << 4);
        reply[4] = static_cast<uint8_t>(address >> 8);
        reply[5] = static_cast<uint8_t>(address);
        for (uint32_t i = 0; i < chunk; ++i) {
            if (!registers) {
                reply[6 + i] = EepromByte(address + i);
            } else if (slave == kSlaveExtension) {
                reply[6 + i] = remote.extension[(address + i) & 0xFF];
            }
        }
        if (encrypted) remote.tables->Encrypt(reply.data() + 6, address & 0xFF, chunk);
        remote.replies.push_back(std::move(reply));
        address += chunk;
        length -= chunk;
    }
}

void GamepadRemoteHost::OnLinkStateChanged(size_t slot, Bluetooth::LinkState state) {
    Emulated& remote = m_emulated[slot];
    if (state != LinkState::Down && remote.linkedTo == Source::None) {
        remote.linkedTo = m_remotes.IsAttached(slot) ? Source::Remote : Source::Gamepad;
    }
    const bool gamepadLink = remote.linkedTo == Source::Gamepad;
    remote.link = state;
    if (state == LinkState::Down) ResetEmulated(slot);
    // A remote reports its extension once linked, as Dolphin's does; reporting stays off until the game asks.
    if (state == LinkState::Ready && gamepadLink) QueueStatus(remote);
    // A gamepad's link stays its own: passed on, its Ready would mark a sleeping remote in the slot connected.
    if (!gamepadLink) m_remotes.OnLinkStateChanged(slot, state);
}

void GamepadRemoteHost::OnGuestDisconnect(size_t slot) {
    if (Physical(slot)) m_remotes.OnGuestDisconnect(slot);
}

void GamepadRemoteHost::OnGuestInquiry(bool active) {
    m_remotes.OnGuestInquiry(active);
}

const char* GamepadRemoteHost::DeviceName(size_t slot) const {
    return Physical(slot) ? m_remotes.DeviceName(slot) : "Nintendo RVL-CNT-01";
}

} // namespace Wiimote
