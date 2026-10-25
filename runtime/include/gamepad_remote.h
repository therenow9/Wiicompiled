#pragma once

// Gamepads as Wii Remotes. A slot of the emulated Bluetooth controller with no
// physical remote on it can be served by a gamepad, which the game then sees as
// a Wii Remote (RVL-CNT-01) lying still with a Classic Controller attached. Pure
// host code: the gamepads come in through GamepadSource, so it is unit-testable.

#include "bluetooth/bt_controller.h"
#include "wiimote_ext_crypto.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace Wiimote {

// Classic Controller buttons, as the extension reports them (before inversion).
namespace ClassicButton {
constexpr uint16_t R = 0x0002;
constexpr uint16_t Plus = 0x0004;
constexpr uint16_t Home = 0x0008;
constexpr uint16_t Minus = 0x0010;
constexpr uint16_t L = 0x0020;
constexpr uint16_t Down = 0x0040;
constexpr uint16_t Right = 0x0080;
constexpr uint16_t Up = 0x0100;
constexpr uint16_t Left = 0x0200;
constexpr uint16_t ZR = 0x0400;
constexpr uint16_t X = 0x0800;
constexpr uint16_t A = 0x1000;
constexpr uint16_t Y = 0x2000;
constexpr uint16_t B = 0x4000;
constexpr uint16_t ZL = 0x8000;
} // namespace ClassicButton

struct ClassicState {
    uint16_t buttons = 0;        // ClassicButton bits, set while held
    float leftX = 0, leftY = 0;  // -1..1, up and right positive
    float rightX = 0, rightY = 0;
    float triggerL = 0, triggerR = 0; // 0..1
};

class GamepadSource {
public:
    virtual ~GamepadSource() = default;
    // The gamepad assigned to `slot`, false when there is none.
    virtual bool Read(size_t slot, ClassicState& state) = 0;
    virtual void Rumble(size_t slot, bool on) = 0;
    virtual uint64_t NowUs() = 0;
};

// Answers each slot from `remotes` while a physical remote is attached there,
// otherwise from the gamepad `gamepads` has for it.
class GamepadRemoteHost final : public Bluetooth::RemoteHost {
public:
    GamepadRemoteHost(Bluetooth::RemoteHost& remotes, GamepadSource& gamepads);

    bool IsAttached(size_t slot) const override;
    bool ConsumeWake(size_t slot) override;
    bool PopInputReport(size_t slot, std::vector<uint8_t>& report) override;
    void SendOutputReport(size_t slot, const uint8_t* report, size_t size) override;
    void OnLinkStateChanged(size_t slot, Bluetooth::LinkState state) override;
    void OnGuestDisconnect(size_t slot) override;
    void OnGuestInquiry(bool active) override;
    const char* DeviceName(size_t slot) const override;

private:
    enum class Source : uint8_t { None, Remote, Gamepad };

    struct Emulated {
        bool present = false;
        uint16_t buttons = 0;
        Source linkedTo = Source::None;
        Bluetooth::LinkState link = Bluetooth::LinkState::Down;
        uint8_t mode = 0; // no data reports until the game picks a mode
        uint8_t leds = 0;
        bool rumble = false;
        bool ir = false;
        bool speaker = false;
        std::array<uint8_t, 256> extension{};
        std::optional<ExtCrypto::Tables> tables;
        std::deque<std::vector<uint8_t>> replies;
        uint64_t nextReportUs = 0;
    };

    Source SourceFor(size_t slot) const;
    bool Physical(size_t slot) const;
    void ResetEmulated(size_t slot);
    static void QueueStatus(Emulated& remote);
    void ReadMemory(Emulated& remote, const uint8_t* report);
    void WriteMemory(Emulated& remote, const uint8_t* report, size_t size);
    std::vector<uint8_t> DataReport(size_t slot);

    Bluetooth::RemoteHost& m_remotes;
    GamepadSource& m_gamepads;
    std::array<Emulated, Bluetooth::kRemoteSlots> m_emulated;
};

} // namespace Wiimote
