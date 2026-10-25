#include "bluetooth/bt_controller.h"
#include "gamepad_remote.h"
#include "wiimote_ext_crypto.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace {

using Bytes = std::vector<uint8_t>;
using Bluetooth::Controller;
using Bluetooth::LinkState;
using Wiimote::ClassicState;

int g_failures = 0;

#define CHECK(cond)                                                                    \
    do {                                                                               \
        if (!(cond)) {                                                                 \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                              \
        }                                                                              \
    } while (0)

struct FakeRemotes : Bluetooth::RemoteHost {
    std::array<bool, Bluetooth::kRemoteSlots> attached{};
    std::array<bool, Bluetooth::kRemoteSlots> wake{};
    std::array<std::deque<Bytes>, Bluetooth::kRemoteSlots> input;
    std::vector<std::pair<size_t, Bytes>> output;
    std::vector<std::pair<size_t, LinkState>> links;

    bool IsAttached(size_t slot) const override { return attached[slot]; }
    bool ConsumeWake(size_t slot) override {
        const bool w = wake[slot];
        wake[slot] = false;
        return w;
    }
    bool PopInputReport(size_t slot, Bytes& report) override {
        if (input[slot].empty()) return false;
        report = input[slot].front();
        input[slot].pop_front();
        return true;
    }
    void SendOutputReport(size_t slot, const uint8_t* report, size_t size) override {
        output.emplace_back(slot, Bytes(report, report + size));
    }
    void OnLinkStateChanged(size_t slot, LinkState state) override { links.emplace_back(slot, state); }
    void OnGuestDisconnect(size_t) override {}
    void OnGuestInquiry(bool) override {}
    const char* DeviceName(size_t) const override { return "Nintendo RVL-CNT-01-TR"; }
};

struct FakePads : Wiimote::GamepadSource {
    std::array<bool, Bluetooth::kRemoteSlots> present{};
    std::array<ClassicState, Bluetooth::kRemoteSlots> state{};
    std::vector<std::pair<size_t, bool>> rumble;
    uint64_t now = 0;

    bool Read(size_t slot, ClassicState& out) override {
        out = state[slot];
        return present[slot];
    }
    void Rumble(size_t slot, bool on) override { rumble.emplace_back(slot, on); }
    uint64_t NowUs() override { return now; }
};

uint16_t Le16(const Bytes& b, size_t i) {
    return static_cast<uint16_t>(b[i] | (b[i + 1] << 8));
}

void PutLe16(Bytes& b, uint16_t v) {
    b.push_back(static_cast<uint8_t>(v));
    b.push_back(static_cast<uint8_t>(v >> 8));
}

struct Guest {
    Controller& controller;
    uint64_t now = 1000;

    std::vector<Bytes> Run(uint16_t opcode, Bytes params = {}) {
        Bytes packet;
        PutLe16(packet, opcode);
        packet.push_back(static_cast<uint8_t>(params.size()));
        packet.insert(packet.end(), params.begin(), params.end());
        controller.ExecuteCommand(packet.data(), packet.size(), now);
        return Events();
    }
    std::vector<Bytes> Events() {
        std::vector<Bytes> events;
        Bytes event;
        while (controller.PopEvent(event)) events.push_back(event);
        return events;
    }
    std::vector<Bytes> Acl() {
        std::vector<Bytes> packets;
        Bytes packet;
        while (controller.PopAcl(packet)) packets.push_back(packet);
        return packets;
    }
    void Tick() {
        now += 1000;
        controller.Tick(now);
    }
    void SendL2cap(uint16_t handle, uint16_t cid, const Bytes& payload) {
        Bytes packet;
        PutLe16(packet, static_cast<uint16_t>(handle | 0x2000));
        PutLe16(packet, static_cast<uint16_t>(payload.size() + 4));
        PutLe16(packet, static_cast<uint16_t>(payload.size()));
        PutLe16(packet, cid);
        packet.insert(packet.end(), payload.begin(), payload.end());
        controller.ReceiveAcl(packet.data(), packet.size(), now);
    }
    void Signal(uint16_t handle, uint8_t code, uint8_t ident, const Bytes& data) {
        Bytes payload = {code, ident};
        PutLe16(payload, static_cast<uint16_t>(data.size()));
        payload.insert(payload.end(), data.begin(), data.end());
        SendL2cap(handle, 0x0001, payload);
    }
};

// Opens the HID channel the remote requests and completes both configurations; returns the remote's CID.
uint16_t AcceptRemoteChannel(Guest& guest, uint16_t handle, uint16_t guestCid) {
    guest.Tick();
    guest.Events();
    const auto acl = guest.Acl();
    CHECK(acl.size() == 1 && acl[0].size() >= 16 && acl[0][8] == 0x02);
    if (acl.size() != 1 || acl[0].size() < 16) return 0;
    const uint8_t ident = acl[0][9];
    const uint16_t remoteCid = Le16(acl[0], 14);
    Bytes rsp;
    PutLe16(rsp, guestCid);
    PutLe16(rsp, remoteCid);
    PutLe16(rsp, 0);
    PutLe16(rsp, 0);
    guest.Signal(handle, 0x03, ident, rsp);
    const auto config = guest.Acl();
    CHECK(config.size() == 1 && config[0].size() >= 10 && config[0][8] == 0x04);
    Bytes cfg;
    PutLe16(cfg, remoteCid);
    PutLe16(cfg, 0);
    guest.Signal(handle, 0x04, 0x40, cfg);
    guest.Acl();
    Bytes ourCfgRsp;
    PutLe16(ourCfgRsp, remoteCid);
    PutLe16(ourCfgRsp, 0);
    PutLe16(ourCfgRsp, 0);
    if (config.size() == 1 && config[0].size() >= 10) guest.Signal(handle, 0x05, config[0][9], ourCfgRsp);
    return remoteCid;
}

// Pages in the remote on `slot` the way WUD does and opens its HID channels. Returns the interrupt CID.
uint16_t Connect(Guest& guest, size_t slot) {
    guest.Tick();
    auto e = guest.Events();
    CHECK(e.size() == 1 && e[0][0] == 0x04);
    const Bluetooth::BdAddr addr = Controller::RemoteAddress(slot);
    Bytes accept(addr.begin(), addr.end());
    accept.push_back(0x00);
    guest.Run(0x0409, accept);
    const uint16_t handle = Controller::ConnectionHandle(slot);
    AcceptRemoteChannel(guest, handle, 0x0041);
    return AcceptRemoteChannel(guest, handle, 0x0042);
}

// Input reports the remote sent on its interrupt channel, without the HID header.
std::vector<Bytes> Reports(Guest& guest) {
    guest.Tick();
    guest.Events();
    std::vector<Bytes> reports;
    for (const Bytes& p : guest.Acl()) {
        if (p.size() > 9 && p[8] == 0xA1) reports.emplace_back(p.begin() + 9, p.end());
    }
    return reports;
}

const Bytes* Find(const std::vector<Bytes>& reports, uint8_t id) {
    for (const Bytes& r : reports) {
        if (r[0] == id) return &r;
    }
    return nullptr;
}

Bytes Output(std::initializer_list<uint8_t> report) {
    Bytes payload = {0xA2};
    payload.insert(payload.end(), report.begin(), report.end());
    return payload;
}

Bytes WriteRegister(uint8_t reg, const Bytes& data) {
    Bytes payload = {0xA2, 0x16, 0x04, 0xA4, 0x00, reg, static_cast<uint8_t>(data.size())};
    payload.insert(payload.end(), data.begin(), data.end());
    payload.resize(23, 0);
    return payload;
}

// A key the game's key schedule produces (rand 11..AA, index 2), in register order.
constexpr std::array<uint8_t, 16> kKey = {0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33,
                                          0x22, 0x11, 0x5E, 0x7D, 0xD8, 0xAB, 0xBD, 0xFE};

void TestGamepadRemote() {
    FakeRemotes remotes;
    FakePads pads;
    Wiimote::GamepadRemoteHost host(remotes, pads);
    Controller controller(host);
    Guest guest{controller};
    controller.Open(guest.now);
    guest.Run(0x0C03);
    guest.Run(0x0C1A, {0x02});

    // Nothing on the slot: nothing connects.
    guest.Tick();
    CHECK(guest.Events().empty());
    CHECK(!host.IsAttached(0));

    pads.present[0] = true;
    CHECK(host.IsAttached(0));
    const uint16_t cid = Connect(guest, 0);
    CHECK(cid != 0 && controller.SlotLinkState(0) == LinkState::Ready);
    CHECK(std::string(host.DeviceName(0)) == "Nintendo RVL-CNT-01");

    // On connecting it reports the extension, then nothing until the game picks a mode.
    auto reports = Reports(guest);
    CHECK(reports.size() == 1 && reports[0].size() == 7 && reports[0][0] == 0x20 && reports[0][3] == 0x02);
    guest.SendL2cap(0x0100, cid, Output({0x12, 0x00, 0x30}));
    reports = Reports(guest);
    CHECK(reports.size() == 1 && reports[0] == Bytes({0x30, 0x00, 0x00}));

    // Status on request: extension attached, the LEDs the game set.
    guest.SendL2cap(0x0100, cid, Output({0x11, 0x10}));
    guest.SendL2cap(0x0100, cid, Output({0x15, 0x00}));
    reports = Reports(guest);
    const Bytes* status = Find(reports, 0x20);
    CHECK(status != nullptr && status->size() == 7 && ((*status)[3] & 0x02) != 0 && ((*status)[3] >> 4) == 0x1);
    CHECK(Find(reports, 0x30) == nullptr);

    // The extension's init, then its identifier and calibration.
    guest.SendL2cap(0x0100, cid, WriteRegister(0xF0, {0x55}));
    guest.SendL2cap(0x0100, cid, WriteRegister(0xFB, {0x00}));
    guest.SendL2cap(0x0100, cid, Output({0x17, 0x04, 0xA4, 0x00, 0xFA, 0x00, 0x06}));
    guest.SendL2cap(0x0100, cid, Output({0x17, 0x04, 0xA4, 0x00, 0x20, 0x00, 0x10}));
    reports = Reports(guest);
    int acks = 0;
    std::vector<Bytes> reads;
    for (const Bytes& r : reports) {
        if (r[0] == 0x22 && r.size() == 5 && r[3] == 0x16 && r[4] == 0x00) ++acks;
        if (r[0] == 0x21) reads.push_back(r);
    }
    CHECK(acks == 2);
    CHECK(reads.size() == 2);
    if (reads.size() == 2) {
        CHECK(reads[0][3] == 0x50 && reads[0][4] == 0x00 && reads[0][5] == 0xFA);
        CHECK(Bytes(reads[0].begin() + 6, reads[0].begin() + 12) == Bytes({0x00, 0x00, 0xA4, 0x20, 0x01, 0x01}));
        CHECK(reads[1][3] == 0xF0 && reads[1][5] == 0x20);
        uint8_t sum = 0x55;
        for (int i = 6; i < 20; ++i) sum = static_cast<uint8_t>(sum + reads[1][i]);
        CHECK(reads[1][8] == 0x80 && reads[1][20] == sum && reads[1][21] == static_cast<uint8_t>(sum + 0x55));
    }

    // A Classic report for a known pad state.
    guest.SendL2cap(0x0100, cid, Output({0x12, 0x04, 0x37}));
    pads.state[0].buttons = Wiimote::ClassicButton::A | Wiimote::ClassicButton::ZL;
    pads.state[0].leftX = 1.0f;
    pads.state[0].rightX = -1.0f;
    pads.state[0].triggerL = 1.0f;
    pads.now += 5000;
    reports = Reports(guest);
    const Bytes* data = Find(reports, 0x37);
    const Bytes expected = {0x3F, 0x20, 0xF0, 0xE0, 0xFF, 0x6F};
    CHECK(data != nullptr && data->size() == 22);
    if (data != nullptr && data->size() == 22) {
        CHECK((*data)[1] == 0 && (*data)[2] == 0 && (*data)[3] == 0x80 && (*data)[4] == 0x80 && (*data)[5] == 0x9A);
        CHECK((*data)[6] == 0xFF && (*data)[15] == 0xFF);
        CHECK(Bytes(data->begin() + 16, data->end()) == expected);
    }

    // The same state once the game has turned encryption on.
    Bytes key(kKey.begin(), kKey.begin() + 6);
    guest.SendL2cap(0x0100, cid, WriteRegister(0x40, key));
    guest.SendL2cap(0x0100, cid, WriteRegister(0x46, Bytes(kKey.begin() + 6, kKey.begin() + 12)));
    guest.SendL2cap(0x0100, cid, WriteRegister(0x4C, Bytes(kKey.begin() + 12, kKey.end())));
    guest.SendL2cap(0x0100, cid, WriteRegister(0xF0, {0xAA}));
    pads.now += 5000;
    reports = Reports(guest);
    data = Find(reports, 0x37);
    const auto tables = Wiimote::ExtCrypto::FromKey(kKey);
    CHECK(tables.has_value() && data != nullptr && data->size() == 22);
    if (tables && data != nullptr && data->size() == 22) {
        Bytes plain(data->begin() + 16, data->end());
        CHECK(plain != expected);
        tables->Decrypt(plain.data(), 0, plain.size());
        CHECK(plain == expected);
    }

    // Rumble reaches the pad, and stops when the game lets go.
    guest.SendL2cap(0x0100, cid, Output({0x10, 0x01}));
    CHECK(pads.rumble.size() == 1 && pads.rumble[0] == std::make_pair(size_t{0}, true));
    guest.Run(0x0406, {0x00, 0x01, 0x13});
    CHECK(pads.rumble.size() == 2 && pads.rumble[1] == std::make_pair(size_t{0}, false));
    CHECK(controller.SlotLinkState(0) == LinkState::Down);
    CHECK(remotes.links.empty());  // the real remotes never saw the pad's link

    // A button press on the idle pad asks to connect again.
    guest.Tick();
    CHECK(guest.Events().empty());
    pads.state[0].buttons = Wiimote::ClassicButton::B;
    guest.Tick();
    auto e = guest.Events();
    CHECK(e.size() == 1 && e[0][0] == 0x04);
}

void TestRemoteWins() {
    FakeRemotes remotes;
    FakePads pads;
    Wiimote::GamepadRemoteHost host(remotes, pads);
    Controller controller(host);
    Guest guest{controller};
    controller.Open(guest.now);
    guest.Run(0x0C03);
    guest.Run(0x0C1A, {0x02});

    pads.present[1] = true;
    remotes.attached[1] = true;
    remotes.wake[1] = true;
    const uint16_t cid = Connect(guest, 1);
    CHECK(cid != 0 && controller.SlotLinkState(1) == LinkState::Ready);
    CHECK(!remotes.links.empty() && remotes.links.back() == std::make_pair(size_t{1}, LinkState::Ready));
    CHECK(std::string(host.DeviceName(1)) == "Nintendo RVL-CNT-01-TR");

    remotes.input[1].push_back({0x30, 0x00, 0x08});
    pads.now += 5000;
    const auto reports = Reports(guest);
    CHECK(reports.size() == 1 && reports[0] == Bytes({0x30, 0x00, 0x08}));
    guest.SendL2cap(0x0101, cid, Output({0x15, 0x00}));
    CHECK(remotes.output.size() == 1 && remotes.output[0].second == Bytes({0x15, 0x00}));
    CHECK(Reports(guest).empty());
}

} // namespace

int main() {
    TestGamepadRemote();
    TestRemoteWins();
    if (g_failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", g_failures);
        return EXIT_FAILURE;
    }
    std::printf("gamepad remote tests passed\n");
    return EXIT_SUCCESS;
}
