// Verifies which controller serves a Wii channel (ChooseChannelSource).

#include "wii_remote_input.h"

#include <cstdio>

using WiiRemoteInput::ChannelSource;
using WiiRemoteInput::ChooseChannelSource;
using WiiRemoteInput::IndexContents;
using WiiRemoteInput::Kind;

static int g_failures = 0;

static void Check(bool ok, const char* what) {
    if (!ok) {
        std::printf("  FAIL: %s\n", what);
        ++g_failures;
    }
}

static void Expect(ChannelSource got, Kind kind, int index, bool live, const char* what) {
    Check(got.kind == kind && got.index == index && got.live == live, what);
}

int main() {
    constexpr uint32_t chan = 2;
    constexpr int gc = 2;
    constexpr int wii = 6;
    const IndexContents empty{};
    const IndexContents pad{true, Kind::NotWii};
    const IndexContents remote{true, Kind::Remote};
    const IndexContents classicRemote{true, Kind::RemoteWithClassic};
    const IndexContents wiiUPro{true, Kind::WiiUPro};
    const ChannelSource none{};

    Expect(ChooseChannelSource(chan, remote, pad, none, false), Kind::Remote, gc, true,
           "a remote on the GameCube index wins over a pad on the Wii index");
    Expect(ChooseChannelSource(chan, pad, pad, none, false), Kind::EmulatedClassic, wii, true,
           "a pad on the Wii index is an emulated Classic beside a GameCube pad");
    Expect(ChooseChannelSource(chan, empty, pad, none, false), Kind::EmulatedClassic, wii, true,
           "a pad on the Wii index is an emulated Classic alone");
    Expect(ChooseChannelSource(chan, pad, classicRemote, none, false), Kind::RemoteWithClassic, wii, true,
           "a remote on the Wii index stays a remote");
    Expect(ChooseChannelSource(chan, empty, wiiUPro, none, false), Kind::EmulatedClassic, wii, true,
           "a Wii U Pro Controller on the Wii index is an emulated Classic");
    Expect(ChooseChannelSource(chan, wiiUPro, empty, none, false), Kind::NotWii, -1, false,
           "a Wii U Pro Controller on the GameCube index serves no channel");
    Expect(ChooseChannelSource(chan, pad, empty, none, false), Kind::NotWii, -1, false,
           "a GameCube pad alone serves no channel");
    Expect(ChooseChannelSource(chan, empty, empty, none, false), Kind::NotWii, -1, false, "nothing anywhere");

    const ChannelSource rememberedAtGc{Kind::RemoteWithNunchuk, gc, true};
    const ChannelSource rememberedAtWii{Kind::RemoteWithClassic, wii, true};
    Expect(ChooseChannelSource(chan, empty, empty, rememberedAtGc, true), Kind::RemoteWithNunchuk, gc, false,
           "a swap on the GameCube index keeps the remembered kind");
    Expect(ChooseChannelSource(chan, empty, empty, rememberedAtGc, false), Kind::NotWii, -1, false,
           "the remembered kind ends with the grace");
    Expect(ChooseChannelSource(chan, pad, empty, rememberedAtGc, true), Kind::NotWii, -1, false,
           "a pad taking the GameCube index ends the swap");
    Expect(ChooseChannelSource(chan, pad, empty, rememberedAtWii, true), Kind::RemoteWithClassic, wii, false,
           "a pad on the GameCube index leaves a swap on the Wii index alone");
    Expect(ChooseChannelSource(chan, empty, empty, ChannelSource{Kind::WiiUPro, gc, true}, true), Kind::NotWii, -1,
           false, "only kinds the game reads through KPAD are remembered");

    if (g_failures == 0) {
        std::printf("all checks passed\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
