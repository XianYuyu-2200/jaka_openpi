#pragma once

#include <cstddef>
#include <cstdint>

namespace jaka_recording_ipc {

constexpr std::uint32_t kMagic = 0x31434552;  // "REC1"
constexpr std::uint16_t kVersion = 1;

#pragma pack(push, 1)
struct TeleopSnapshotPacket {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t size;
    std::uint64_t sequence;
    std::uint64_t monotonic_ns;
    double actual_arm[6];
    double action_arm[6];
    std::uint16_t actual_hand[6];
    std::uint16_t action_hand[6];
    std::int32_t arm_state_ret;
    std::int32_t hand_state_ret;
    std::uint8_t operator_dragging;
    std::uint8_t reserved[3];
};
#pragma pack(pop)

static_assert(sizeof(TeleopSnapshotPacket) == 156);

inline bool valid_packet(const TeleopSnapshotPacket& packet) {
    return packet.magic == kMagic && packet.version == kVersion &&
           packet.size == sizeof(TeleopSnapshotPacket);
}

}  // namespace jaka_recording_ipc
