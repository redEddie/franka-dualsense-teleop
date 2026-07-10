// Wire protocol between the Python teleop client and the libfranka bridge.
// MUST stay in sync with src/dsfranka/real/franka_client.py (CMD_FMT / STATE_FMT).
#pragma once
#include <cstdint>

namespace dsfranka {

constexpr uint32_t kMagic = 0x44534652;  // "DSFR"
constexpr int kCmdPort = 5555;           // bridge listens here
constexpr int kStatePort = 5556;         // client listens here

#pragma pack(push, 1)
struct CmdPacket {                       // python: "<II8dB"
  uint32_t magic;
  uint32_t seq;
  double q[7];        // joint position targets [rad]
  double gripper;     // 0 = closed, 1 = fully open
  uint8_t flags;
};

struct StatePacket {                     // python: "<II22dB"
  uint32_t magic;
  uint32_t seq;
  double q[7];
  double dq[7];
  double ee_pos[3];   // from O_T_EE
  double ee_quat[4];  // wxyz
  double gripper_width;
  uint8_t robot_mode;
};
#pragma pack(pop)

}  // namespace dsfranka
