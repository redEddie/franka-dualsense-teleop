// franka_bridge: UDP <-> libfranka realtime loop.
//
// Receives joint-position targets (~50-100 Hz) from the Python teleop client,
// tracks them in a 1 kHz JointPositions control loop with exponential smoothing
// and per-tick rate clamping, and publishes robot state back over UDP.
//
// Usage: ./franka_bridge <robot-ip>
//
// Safety notes:
//  - keep the external activation device (user stop) in hand at all times
//  - the loop refuses to start until a first command packet arrives that is
//    close to the current robot configuration (client syncs from state first)

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include "udp_protocol.hpp"

using dsfranka::CmdPacket;
using dsfranka::StatePacket;

namespace {

constexpr double kSmoothing = 0.995;      // per-ms exponential filter on targets
constexpr double kMaxDqPerTick = 0.001;   // rad per 1 ms tick (~1 rad/s) hard clamp
constexpr double kStartTolerance = 0.05;  // rad: first target must be near current q

std::mutex g_mtx;
std::array<double, 7> g_target_q{};
double g_target_gripper = 1.0;
std::atomic<bool> g_have_cmd{false};
std::atomic<bool> g_running{true};

std::mutex g_state_mtx;
StatePacket g_state{};

void udp_rx_loop(int sock) {
  CmdPacket cmd;
  while (g_running) {
    ssize_t n = recv(sock, &cmd, sizeof(cmd), 0);
    if (n != sizeof(cmd) || cmd.magic != dsfranka::kMagic) continue;
    std::lock_guard<std::mutex> lk(g_mtx);
    std::memcpy(g_target_q.data(), cmd.q, sizeof(cmd.q));
    g_target_gripper = cmd.gripper;
    g_have_cmd = true;
  }
}

void state_tx_loop(int sock, sockaddr_in client) {
  while (g_running) {
    {
      std::lock_guard<std::mutex> lk(g_state_mtx);
      if (g_state.magic == dsfranka::kMagic) {
        sendto(sock, &g_state, sizeof(g_state), 0,
               reinterpret_cast<sockaddr*>(&client), sizeof(client));
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz
  }
}

void gripper_loop(franka::Gripper* gripper) {
  double last_cmd = -1.0;
  while (g_running) {
    double target;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      target = g_target_gripper;
    }
    if (g_have_cmd && std::abs(target - last_cmd) > 0.05) {
      last_cmd = target;
      try {
        if (target < 0.5) {
          gripper->grasp(0.0, 0.1, 40.0, 0.08, 0.08);  // close & grasp
        } else {
          gripper->move(0.08 * target, 0.1);
        }
      } catch (const franka::Exception& e) {
        std::cerr << "[gripper] " << e.what() << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void publish_state(const franka::RobotState& rs, double width, uint32_t seq) {
  StatePacket s{};
  s.magic = dsfranka::kMagic;
  s.seq = seq;
  for (int i = 0; i < 7; i++) { s.q[i] = rs.q[i]; s.dq[i] = rs.dq[i]; }
  // O_T_EE is column-major 4x4
  const auto& T = rs.O_T_EE;
  s.ee_pos[0] = T[12]; s.ee_pos[1] = T[13]; s.ee_pos[2] = T[14];
  // rotation matrix -> quaternion (wxyz)
  double r11 = T[0], r21 = T[1], r31 = T[2];
  double r12 = T[4], r22 = T[5], r32 = T[6];
  double r13 = T[8], r23 = T[9], r33 = T[10];
  double tr = r11 + r22 + r33;
  double qw, qx, qy, qz;
  if (tr > 0) {
    double s4 = std::sqrt(tr + 1.0) * 2;
    qw = 0.25 * s4; qx = (r32 - r23) / s4; qy = (r13 - r31) / s4; qz = (r21 - r12) / s4;
  } else if (r11 > r22 && r11 > r33) {
    double s4 = std::sqrt(1.0 + r11 - r22 - r33) * 2;
    qw = (r32 - r23) / s4; qx = 0.25 * s4; qy = (r12 + r21) / s4; qz = (r13 + r31) / s4;
  } else if (r22 > r33) {
    double s4 = std::sqrt(1.0 + r22 - r11 - r33) * 2;
    qw = (r13 - r31) / s4; qx = (r12 + r21) / s4; qy = 0.25 * s4; qz = (r23 + r32) / s4;
  } else {
    double s4 = std::sqrt(1.0 + r33 - r11 - r22) * 2;
    qw = (r21 - r12) / s4; qx = (r13 + r31) / s4; qy = (r23 + r32) / s4; qz = 0.25 * s4;
  }
  s.ee_quat[0] = qw; s.ee_quat[1] = qx; s.ee_quat[2] = qy; s.ee_quat[3] = qz;
  s.gripper_width = width;
  s.robot_mode = static_cast<uint8_t>(rs.robot_mode);
  std::lock_guard<std::mutex> lk(g_state_mtx);
  g_state = s;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-ip>" << std::endl;
    return 1;
  }

  // sockets
  int rx = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(dsfranka::kCmdPort);
  if (bind(rx, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }
  int tx = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in client{};
  client.sin_family = AF_INET;
  client.sin_addr.s_addr = inet_addr("127.0.0.1");  // TODO: learn from first rx packet
  client.sin_port = htons(dsfranka::kStatePort);

  try {
    franka::Robot robot(argv[1]);
    robot.setCollisionBehavior(
        {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
        {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
        {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}},
        {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}});

    franka::Gripper gripper(argv[1]);
    double gripper_width = 0.08;

    std::thread t_rx(udp_rx_loop, rx);
    std::thread t_tx(state_tx_loop, tx, client);
    std::thread t_grip(gripper_loop, &gripper);

    // publish state while waiting for the client to sync + send first command
    std::cout << "waiting for first command near current configuration..." << std::endl;
    uint32_t seq = 0;
    while (g_running) {
      franka::RobotState rs = robot.readOnce();
      publish_state(rs, gripper_width, seq++);
      if (g_have_cmd) {
        std::lock_guard<std::mutex> lk(g_mtx);
        double err = 0;
        for (int i = 0; i < 7; i++) err = std::max(err, std::abs(g_target_q[i] - rs.q[i]));
        if (err < kStartTolerance) break;
        std::cout << "first target too far from current q (err=" << err
                  << " rad), ignoring" << std::endl;
        g_have_cmd = false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "starting 1 kHz joint position control" << std::endl;
    std::array<double, 7> q_cmd{};
    bool first = true;
    robot.control([&](const franka::RobotState& rs,
                      franka::Duration /*period*/) -> franka::JointPositions {
      if (first) {
        q_cmd = rs.q_d;
        first = false;
      }
      std::array<double, 7> tgt;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        tgt = g_target_q;
      }
      for (int i = 0; i < 7; i++) {
        double next = kSmoothing * q_cmd[i] + (1.0 - kSmoothing) * tgt[i];
        double step = next - q_cmd[i];
        if (step > kMaxDqPerTick) step = kMaxDqPerTick;
        if (step < -kMaxDqPerTick) step = -kMaxDqPerTick;
        q_cmd[i] += step;
      }
      publish_state(rs, gripper_width, seq++);
      return franka::JointPositions(q_cmd);
    });
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    g_running = false;
    return 1;
  }
  g_running = false;
  return 0;
}
