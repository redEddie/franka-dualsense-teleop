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
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include "robot_limits.hpp"
#include "udp_protocol.hpp"

using dsfranka::CmdPacket;
using dsfranka::StatePacket;

namespace {

constexpr double kSmoothing = 0.995;      // per-ms exponential filter on targets
constexpr double kStartTolerance = 0.05;  // rad: first target must be near current q

// gripper (Franka Hand) — blocking commands run async so the loop can preempt
constexpr double kGripMaxWidth = 0.08;    // m, fully open
constexpr double kGripSpeed = 0.1;        // m/s (Franka Hand max)
constexpr double kGripDeadband = 0.02;    // min target change [0..1] to reissue a command
constexpr double kGraspThresh = 0.06;     // target below this -> grasp (apply force) vs move
constexpr double kGraspForce = 40.0;      // N holding force when grasping

// per-joint hard clamps derived from the compile-time robot selection
double max_dq_per_tick(int i) { return dsfranka::kVelFraction * dsfranka::kDqMax[i] * 1e-3; }
// max change in per-tick position delta = accel * dt^2 (dt = 1 ms)
double max_ddq_per_tick(int i) { return dsfranka::kAccFraction * dsfranka::kDdqMax[i] * 1e-6; }
double clamp_q(int i, double q) {
  const double lo = dsfranka::kQLower[i] + dsfranka::kQMargin;
  const double hi = dsfranka::kQUpper[i] - dsfranka::kQMargin;
  return q < lo ? lo : (q > hi ? hi : q);
}

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
  // Franka Hand grasp()/move() are blocking (~up to 0.8 s per stroke). Running
  // them synchronously made the gripper lag — intermediate trigger values were
  // dropped while a stale command finished. Instead run each command async and
  // preempt it with stop() as soon as a newer target arrives, so the hand always
  // chases the latest R2 position.
  double last_issued = -1.0;
  std::future<void> motion;
  auto motion_done = [&]() {
    return !motion.valid() ||
           motion.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  };
  while (g_running) {
    double target;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      target = g_target_gripper;
    }
    if (g_have_cmd && std::abs(target - last_issued) > kGripDeadband) {
      if (!motion_done()) {
        gripper->stop();   // abort the in-flight grasp/move so we can reissue
        motion.wait();
      }
      last_issued = target;
      motion = std::async(std::launch::async, [gripper, target]() {
        try {
          if (target < kGraspThresh) {
            gripper->grasp(0.0, kGripSpeed, kGraspForce, 0.08, 0.08);  // close & hold
          } else {
            gripper->move(kGripMaxWidth * target, kGripSpeed);
          }
        } catch (const franka::Exception&) {
          // benign: grasp closed on empty air, or the motion was preempted by stop()
        }
      });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (motion.valid()) motion.wait();
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
  std::cout << "franka_bridge — robot type: " << dsfranka::kRobotName
            << " (compile-time, see cpp/bridge/robot_limits.hpp)" << std::endl;

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
  // bounded recv so udp_rx_loop can observe g_running and be joined on shutdown
  struct timeval rx_timeout{0, 100000};  // 100 ms
  setsockopt(rx, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));
  int tx = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in client{};
  client.sin_family = AF_INET;
  client.sin_addr.s_addr = inet_addr("127.0.0.1");  // TODO: learn from first rx packet
  client.sin_port = htons(dsfranka::kStatePort);

  try {
    franka::Robot robot(argv[1]);
    // Collision thresholds. The acceleration-phase thresholds are set higher than
    // the nominal ones because the external-force/torque estimate is noisier while
    // joints accelerate — keeping them equal (and low) made fast teleop moves trip
    // a spurious cartesian_reflex right at start. Raise these further for hard
    // contact tasks, lower them for tighter safety. [torque Nm, force N/Nm]
    robot.setCollisionBehavior(
        {{40, 40, 38, 38, 34, 32, 28}}, {{40, 40, 38, 38, 34, 32, 28}},  // torque, acceleration
        {{30, 30, 28, 28, 26, 24, 22}}, {{30, 30, 28, 28, 26, 24, 22}},  // torque, nominal
        {{50, 50, 50, 50, 50, 50}}, {{50, 50, 50, 50, 50, 50}},          // force, acceleration
        {{35, 35, 35, 40, 40, 40}}, {{35, 35, 35, 40, 40, 40}});         // force, nominal

    franka::Gripper gripper(argv[1]);
    double gripper_width = 0.08;

    std::thread t_rx(udp_rx_loop, rx);
    std::thread t_tx(state_tx_loop, tx, client);
    std::thread t_grip(gripper_loop, &gripper);

    // Join the workers on ANY exit from this scope (normal return or a control
    // exception unwinding) BEFORE the gripper/sockets they reference die and before
    // the std::thread destructors run — a joinable thread destructor calls
    // std::terminate and would mask the real franka error. Declared last so it is
    // destroyed first; the outer catch then reports e.what().
    struct Joiner {
      std::thread &a, &b, &c;
      ~Joiner() {
        g_running = false;
        if (a.joinable()) a.join();
        if (b.joinable()) b.join();
        if (c.joinable()) c.join();
      }
    } joiner{t_rx, t_tx, t_grip};

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
    std::array<double, 7> dq_cmd{};  // per-tick position delta (velocity·dt), for accel limiting
    bool first = true;
    robot.control([&](const franka::RobotState& rs,
                      franka::Duration /*period*/) -> franka::JointPositions {
      if (first) {
        // start exactly at the current commanded configuration with zero velocity so
        // the motion generator sees a continuous trajectory from rest (otherwise the
        // first non-zero step is a velocity/acceleration discontinuity -> reflex)
        q_cmd = rs.q_d;
        dq_cmd.fill(0.0);
        first = false;
        publish_state(rs, gripper_width, seq++);
        return franka::JointPositions(q_cmd);
      }
      std::array<double, 7> tgt;
      {
        std::lock_guard<std::mutex> lk(g_mtx);
        tgt = g_target_q;
      }
      for (int i = 0; i < 7; i++) {
        double next = kSmoothing * q_cmd[i] + (1.0 - kSmoothing) * clamp_q(i, tgt[i]);
        double step = next - q_cmd[i];
        // velocity clamp (position delta per tick)
        const double dq_max = max_dq_per_tick(i);
        if (step > dq_max) step = dq_max;
        if (step < -dq_max) step = -dq_max;
        // acceleration clamp: bound the change in per-tick delta so velocity ramps
        // instead of stepping — prevents the velocity/acceleration discontinuity reflex
        const double ddq_max = max_ddq_per_tick(i);
        double dstep = step - dq_cmd[i];
        if (dstep > ddq_max) dstep = ddq_max;
        if (dstep < -ddq_max) dstep = -ddq_max;
        step = dq_cmd[i] + dstep;
        dq_cmd[i] = step;
        q_cmd[i] = clamp_q(i, q_cmd[i] + step);
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
