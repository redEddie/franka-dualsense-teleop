// libfranka practice 2: smooth joint point-to-point motion to the home pose.
// Cosine-interpolated JointPositions callback — the minimal "make it move" example.
// Usage: ./02_move_to_home <robot-ip> [preset]   preset = libero (default) | dsfranka
// SAFETY: robot moves! Clear the workspace, keep the user stop in hand.
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: " << argv[0] << " <robot-ip> [libero|dsfranka]" << std::endl;
    return 1;
  }
  // Home presets must match configs/teleop.yaml home.presets so the arm lands
  // exactly where teleop_real seeds its target (q7 = +M_PI_4 in both).
  const std::array<double, 7> q_dsfranka = {{0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4}};
  const std::array<double, 7> q_libero =
      {{0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, M_PI_4}};
  const char* preset = (argc == 3) ? argv[2] : "libero";  // default matches config
  std::array<double, 7> q_goal;
  if (std::strcmp(preset, "dsfranka") == 0) {
    q_goal = q_dsfranka;
  } else if (std::strcmp(preset, "libero") == 0) {
    q_goal = q_libero;
  } else {
    std::cerr << "unknown preset '" << preset << "' (use libero|dsfranka)" << std::endl;
    return 1;
  }
  std::cout << "moving to '" << preset << "' home..." << std::endl;
  const double duration = 5.0;  // seconds

  try {
    franka::Robot robot(argv[1]);
    robot.setCollisionBehavior(
        {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
        {{20, 20, 18, 18, 16, 14, 12}}, {{20, 20, 18, 18, 16, 14, 12}},
        {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}},
        {{20, 20, 20, 25, 25, 25}}, {{20, 20, 20, 25, 25, 25}});

    std::array<double, 7> q_start{};
    double t = 0.0;
    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      if (t == 0.0) q_start = rs.q_d;
      t += period.toSec();
      double s = 0.5 * (1.0 - std::cos(M_PI * std::min(t / duration, 1.0)));
      std::array<double, 7> q{};
      for (int i = 0; i < 7; i++) q[i] = q_start[i] + s * (q_goal[i] - q_start[i]);
      franka::JointPositions out(q);
      if (t >= duration) return franka::MotionFinished(out);
      return out;
    });
    std::cout << "done." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
