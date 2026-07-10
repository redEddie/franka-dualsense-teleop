// libfranka practice 2: smooth joint point-to-point motion to the home pose.
// Cosine-interpolated JointPositions callback — the minimal "make it move" example.
// Usage: ./02_move_to_home <robot-ip>
// SAFETY: robot moves! Clear the workspace, keep the user stop in hand.
#include <array>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-ip>" << std::endl;
    return 1;
  }
  const std::array<double, 7> q_goal = {{0, 0, 0, -M_PI_2, 0, M_PI_2, -M_PI_4}};
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
