// libfranka practice 1: connect and read robot state (no motion).
// Usage: ./01_echo_state <robot-ip>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-ip>" << std::endl;
    return 1;
  }
  try {
    franka::Robot robot(argv[1]);
    for (int i = 0; i < 20; i++) {  // ~2 s at readOnce pace
      franka::RobotState s = robot.readOnce();
      std::cout << "q: ";
      for (double v : s.q) std::cout << v << " ";
      std::cout << "\nEE pos: " << s.O_T_EE[12] << " " << s.O_T_EE[13] << " "
                << s.O_T_EE[14] << "\nmode: " << static_cast<int>(s.robot_mode)
                << "\n---" << std::endl;
    }
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
