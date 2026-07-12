// libfranka practice 3: characterize the Franka Hand to settle whether real-time
// continuous width control is feasible, and measure the actual max speed.
//
// Usage: ./03_gripper_probe <robot-ip>
// SAFETY: the fingers open/close repeatedly — keep them clear. move() only
// positions (no grasp force), so it stalls on contact rather than crushing.
//
// The bridge holds the gripper connection while it runs, so STOP the bridge
// first (only one gripper client at a time).
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/gripper.h>

using clk = std::chrono::steady_clock;
static double secs(clk::time_point a, clk::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <robot-ip>" << std::endl;
    return 1;
  }
  try {
    franka::Gripper gripper(argv[1]);
    franka::GripperState st = gripper.readOnce();
    std::cout << "max_width=" << st.max_width << " m  width=" << st.width
              << " m  temp=" << st.temperature << " C  grasped=" << st.is_grasped
              << std::endl;
    const double W = st.max_width;

    // --- A. effective width-change speed vs the commanded speed ---------------
    // Commanding more than the hardware max should NOT go faster; this shows the
    // real ceiling.
    std::cout << "\n[A] effective open->closed speed vs commanded speed:" << std::endl;
    for (double v : std::vector<double>{0.05, 0.1, 0.15, 0.2}) {
      gripper.move(W, 0.1);                         // ensure fully open
      double w0 = gripper.readOnce().width;
      auto t0 = clk::now();
      gripper.move(0.005, v);                       // close (no grasp force)
      double dt = secs(t0, clk::now());
      double w1 = gripper.readOnce().width;
      std::cout << "  cmd " << v << " m/s -> moved " << (w0 - w1) * 1000 << " mm in "
                << dt << " s = effective " << (w0 - w1) / dt << " m/s" << std::endl;
    }

    // --- B. pure move() round-trip latency (target == current, ~no travel) ----
    // This is the protocol floor: even a zero-distance command costs this much,
    // so it caps how fast width targets can be streamed.
    std::cout << "\n[B] move() round-trip latency (target already reached):" << std::endl;
    gripper.move(0.04, 0.1);
    const int N = 10;
    auto t0 = clk::now();
    for (int i = 0; i < N; i++) gripper.move(0.04, 0.1);
    double per = secs(t0, clk::now()) / N;
    std::cout << "  " << per * 1000 << " ms per call -> max update rate ~" << (1.0 / per)
              << " Hz" << std::endl;

    // --- C. small 5 mm step responsiveness -----------------------------------
    std::cout << "\n[C] alternating +/-5 mm steps:" << std::endl;
    auto t1 = clk::now();
    for (int i = 0; i < N; i++) gripper.move(0.04 + (i % 2 ? 0.005 : -0.005), 0.1);
    std::cout << "  " << secs(t1, clk::now()) / N * 1000 << " ms per 5 mm step" << std::endl;

    gripper.move(W, 0.1);  // leave it open
    std::cout << "\ndone." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "gripper error: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
