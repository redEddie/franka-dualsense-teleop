// Robot-specific joint limits, fixed at compile time.
// Select with:  cmake -B build -DDSFRANKA_ROBOT=fr3   (default)
//               cmake -B build -DDSFRANKA_ROBOT=panda
// Keep the choice in sync with `robot:` in configs/teleop.yaml.
#pragma once
#include <array>

namespace dsfranka {

#if defined(DSFRANKA_ROBOT_PANDA)

inline constexpr const char* kRobotName = "panda";
// Panda (FER) joint position limits [rad]
inline constexpr std::array<double, 7> kQLower{-2.8973, -1.7628, -2.8973, -3.0718,
                                               -2.8973, -0.0175, -2.8973};
inline constexpr std::array<double, 7> kQUpper{2.8973, 1.7628, 2.8973, -0.0698,
                                               2.8973, 3.7525, 2.8973};
// Panda joint velocity limits [rad/s]
inline constexpr std::array<double, 7> kDqMax{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
// Panda joint acceleration limits [rad/s^2] (datasheet)
inline constexpr std::array<double, 7> kDdqMax{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

#else  // FR3 (default)

inline constexpr const char* kRobotName = "fr3";
// FR3 joint position limits [rad] (matches menagerie franka_fr3/fr3.xml)
inline constexpr std::array<double, 7> kQLower{-2.7437, -1.7837, -2.9007, -3.0421,
                                               -2.8065, 0.5445, -3.0159};
inline constexpr std::array<double, 7> kQUpper{2.7437, 1.7837, 2.9007, -0.1518,
                                               2.8065, 4.5169, 3.0159};
// FR3 joint velocity limits [rad/s] (datasheet; actual limits are position-dependent)
inline constexpr std::array<double, 7> kDqMax{2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
// FR3 joint acceleration limits [rad/s^2] (datasheet)
inline constexpr std::array<double, 7> kDdqMax{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

#endif

// teleop safety: stay this far inside the position limits [rad]
inline constexpr double kQMargin = 0.02;
// teleop velocity cap as a fraction of the robot's joint velocity limit
inline constexpr double kVelFraction = 0.4;
// teleop acceleration cap as a fraction of the robot's joint acceleration limit
// (bounds the per-tick velocity change so the motion generator sees a smooth ramp)
inline constexpr double kAccFraction = 0.3;

}  // namespace dsfranka
