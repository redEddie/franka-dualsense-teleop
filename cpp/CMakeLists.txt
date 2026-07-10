cmake_minimum_required(VERSION 3.16)
project(dsfranka_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# libfranka: install from source (see cpp/README.md), then
#   cmake -B build -DFranka_DIR=/path/to/libfranka/build
find_package(Franka REQUIRED)
find_package(Threads REQUIRED)

add_executable(01_echo_state examples/01_echo_state.cpp)
target_link_libraries(01_echo_state Franka::Franka)

add_executable(02_move_to_home examples/02_move_to_home.cpp)
target_link_libraries(02_move_to_home Franka::Franka)

add_executable(franka_bridge bridge/franka_bridge.cpp)
target_link_libraries(franka_bridge Franka::Franka Threads::Threads)
