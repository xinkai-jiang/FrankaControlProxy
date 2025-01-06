cmake_minimum_required(VERSION 3.10)
project(FRANKA_ZMQ)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find CZMQ package
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Include directories
include_directories(include)

# Add executable target
add_executable(robot_config src/main.cpp)

# Link ZeroMQ to your target
target_link_libraries(robot_config PRIVATE ${ZMQ_LIBRARIES})
