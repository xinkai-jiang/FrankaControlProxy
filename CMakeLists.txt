cmake_minimum_required(VERSION 3.10)
project(FRANKA_CONTROL_PROXY)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find_package(Franka REQUIRED)

# Find libzmq package
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIR})

# Add executable target
add_executable(robot src/main.cpp)

# Link ZeroMQ to your target
target_link_libraries(robot PRIVATE ${ZMQ_LIBRARIES} ${YAML_CPP_LIBRARIES})
