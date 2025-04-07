cmake_minimum_required(VERSION 3.10)
project(FRANKA_CONTROL_PROXY)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Franka REQUIRED)

# Find libzmq package
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include ${YAML_CPP_INCLUDE_DIR} ${Franka_INCLUDE_DIRS})

# Add executable target
add_executable(leader src/human_controller.cpp)
add_executable(follower src/follow.cpp)

# Link libraries to your target (franka, yaml-cpp, zmq)
target_link_libraries(leader PRIVATE ${YAML_CPP_LIBRARIES} ${Franka_LIBRARIES} ${ZMQ_LIBRARIES})
target_link_libraries(follower PRIVATE ${YAML_CPP_LIBRARIES} ${Franka_LIBRARIES} ${ZMQ_LIBRARIES})
