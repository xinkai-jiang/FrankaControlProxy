#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <zmq.hpp>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

void handleSIGINT(int signal)
{
    std::cout << "\nSIGINT received. Stopping..." << std::endl;
    if (signal == SIGINT)
    {
        std::cout << "\nSIGINT received. Stopping..." << std::endl;
        // isRunning = false; // Set flag to false to stop the main loop
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Please input the config path" << std::endl;
        return 0;
    }
    YAML::Node config = YAML::LoadFile(argv[1]);
    std::string robot_ip = config["RobotIP"].as<std::string>();
    std::string publisher_addr = config["PublisherAddr"].as<std::string>();
    std::string service_addr = config["ServiceAddr"].as<std::string>();
    std::cout << "Robot IP: " << robot_ip << std::endl;
    std::cout << "Publisher Address: " << publisher_addr << std::endl;
    std::cout << "Service Address: " << service_addr << std::endl;

    // Read config file
    franka::Robot robot(config.getValue("RobotIP"));
    franka::Gripper gripper(config.getValue("RobotIP"));
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    franka::Model model = robot.loadModel();
    struct
    {
        std::mutex mutex;
        std::array<double, 9> target_joint_pose;
    } _state{};
    _state.target_joint_pose = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::atomic_bool running{true};

    // ZMQ setting 
    zmq::context_t context{1};
    zmq::socket_t pub_socket{context, zmq::socket_type::pub};
    pub_socket.bind(publisher_addr);
    // zmq::socket_t srv_socket(context, zmq::socket_type::rep);
    // srv_socket.bind(service_addr);
    zmq::socket_t sub_socket(context, zmq::socket_type::sub);


    std::thread state_publish_thread(
        [&pub_socket, &robot, &gripper, &running]() {
            while (running)
            {
                // ...existing code...
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                std::vector<float> msg_data;
                franka::RobotState robot_state = robot.readOnce();
                franka::GripperState gripper_state = gripper.readOnce();
                msg_data.insert(msg_data.end(), robot_state.q.begin(), robot_state.q.end());
                msg_data.insert(msg_data.end(), robot_state.dq.begin(), robot_state.dq.end());
                msg_data.push_back(static_cast<float>(gripper_state.width));
                msg_data.push_back(static_cast<float>(gripper_state.max_width));
                if (!msg_data.empty())
                {
                    pub_socket.send(zmq::buffer(msg_data.data(), msg_data.size() * sizeof(float)), 
                                    zmq::send_flags::none);
                }
            }
        });

    std::thread sub_thread(
        [&sub_socket, &running, &_state]() {
            while (running)
            {
                zmq::message_t message;
                sub_socket.recv(message, zmq::recv_flags::none);
                if (message.size() != sizeof(std::array<double, 9>))
                {
                    std::cerr << "Received message size mismatch: expected " << sizeof(std::array<double, 9>)
                              << ", got " << message.size() << std::endl;
                    continue;
                }
                std::array<double, 9> target_joint_pose;
                memcpy(target_joint_pose.data(), message.data(), message.size());
                if (_state.mutex.try_lock())
                {
                    _state.target_joint_pose = target_joint_pose;
                    _state.mutex.unlock();
                }
            }
        });


    std::function<franka::JointPositions(const franka::RobotState& robot_state, franka::Duration period)> joint_position_callback =
        [&_state](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
            std::array<double, 7> joint_positions;
            if (_state.mutex.try_lock())
            {
                joint_positions = _state.target_joint_pose;
                _state.mutex.unlock();
            }
            franka::JointPositions output = franka::JointPositions(joint_positions);
            return output;
        };

    try
    {
        robot.control(joint_position_callback);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    // while (true)
    // {
    //     std::string input;
    //     std::cin >> input;
    //     if (input == "q")
    //     {
    //         std::cout << "Exiting..." << std::endl;
    //         running = false;
    //         if (publish_thread.joinable())
    //         {
    //             publish_thread.join();
    //             std::cout << "publish thread is quit safely" << std::endl;
    //         }
    //         break;
    //     }
    // }

    return 0;
}