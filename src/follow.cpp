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
    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);
    // robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
    //                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 300.0, 100.0, 150.0, 150.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 25.0, 25.0, 15.0}};

    // const std::array<double, 7> k_gains = {{6.0, 6.0, 6.0, 6.0, 2.50, 1.50, 1.0}};
    // // Damping
    // const std::array<double, 7> d_gains = {{5.0, 5.0, 5.0, 5.0, 3.0, 2.50, 1.50}};
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    // robot.setJointImpedance({{30, 30, 30, 25, 25, 20, 20}});
    robot.setJointImpedance({{0, 0, 0, 0, 0, 0, 0}});
    franka::Model model = robot.loadModel();
    // struct
    // {
    //     std::mutex mutex;
    //     std::array<double, 7> target_joint_pose;
    //     std::array<double, 2> gripper;
    // } _state{};
    struct
    {
        std::mutex mutex;
        std::array<double, 7> target_joint_pose;
        franka::RobotState robot_state;
    } _state{};
    franka::RobotState initial_state = robot.readOnce();
    _state.target_joint_pose = initial_state.q;
    _state.robot_state = initial_state;
    std::atomic_bool running{true};

    // ZMQ setting 
    zmq::context_t context{1};
    // zmq::socket_t pub_socket{context, zmq::socket_type::pub};
    // pub_socket.bind(publisher_addr);
    // zmq::socket_t srv_socket(context, zmq::socket_type::rep);
    // srv_socket.Connect(publisher_addr);
    zmq::socket_t sub_socket(context, zmq::socket_type::sub);
    sub_socket.connect(publisher_addr);
    sub_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);


    // std::thread state_publish_thread(
    //     [&pub_socket, &robot, &gripper, &running]() {
    //         while (running)
    //         {
    //             // ...existing code...
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //             std::vector<float> msg_data;
    //             franka::RobotState robot_state = robot.readOnce();
    //             franka::GripperState gripper_state = gripper.readOnce();
    //             msg_data.insert(msg_data.end(), robot_state.q.begin(), robot_state.q.end());
    //             msg_data.insert(msg_data.end(), robot_state.dq.begin(), robot_state.dq.end());
    //             msg_data.push_back(static_cast<float>(gripper_state.width));
    //             msg_data.push_back(static_cast<float>(gripper_state.max_width));
    //             if (!msg_data.empty())
    //             {
    //                 pub_socket.send(zmq::buffer(msg_data.data(), msg_data.size() * sizeof(float)), 
    //                                 zmq::send_flags::none);
    //             }
    //         }
    //     });

    std::thread sub_thread(
        [&sub_socket, &running, &_state]() {
            while (running)
            {
                std::cout << "start sub" << std::endl;
                zmq::message_t message;
                sub_socket.recv(message, zmq::recv_flags::none);
                std::cout << "received data" << std::endl;
                if (message.size() != sizeof(std::array<double, 7>))
                {
                    std::cerr << "Received message size mismatch: expected " << sizeof(std::array<double, 7>)
                              << ", got " << message.size() << std::endl;
                    continue;
                }
                std::array<double, 7> target_joint_pose;
                memcpy(target_joint_pose.data(), message.data(), message.size());
                for (const auto& pos : target_joint_pose) {
                    std::cout << pos << " ";
                }
                
                std::cout << std::endl;
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

    // human controller
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        joint_torque_callback = [](const franka::RobotState &state, franka::Duration) -> franka::Torques
    {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        return zero_torques;
    };

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        impedance_control_callback =
            [&_state, &model, &k_gains, &d_gains](
                const franka::RobotState &state, franka::Duration) -> franka::Torques
    {
        if (_state.mutex.try_lock())
        {
            _state.robot_state = state;
            // double sum = 0;
            // for (size_t i = 0; i < state.q.size(); ++i)
            // {
            //     // std::cout << _state.target_joint_pose[i] << "   " << state.q[i] << std::endl;
            //     sum += std::abs(_state.target_joint_pose[i] - state.q[i]) / k_gains[i];
            //     // std::cout << "sum: " << std::abs(_state.target_joint_pose[i] - state.q[i]) << "   " << sum << std::endl;
            // }
            // std::cout << sum << std::endl;
            _state.mutex.unlock();
        }

        std::array<double, 7> coriolis = model.coriolis(state);
        std::array<double, 7> tau_d_calculated;
        if (_state.mutex.try_lock())
        {
            for (size_t i = 0; i < 7; i++)
            {
                // tau_d_calculated[i] =
                //     k_gains[i] * (_state.target_joint_pose[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
                tau_d_calculated[i] = k_gains[i] * (_state.target_joint_pose[i] - state.q[i]) - d_gains[i] * state.dq[i];
            }
            _state.mutex.unlock();
        }
        // if (in_controlled)
        // {
        //     if (_state.mutex.try_lock())
        //     {
        //         for (size_t i = 0; i < 7; i++)
        //         {
        //             tau_d_calculated[i] =
        //                 k_gains[i] * (_state.target_joint_pose[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
        //         }
        //         _state.mutex.unlock();
        //     }
        // }
        // else
        // {
        //     for (size_t i = 0; i < 7; i++)
        //     {
        //         tau_d_calculated[i] = coriolis[i];
        //     }
        // }
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
        // Send torque command.
        return tau_d_rate_limited;
    };




    try
    {
        // robot.control(joint_torque_callback);
        robot.control(impedance_control_callback);
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