#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

#include "RobotConfig.h"

class RobotServer {
private:
    zmq::context_t context{1};
    zmq::socket_t pub_socket;
    zmq::socket_t sub_socket;
    // zmq::socket_t rep_socket;

    std::thread pub_thread;
    std::thread sub_thread;
    // std::thread rep_thread;

    std::atomic<bool> isRunning{true};
    std::atomic<bool> isClientConnected{false};
    RobotConfig config;

    // Publisher thread: publishes "Hello World" at 100 Hz
    void publisherTask() {
        while (isRunning) {
            // std::cout << "Publishing..." << std::endl;
            std::string message = "Hello World";
            zmq::message_t zmq_message(message.size());
            memcpy(zmq_message.data(), message.data(), message.size());

            pub_socket.send(zmq_message, zmq::send_flags::none);
            // std::cout << "Published: " << message << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100 Hz
        }
        std::cout << "Publisher thread stopped." << std::endl;
    }

    // Subscriber thread: receives messages and does nothing
    void subscriberTask() {
        while (isRunning) {
            zmq::message_t message;
            sub_socket.recv(message, zmq::recv_flags::none);
            std::string received_msg(static_cast<char*>(message.data()), message.size());

            std::cout << "Subscriber received: " << received_msg << std::endl;
        }
        std::cout << "Subscriber thread stopped." << std::endl;
    }

    // Responder thread: responds with "Hello World" upon receiving a request
    // void serviceTask() {
    //     std::cout << "Service thread started." << std::endl;
    //     while (isRunning) {
    //         zmq::message_t request;
    //         rep_socket.recv(request, zmq::recv_flags::none);
    //         std::string request_msg(static_cast<char*>(request.data()), request.size());

    //         std::cout << "Responder received request: " << request_msg << std::endl;
    //         if (isClientConnected) {
    //             std::cout << "Client is already connected." << std::endl;
    //             isClientConnected = false;
    //             sub_thread.join();
    //         } else {
    //             std::cout << "Client is not connected." << std::endl;
    //         }
    //         isClientConnected = true;
    //         std::string response = config.getValue("PublisherAddr");
    //         zmq::message_t zmq_response(response.size());
    //         memcpy(zmq_response.data(), response.data(), response.size());
    //         rep_socket.send(zmq_response, zmq::send_flags::none);
    //     }
    //     std::cout << "Responder thread stopped." << std::endl;
    // }

public:
    RobotServer(const std::string& config_path)
        : pub_socket(context, zmq::socket_type::pub),
          sub_socket(context, zmq::socket_type::sub),
        //   rep_socket(context, zmq::socket_type::rep),
          config(config_path) {
        config.display();
        // Bind and connect sockets
        pub_socket.bind(config.getValue("PublisherAddr"));
        // sub_socket.connect(config.getValue("SubscriberAddr"));
        // sub_socket.set(zmq::sockopt::subscribe, "");
        // rep_socket.bind(config.getValue("LocalServiceAddr"));
        sub_thread = std::thread(&RobotServer::subscriberTask, this);
        // Launch threads
        pub_thread = std::thread(&RobotServer::publisherTask, this);
        // 
        // rep_thread = std::thread(&RobotServer::serviceTask, this);
    }

    ~RobotServer() {
        std::cout << "RobotServer stopped and all threads joined." << std::endl;
        // Set isRunning to false to signal threads to stop
        isRunning = false;

        // Join threads to ensure they stop before destruction
        if (pub_thread.joinable()) pub_thread.join();
        if (sub_thread.joinable()) sub_thread.join();
        // if (rep_thread.joinable()) rep_thread.join();

    }

    void stop() {
        isRunning = false;
    }

    void spin() {
  try {
        franka::Robot robot("192.168.1.1");  // Replace with your robot's IP address

        // Set collision behavior to ensure safety during manual manipulation
        robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},  // lower torque thresholds
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},         // external force thresholds
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

        std::ofstream joint_data_file("joint_data.txt");

        // Lambda function to log joint positions in real time
        robot.read([&joint_data_file](const franka::RobotState& state) {
        for (size_t i = 0; i < state.q.size(); i++) {
            joint_data_file << state.q[i] << " ";
        }
        joint_data_file << std::endl;
        return franka::MotionFinished(state);  // End the loop after one run
        });

    } catch (const franka::Exception& ex) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
        std::cout << "RobotServer is running. Press Ctrl+C to stop." << std::endl;
        while (isRunning) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};