#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <mutex>
#include <thread>
#include <atomic>
#include <zmq.hpp>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include "RobotConfig.h"

bool isNumber(const std::string &str)
{
    // Basic check if the string represents a number (integer or float)
    std::istringstream iss(str);
    double temp;
    return (iss >> temp) && (iss.eof() || iss.get() == '\n');
}

template <size_t N>
bool parseJsonArrayToStdArray(const std::string &json_str, std::array<double, N> &arr)
{
    // Find the positions of the opening and closing brackets
    size_t start = json_str.find('[');
    size_t end = json_str.find(']');

    if (start == std::string::npos || end == std::string::npos || end <= start)
    {
        std::cerr << "Invalid JSON format: Missing brackets." << std::endl;
        return false;
    }

    // Extract content between brackets
    std::string content = json_str.substr(start + 1, end - start - 1);

    // Split content into numbers
    std::string num_str;
    bool in_number = false;
    size_t index = 0;

    for (size_t i = 0; i <= content.size(); ++i)
    {
        char ch = (i < content.size()) ? content[i] : ','; // Include comma at the end to trigger parsing

        if (isdigit(ch) || ch == '.' || ch == '-' || ch == '+')
        {
            // Collect characters that form a number
            num_str += ch;
            in_number = true;
        }
        else if (in_number)
        {
            // Number complete, convert to double and store
            if (isNumber(num_str))
            {
                if (index < N)
                {
                    arr[index++] = std::stod(num_str);
                }
            }
            num_str.clear();
            in_number = false;
        }
    }

    return index > 0; // Return true if at least one number was parsed
}

template <typename T, std::size_t N>
std::string arrayToJsonArray(const std::array<T, N> &vec)
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        oss << vec[i];
        if (i < vec.size() - 1)
        {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

template <typename T, std::size_t N>
void addKeyValue(std::ostringstream &oss, const std::string &key, const std::array<T, N> &value, const bool isEnd = false)
{
    if (isEnd)
    {
        oss << "\"" << key << "\":" << arrayToJsonArray(value);
    }
    else
    {
        oss << "\"" << key << "\":" << arrayToJsonArray(value) << ",";
    }
}

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
    // Read config file
    RobotConfig config = RobotConfig(argv[1]);
    franka::Robot robot(config.getValue("RobotIP"));
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 300.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    franka::Model model = robot.loadModel();
    struct
    {
        std::mutex mutex;
        bool in_controlled;
        std::array<double, 7> target_joint_pose;
        franka::RobotState robot_state;
    } _state{};
    _state.in_controlled = false;

    std::atomic_bool running{true};
    zmq::context_t context{1};
    zmq::socket_t pub_socket{context, zmq::socket_type::pub};
    pub_socket.bind(config.getValue("PublisherAddr"));
    zmq::socket_t res_socket(context, zmq::socket_type::rep);
    res_socket.bind(config.getValue("ServiceAddr"));

    // Start print thread.
    std::thread publish_thread([&pub_socket, &_state, &running, &config]()
                               {
        franka::Gripper gripper(config.getValue("RobotIP"));
        while (running) {
        // Sleep to achieve the desired print rate.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::string msg("");
        franka::GripperState gripper_state = gripper.readOnce();
        std::array<double, 2> gripper_width = {gripper_state.width, gripper_state.max_width};
        if (_state.mutex.try_lock()) {
                std::ostringstream oss;
                oss << "{";
                addKeyValue(oss, "q", _state.robot_state.q);
                addKeyValue(oss, "dq", _state.robot_state.dq);
                addKeyValue(oss, "gripper_width", gripper_width, true);
                oss << "}";
                msg = oss.str();
            _state.mutex.unlock();
        }
        if (!msg.empty())
        {
            pub_socket.send(zmq::buffer(msg), zmq::send_flags::none);
            // std::cout << msg << std::endl;
        }
        } });

    std::thread service_thread([&res_socket, &_state, &running]()
                               {
        while (running) {
        zmq::message_t request;
        auto result = res_socket.recv(request, zmq::recv_flags::none);
        res_socket.send(zmq::buffer("Success"), zmq::send_flags::none);
        std::array<double, 7> target_joint_pose;
        std::cout << "receive request" << request.to_string() << std::endl;
        parseJsonArrayToStdArray<7>(request.to_string(), target_joint_pose);
        if (_state.mutex.try_lock()) {
            _state.in_controlled = true;
            _state.target_joint_pose = target_joint_pose;
            _state.mutex.unlock();
        }
        } });

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{60.0, 60.0, 60.0, 60.0, 25.0, 15.0, 10.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        impedance_control_callback =
            [&_state, &model, &k_gains, &d_gains](
                const franka::RobotState &state, franka::Duration) -> franka::Torques
    {
        bool in_controlled = false;
        if (_state.mutex.try_lock())
        {
            _state.robot_state = state;
            if (_state.in_controlled)
            {
                double sum = 0;
                for (size_t i = 0; i < state.q.size(); ++i)
                {
                    // std::cout << _state.target_joint_pose[i] << "   " << state.q[i] << std::endl;
                    sum += std::abs(_state.target_joint_pose[i] - state.q[i]) / k_gains[i];
                    // std::cout << "sum: " << std::abs(_state.target_joint_pose[i] - state.q[i]) << "   " << sum << std::endl;
                }
                std::cout << sum << std::endl;
                if (sum < 0.015)
                {
                    _state.in_controlled = false;
                    std::cout << "Reach to target" << std::endl;
                }
            }
            in_controlled = _state.in_controlled;
            _state.mutex.unlock();
        }

        std::array<double, 7> coriolis = model.coriolis(state);
        std::array<double, 7> tau_d_calculated;
        if (in_controlled)
        {
            if (_state.mutex.try_lock())
            {
                for (size_t i = 0; i < 7; i++)
                {
                    tau_d_calculated[i] =
                        k_gains[i] * (_state.target_joint_pose[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
                }
                _state.mutex.unlock();
            }
        }
        else
        {
            for (size_t i = 0; i < 7; i++)
            {
                tau_d_calculated[i] = coriolis[i];
            }
        }
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
        // Send torque command.
        return tau_d_rate_limited;
    };

    robot.control(impedance_control_callback);
    while (true)
    {
        std::string input;
        std::cin >> input;
        if (input == "q")
        {
            std::cout << "Exiting..." << std::endl;
            running = false;
            if (publish_thread.joinable())
            {
                publish_thread.join();
                std::cout << "publish thread is quit safetly" << std::endl;
            }
            break;
        }
    }

    return 0;
}