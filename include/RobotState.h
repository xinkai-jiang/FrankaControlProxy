#include <iostream>
#include <array>
#include <string>

class RobotState {
public:
    // Member variables
    std::array<double, 7> joint_pos;       // Joint positions (7 DOF)
    std::array<double, 7> joint_vel;       // Joint velocities (7 DOF)
    std::array<double, 7> joint_acc;       // Joint accelerations (7 DOF)
    std::array<double, 16> cart_pos;       // Cartesian position (4x4 homogeneous transformation matrix)
    double gripper_width;                  // Gripper width
    std::string status;                    // Robot status (e.g., "OK", "Error")

    // Constructor to initialize with default values
    RobotState()
        : joint_pos{0.0}, joint_vel{0.0}, joint_acc{0.0}, cart_pos{0.0}, gripper_width(0.0), status("Unknown") {}

    // Function to display the robot state
    void display() const {
        std::cout << "Joint Positions: ";
        for (const auto& pos : joint_pos) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        std::cout << "Joint Velocities: ";
        for (const auto& vel : joint_vel) {
            std::cout << vel << " ";
        }
        std::cout << std::endl;

        std::cout << "Joint Accelerations: ";
        for (const auto& acc : joint_acc) {
            std::cout << acc << " ";
        }
        std::cout << std::endl;

        std::cout << "Cartesian Position (Homogeneous Matrix):" << std::endl;
        for (size_t i = 0; i < 4; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                std::cout << cart_pos[i * 4 + j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Gripper Width: " << gripper_width << std::endl;
        std::cout << "Status: " << status << std::endl;
    }
};
