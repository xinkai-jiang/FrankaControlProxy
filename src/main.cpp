#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include "RobotServer.h"

void handleSIGINT(int signal) {
    std::cout << "\nSIGINT received. Stopping..." << std::endl;
    if (signal == SIGINT) {
        std::cout << "\nSIGINT received. Stopping..." << std::endl;
        // isRunning = false; // Set flag to false to stop the main loop
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Please input the config path" << std::endl;
        return -1;
    }
    // Read config file
    RobotServer server = RobotServer(argv[1]);
    // Set up the SIGINT handler using a lambda
    server.spin();

    return 0;
}