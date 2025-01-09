// #include <iostream>
// #include <cmath>
// #include <functional>
// #include <thread>
// #include <chrono>
// #include <atomic>

// #include <zmq.hpp>
// #include <franka/duration.h>
// #include <franka/exception.h>
// #include <franka/model.h>
// #include <franka/rate_limiting.h>
// #include <franka/robot.h>

// #include "RobotConfig.h"



// class RobotServer
// {
// private:
// 	// franka::Robot robot;

// 	zmq::context_t context{1};
// 	zmq::socket_t pub_socket;
// 	zmq::socket_t sub_socket;
// 	// zmq::socket_t rep_socket;

// 	std::thread pub_thread;
// 	std::thread sub_thread;
// 	// std::thread rep_thread;

// 	std::atomic<bool> isRunning{true};
// 	std::atomic<bool> isClientConnected{false};
// 	RobotConfig config;

// 	// Publisher thread: publishes "Hello World" at 100 Hz
// 	void publisherTask(franka::Robot &robot)
// 	{
// 		std::vector<std::string> keys_to_send = {"name", "scores"};
// 		while (isRunning)
// 		{
// 			franka::RobotState state = robot.readOnce();
// 			// Serialize only the selected keys
// 			std::ostringstream oss;
// 			oss << "{";
// 			addKeyValue(oss, "q", state.q);
// 			addKeyValue(oss, "dq", state.dq);
// 			oss << "}";
// 			std::string message = oss.str();
// 			zmq::message_t zmq_message(message.size());
// 			memcpy(zmq_message.data(), message.data(), message.size());

// 			pub_socket.send(zmq_message, zmq::send_flags::none);
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
// 		}
// 		std::cout << "Publisher thread stopped." << std::endl;
// 	}

// 	// Subscriber thread: receives messages and does nothing
// 	void subscriberTask()
// 	{
// 		while (isRunning)
// 		{
// 			zmq::message_t message;
// 			sub_socket.recv(message, zmq::recv_flags::none);
// 			std::string received_msg(static_cast<char *>(message.data()), message.size());

// 			std::cout << "Subscriber received: " << received_msg << std::endl;
// 		}
// 		std::cout << "Subscriber thread stopped." << std::endl;
// 	}

// public:
// 	RobotServer(const std::string &config_path)
// 		: pub_socket(context, zmq::socket_type::pub),
// 		  sub_socket(context, zmq::socket_type::sub),
// 		  //   rep_socket(context, zmq::socket_type::rep),
// 		  config(config_path)
// 	{
// 		config.display();
// 		// Bind and connect sockets
// 		pub_socket.bind(config.getValue("PublisherAddr"));
// 		// sub_socket.connect(config.getValue("SubscriberAddr"));
// 		// sub_socket.set(zmq::sockopt::subscribe, "");
// 		// rep_socket.bind(config.getValue("LocalServiceAddr"));
// 		// sub_thread = std::thread(&RobotServer::subscriberTask, this);
// 		// Launch threads
		
// 		//
// 		// rep_thread = std::thread(&RobotServer::serviceTask, this);
// 	}

// 	~RobotServer()
// 	{
// 		std::cout << "RobotServer stopped and all threads joined." << std::endl;
// 		// Set isRunning to false to signal threads to stop


// 		// Join threads to ensure they stop before destruction
// 		// if (rep_thread.joinable()) rep_thread.join();
// 		stop();
// 	}

// 	void stop()
// 	{
// 		isRunning = false;
// 		if (pub_thread.joinable())
// 			pub_thread.join();
// 		if (sub_thread.joinable())
// 			sub_thread.join();
// 	}

// 	void spin()
// 	{
// 		try
// 		{
// 			franka::Robot robot(config.getValue("RobotIP")); // Replace with your robot's IP address
// 			// First move the robot to a suitable joint configuration
// 			std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
// 			// MotionGenerator motion_generator(0.5, q_goal);
// 			std::cout << "WARNING: This example will move the robot! "
// 					  << "Please make sure to have the user stop button at hand!" << std::endl
// 					  << "Press Enter to continue..." << std::endl;
// 			// Set collision behavior to ensure safety during manual manipulation
// 			robot.setCollisionBehavior(
// 				{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, // lower torque thresholds
// 				{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
// 				{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, // external force thresholds
// 				{{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
// 			robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
// 			robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
// 			franka::Model model = robot.loadModel();

// 			// Set gains for the joint impedance control.
// 			// TODO: Real it from the config file
// 			// Stiffness
// 			const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
// 			// Damping
// 			const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
// 			// Define callback for the joint torque control loop.
// 			std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
// 				impedance_control_callback =
// 					[&model, k_gains, d_gains](
// 						const franka::RobotState &state, franka::Duration /*period*/) -> franka::Torques
// 			{
// 				// Read current coriolis terms from model.
// 				std::array<double, 7> coriolis = model.coriolis(state);

// 				// Compute torque command from joint impedance control law.
// 				// Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
// 				// time step delay.
// 				std::array<double, 7> tau_d_calculated;
// 				for (size_t i = 0; i < 7; i++)
// 				{
// 					tau_d_calculated[i] =
// 						// k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i];
// 						coriolis[i];
// 				}

// 				// The following line is only necessary for printing the rate limited torque. As we activated
// 				// rate limiting for the control loop (activated by default), the torque would anyway be
// 				// adjusted!
// 				std::array<double, 7> tau_d_rate_limited =
// 					franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

// 				// Send torque command.
// 				return tau_d_rate_limited;
// 			};
			
// 			pub_thread = std::thread(&RobotServer::publisherTask, this, std::ref(robot));


// 			std::cout << "Type 'q' to quit: " << std::endl;
// 			while (true)
// 			{
// 				std::string input;
// 				std::cin >> input;
// 				if (input == "q")
// 				{
// 					std::cout << "Exiting..." << std::endl;
// 					break;
// 				}
// 			}
// 		}
// 		catch (const franka::Exception &e)
// 		{
// 			std::cerr << e.what() << std::endl;
// 			std::cerr << "Exception" << std::endl;
// 		}
// 		stop();
// 	}
// };