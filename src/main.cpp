/**
 * @file node_robot_swarm.cpp
 * @brief Main function for a robot swarm control application using ROS 2 and
 * RobotSwarm.
 * @author Neha Nitin Madhekar,Vinay Krishna Bukka, Rashmi Kapu
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */

#include <stdexcept>
#include <string>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "master.cpp"
#include "node_robot_swarm.cpp"

/**
 * @brief Main function to initialize and run a robot swarm control application.
 *
 * This function initializes ROS 2, creates a multi-threaded executor, generates
 * a specified number of robot nodes with random parameters, and adds them to
 * the executor. It also creates a SwarmMaster node, adds it to the executor,
 * and starts spinning the executor.
 *
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return 0 on successful execution.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  int nodes = 10;

  std::vector<std::shared_ptr<RobotSwarm>> robot_array;

  std::random_device rd;
  std::mt19937 gen(rd());

  // Define the distribution for real numbers between 0 and 1
  std::uniform_real_distribution<double> distribution(0.1, 0.2);

  // Generate a random number

  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    double random_number = distribution(gen);
    auto robot = std::make_shared<RobotSwarm>(nodename, r_namespace, false,
                                              random_number);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node =
      std::make_shared<SwarmMaster>(robot_array, static_cast<int>(nodes));
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
