#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "node_robot_swarm.cpp"
#include "master.cpp"
#include <random>


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  int nodes = 10;
  
  
  std::vector<std::shared_ptr<Robot>> robot_array;

  std::random_device rd;
  std::mt19937 gen(rd());

// Define the distribution for real numbers between 0 and 1
  std::uniform_real_distribution<double> distribution(0.1, 1.0);

// Generate a random number

  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    double random_number = distribution(gen);
    auto robot = std::make_shared<Robot>(nodename, r_namespace, false, random_number);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array, static_cast<int>(nodes));
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}