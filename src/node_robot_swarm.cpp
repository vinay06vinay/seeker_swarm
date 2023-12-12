/*
 * @file node_robot_swarm.cpp
 * @brief This file contains the implementation of the RobotSwarm class
 * @author Neha Nitin Madhekar,Vinay Krishna Bukka, Rashmi Kapu
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// #include "my_dummy_lib_funct2.hpp"

// using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using STRING = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;

/**
 * @class RobotSwarm
 * @brief The RobotSwarm class represents a ROS 2 node for managing a swarm of
 * robots.
 *
 */
class RobotSwarm : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for RobotSwarm class.
   *
   * Initializes the RobotSwarm node, creates a publisher, and sets up a timer
   * with a callback function.
   */
  RobotSwarm() : Node("RobotSwarm"), count_(0) {
    // define topic name
    auto topicName = "topic";

    // creates publisher with buffer size of 10
    publisher_ = this->create_publisher<STRING>(topicName, 10);

    // creates 2 hz timer and ties the callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&RobotSwarm::timer_callback, this));
  }

  // id

 private:
  // Placeholder member variables for future functionalities
  // current_position
  // home_position
  // alert_status
  // camera_data
  //  Dummy Variables that will be removed later
  size_t count_;
  PUBLISHER publisher_;
  TIMER timer_;

  /**
   * @brief Timer callback function.
   *
   * This function is called at regular intervals by the timer. It creates a
   * STRING message, updates the count_, logs a message, and publishes the
   * message.
   */
  void timer_callback() {
    // Create the message to publish
    auto message = STRING();

    message.data = "Robot Swarm Node Called" + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "
                                               << " " << message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }

  /**
   * @brief Move to a specific position.
   *
   * Placeholder method for moving the robot to a specific position.
   * Implementation details can be added later.
   */
  // void MovotoPosition() {}

  /**
   * @brief Search for a path.
   *
   * Placeholder method for searching a path for the robot.
   * Implementation details can be added later.
   */
  // void SearchPath() {}

  /**
   * @brief Alert when a part is found.
   *
   * Placeholder method for alerting when a part is found.
   * Implementation details can be added later.
   */
  // void AlertPartFound() {}
};

/**
 * @brief Main function to run the RobotSwarm node.
 *
 * Initializes ROS 2, spins the RobotSwarm node, and shuts down ROS 2 on exit.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RobotSwarm>());

  rclcpp::shutdown();

  return 0;
}
