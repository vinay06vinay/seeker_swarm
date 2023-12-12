/*
 * @file node_warehouse_head.cpp
 * @brief This file contains the implementation of the WarehouseHead class
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
 * @class WarehouseHead
 * @brief The WarehouseHead class represents a ROS 2 node for warehouse head
 * operations.
 *
 */
class WarehouseHead : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for WarehouseHead class.
   *
   * Initializes the WarehouseHead node, creates a publisher, and sets up a
   * timer with a callback function.
   */
  WarehouseHead() : Node("WarehouseHead"), count_(0) {
    // define topic name
    auto topicName = "topic";

    // creates publisher with buffer size of 10
    publisher_ = this->create_publisher<STRING>(topicName, 10);

    // creates 2 hz timer and ties the callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&WarehouseHead::timer_callback, this));
  }

  // robot_list
  // alert_status

 private:
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

    message.data = "Warehouse Head Node Called" + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "
                                               << " " << message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }
  /**
   * @brief Placeholder for a method to receive alerts.
   *
   * This method is a placeholder for a future functionality to receive alerts.
   * It can be expanded as needed.
   */
  // void Receive_Alert() {}
};

/**
 * @brief Main function to run the WarehouseHead node.
 *
 * Initializes ROS 2, spins the WarehouseHead node, and shuts down ROS 2 on
 * exit.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WarehouseHead>());

  rclcpp::shutdown();

  return 0;
}
