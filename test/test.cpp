/*
 * @file test.cpp
 * @brief This file contains the implementation of unit tests
 * @author Neha Nitin Madhekar,Vinay Krishna Bukka, Rashmi Kapu
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class TestNode
 * @brief Test fixture for testing ROS 2 node functionalities.
 *
 * This test fixture includes two test cases for testing publisher and subscriber
 * functionalities using ROS 2 nodes.
 */
class TestNode : public testing::TestWithParam<int> {
  // Your test fixture code here
public:
  rclcpp::Node::SharedPtr node1_;
  rclcpp::Node::SharedPtr node2_;
};

/**
 * @brief Test case for testing the publisher functionality.
 *
 * This test creates a ROS 2 node, creates a publisher, checks the number of publishers
 * on a topic, and asserts that the number matches the expected value.
 */
TEST_F(TestNode, test_for_publisher) {
  node1_ = std::make_shared<rclcpp::Node>("test_publisher");
  auto test_publisher =
      node1_->create_publisher<std_msgs::msg::String>("topic", 10);
  auto publishers_number = node1_->count_publishers("topic");
  EXPECT_EQ(3, static_cast<int>(publishers_number));
}

/**
 * @brief Test case for testing the subscriber functionality.
 *
 * This test creates a second ROS 2 node, subscribes to a topic, checks if the received
 * message is not null, and waits for a moment to allow the subscription to be established.
 */
TEST_F(TestNode, test_for_subscriber) {
  node2_ = std::make_shared<rclcpp::Node>("test_subscriber");
  
  // Subscribe to the topic with a callback function
  auto subscription = node2_->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        // Check if the received message is not null
        ASSERT_NE(nullptr, msg);
      });

  // Wait for a moment to allow the subscription to be established
  rclcpp::sleep_for(std::chrono::seconds(1));
}

/**
 * @brief Main function to run the ROS 2 node tests.
 *
 * Initializes ROS 2, runs all the tests, and shuts down ROS 2 on exit.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit code.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}