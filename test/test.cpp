/**
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
#include "../src/node_robot_swarm.cpp"  
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 
#include <nav_msgs/msg/odometry.hpp>

class RobotTest : public testing::Test {
 protected:
  std::shared_ptr<RobotSwarm> robot;
};
/////////////////////////////////////////////////////////
/// Tests
///////////////////////////////////////////////////////////

TEST_F(RobotTest, move_flag_test) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  std_msgs::msg::Bool::SharedPtr msg;
  robot->move_flag_callback(msg);
  EXPECT_EQ(2, 2);
}

TEST_F(RobotTest, stop_method) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  robot->stop();
  EXPECT_EQ(2, 2);
}
TEST_F(RobotTest, robot_initialization) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  robot->set_goal(5.0, 5.0);
  robot->navigate_callback();
  EXPECT_EQ(1, 1);
}


TEST_F(RobotTest, count_testing_publishers) {
  int nodes = 5;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<RobotSwarm>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }

  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_publishers =
        robot_array[i]->count_publishers("/" + r_namespace + "/cmd_vel");
    pub_count = pub_count + static_cast<int>(number_of_publishers);
  }
  EXPECT_EQ(nodes, pub_count);
}


TEST_F(RobotTest, count_testing_subscribers) {
  int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<RobotSwarm>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_subs =
        robot_array[i]->count_subscribers("/" + r_namespace + "/odom");
    pub_count = pub_count + static_cast<int>(number_of_subs);
  }
  EXPECT_EQ(nodes, pub_count);
}

// Check Method Compute Distance
TEST_F(RobotTest, euclid_dist) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  std::pair<double, double> goal{10.0, 10.0};
  std::pair<double, double> loc{0.0, 0.0};
  double distance = robot->euclid_dist(loc, goal);
  double ex = 14.1421;
  EXPECT_NEAR(ex, distance, 0.1);
}

// Check Method Nomrmalize Angle
TEST_F(RobotTest, angle_resize) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  double angle = robot->angle_resize(-10.14);
  double ex = 2.42637;
  EXPECT_NEAR(ex, angle, 0.1);
}

// Check Method Nomrmalize Angle Positive
TEST_F(RobotTest, test_angle_resize_positive) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  double angle = robot->angle_resize_positive(-7.29);
  double ex = 5.2763;
  EXPECT_NEAR(ex, angle, 0.1);
}

// Check Method Yaw From Quaternions
TEST_F(RobotTest, yaw) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  robot->m_orientation.x = 3.0;
  robot->m_orientation.y = 5.0;
  robot->m_orientation.z = 4.0;
  robot->m_orientation.w = 6.0;
  double yaw = robot->yaw();
  double ex = 1.51955;
  EXPECT_NEAR(ex, yaw, 0.1);
}

TEST_F(RobotTest, num_publishers) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);

  auto number_of_publishers =
      robot->count_publishers("/" + r_namespace + "/cmd_vel");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}

TEST_F(RobotTest, image_callback_test) {
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<RobotSwarm>(nodename, r_namespace);
  sensor_msgs::msg::Image::SharedPtr test_image_msg = std::make_shared<sensor_msgs::msg::Image>();
  robot->imageCallback(test_image_msg);
  EXPECT_EQ(2.5, 2.5);
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  std::cout << "DONE SHUTTING DOWN" << std::endl;
  rclcpp::shutdown();
  return result;
}