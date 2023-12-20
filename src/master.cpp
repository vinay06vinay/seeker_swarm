/**
 * @file master.cpp
 * @brief This file contains the implementation of the SwarmMaster class
 * @author Neha Nitin Madhekar,Vinay Krishna Bukka, Rashmi Kapu
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */
#pragma once
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "node_robot_swarm.cpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

/**
 * @class SwarmMaster
 * @brief A class representing the master node that controls a swarm of robots.
 *
 * The SwarmMaster class inherits from the rclcpp::Node class and provides functionality to control and coordinate
 * a swarm of robots. It initializes a timer for periodic tasks and a publisher to send commands to the robots.
 */
class SwarmMaster : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;  // Change to publish to custom robot array
  std::vector<std::shared_ptr<RobotSwarm>> robot_array;
  int nodes;

 public:
  /**
     * @brief Constructor for the SwarmMaster class.
     *
     * Initializes the SwarmMaster with the provided robot array and the number of nodes in the swarm. It creates a
     * timer for periodic tasks and sets an initial goal for the robots.
     *
     * @param robot_array Vector of shared pointers to RobotSwarm instances.
     * @param nodes Number of robot nodes in the swarm.
     */
  SwarmMaster(std::vector<std::shared_ptr<RobotSwarm>> const &robot_array, int nodes): Node("master_node") {
        this->robot_array = robot_array;
        this->nodes = nodes;
        auto processCallback = std::bind(&SwarmMaster::process_callback, this);
        this->timer_ = this->create_wall_timer(100ms, processCallback);
        this->move(10.0);
    }

   /**
     * @brief Callback function for the timer.
     *
     * This function is called periodically by the timer. It logs a message indicating the iteration through the
     * Publisher array.
     */
  void process_callback(){
    RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
  }
  
  /**
     * @brief Sets a goal for each robot in the swarm.
     *
     * @param dist The distance to the goal for each robot.
     */
  void move(double dist){
    double h = 2 * 3.142 / this->nodes;
  int i = 0;
  for (i =0 ; i<10 ; i++) {
    this->robot_array[i]->set_goal(dist, double(i - (5 / 2)));
    
    }
  }

};






