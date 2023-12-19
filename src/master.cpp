
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


class Master : public rclcpp::Node {
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;  // Change to publish to custom robot array
  std::vector<std::shared_ptr<Robot>> robot_array;
  int nodes;

 public:
  Master(std::vector<std::shared_ptr<Robot>> const &robot_array, int nodes): Node("master_node") {
        this->robot_array = robot_array;
        this->nodes = nodes;
        auto processCallback = std::bind(&Master::process_callback, this);
        this->timer_ = this->create_wall_timer(100ms, processCallback);
        this->move(10.0);
    }

  void process_callback(){
    RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
  }
  
  void move(double dist){
    double h = 2 * 3.142 / this->nodes;
  int i = 0;
  for (i =0 ; i<10 ; i++) {
    this->robot_array[i]->set_goal(dist, double(i - (5 / 2)));
    
    }
  }

};






