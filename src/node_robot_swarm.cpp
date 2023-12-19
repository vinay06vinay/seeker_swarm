
#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <utility>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"



class Robot : public rclcpp::Node {
 public:
  Robot(std::string node_name, std::string robot_name, bool go_to_goal = false,
        double linear_speed = 1.0, double angular_speed = 0.5)
      : Node(node_name),
        m_robot_name{robot_name},
        m_go_to_goal{go_to_goal},
        m_linear_speed{linear_speed},
        m_angular_speed{angular_speed},
        m_roll{0},
        m_pitch{0},
        m_yaw{0},
        m_kv{1},
        m_kh{1},
        m_goal_x{0.0},
        m_goal_y{0.0} {
    auto current_location = std::make_pair(3.0, 0.0);
    m_location = current_location;
    m_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto command_topic_name = "/" + m_robot_name + "/cmd_vel";
    auto pose_topic_name = "/" + m_robot_name + "/odom";

    RCLCPP_INFO_STREAM(this->get_logger(), "Robot Constructor");
    m_publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        command_topic_name, 10);
    m_goal_reached_publisher =
        this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
    m_subscriber_robot3_pose =
        this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic_name, 10,
            std::bind(&Robot::robot_pose_callback, this,
                      std::placeholders::_1));
    // Call on_timer function 5 times per second
    m_go_to_goal_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Robot::go_to_goal_callback, this), m_cbg);
        }


    void set_goal(double x, double y) {
      m_go_to_goal = true;
      m_goal_x = x;
      m_goal_y = y;
      RCLCPP_INFO_STREAM(this->get_logger(), "Going to goal: [" << m_goal_x << ","
                                                                << m_goal_y
                                                              << "]");
  }

    void robot_pose_callback(const nav_msgs::msg::Odometry &msg) {
      m_location.first = msg.pose.pose.position.x;
      m_location.second = msg.pose.pose.position.y;
      m_orientation = msg.pose.pose.orientation;
    }

    double normalize_angle_positive(double angle) {
      const double result = fmod(angle, 2.0 * M_PI);
      if (result < 0) return result + 2.0 * M_PI;
      return result;
    }

    double compute_distance(const std::pair<double, double> &a,
                               const std::pair<double, double> &b) {
      return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
    }

    double compute_yaw_from_quaternion() {
      tf2::Quaternion q(m_orientation.x, m_orientation.y, m_orientation.z,
                        m_orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      return yaw;
    }
    double normalize_angle(double angle) {
      const double result = fmod(angle + M_PI, 2.0 * M_PI);
      if (result <= 0.0) return result + M_PI;
      return result - M_PI;
    }

    void move(double linear, double angular) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = linear;
      msg.angular.z = angular;
      m_publisher_cmd_vel->publish(msg);
    }
    void stop() {
      m_go_to_goal = false;
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = 0;
      cmd_vel_msg.angular.z = 0;
      m_publisher_cmd_vel->publish(cmd_vel_msg);

      std_msgs::msg::Bool goal_reached_msg;
      goal_reached_msg.data = true;
      m_goal_reached_publisher->publish(goal_reached_msg);
    }


    void go_to_goal_callback() {
      if (!m_go_to_goal) return;

      std::pair<double, double> goal{m_goal_x, m_goal_y};
      double distance_to_goal = compute_distance(m_location, goal);

      if (distance_to_goal > 0.1) {
        distance_to_goal = compute_distance(m_location, goal);
        double angle_to_goal =
            std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);

        if (angle_to_goal < 0)
          // angle_to_goal = 2 * M_PI + angle_to_goal;
          angle_to_goal = normalize_angle_positive(angle_to_goal);

        // angle to rotate to face the goal
        double w = angle_to_goal - compute_yaw_from_quaternion();

        if (w > M_PI) {
          w = w - 2 * M_PI;
          // w = m_normalize_angle_positive(w);
        }

        // proportional control for linear velocity
        double linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);

        // proportional control for angular velocity
        double angular_z = m_kh * w;
        if (angular_z > 0)
          angular_z = std::min(angular_z, m_angular_speed);
        else
          angular_z = std::max(angular_z, -m_angular_speed);

        move(linear_x, angular_z);
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                          "********** Goal reached **********");
        stop();
      }
    
      }

      geometry_msgs::msg::Quaternion m_orientation;

      private:
        // attributes
        std::string m_robot_name;  // robot name used for creating namespace
        bool m_go_to_goal;         // flag to store if the robot has reached position
        double m_linear_speed;     // base linear velocity of robot
        double m_angular_speed;    // base angular velocity of robot
        double m_roll;             // rad
        double m_pitch;            // rad
        double m_yaw;              // rad
        double m_kv;               // gain for linear velocity
        double m_kh;               // gain for angular velocity
        double m_goal_x;
        double m_goal_y;
        double m_distance_to_goal;

        rclcpp::CallbackGroup::SharedPtr m_cbg;
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_goal_reached_publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_robot3_pose;
        std::pair<double, double> m_location;
        //   geometry_msgs::msg::Quaternion m_orientation;
        rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;
};










