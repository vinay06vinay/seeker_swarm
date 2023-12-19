#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <utility>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;


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
    obstacle = false;
    auto command_topic_name = "/" + m_robot_name + "/cmd_vel";
    auto pose_topic_name = "/" + m_robot_name + "/odom";
    auto detection_topic_name = "/" + m_robot_name + "/camera/image_raw";
    auto lidar_topic_name = "/" + m_robot_name + "/scan";

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

    m_lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic_name, 10, std::bind(&Robot::lidar_callback, this, _1));

    m_subscriber_image =
        this->create_subscription<sensor_msgs::msg::Image>(
            detection_topic_name, 10,
            std::bind(&Robot::imageCallback, this,
                      std::placeholders::_1));
    m_subscriber_move_flag =
        this->create_subscription<std_msgs::msg::Bool>(
            "/red_object_detected", 10,
            std::bind(&Robot::move_flag_callback, this,
                      std::placeholders::_1));
    m_red_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/red_object_detected", 10);
    // Call on_timer function 5 times per second
    m_go_to_goal_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&Robot::go_to_goal_callback, this), m_cbg);
        }
    void move_flag_callback(const std_msgs::msg::Bool::SharedPtr msg)
      {
          // Implementation of the callback method
          global_move_flag = true;
          if (global_move_flag)
          {
              RCLCPP_INFO(this->get_logger(), "Received true from /red_object_detected. Start moving!");
              // Implement your logic for moving the robot
              // global_move_flag = true;

          }
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

    void lidar_callback(const sensor_msgs::msg::LaserScan& msg) {
    // if (msg.header.stamp.sec == 0) {
    //   return;
    // }
      auto scan_data = msg.ranges;

    // Setting field of view = -50 to +50 degrees
      int min_angle = 330;
      int max_angle = 60;
      auto threshold = 1.5;
      for (int i = min_angle; i < min_angle + max_angle; i++) {
        if (scan_data[i % 360] < threshold) {
          // turn
          RCLCPP_INFO_STREAM(this->get_logger(),
                          "********** Obstacle**********");
          obstacle = true;
        } else {
          // move forward
          RCLCPP_INFO_STREAM(this->get_logger(),
                          "********** No obstacle**********");
          obstacle = false;
        }
      }
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
        if (this->obstacle==false){
          if (angular_z > 0)
            angular_z = std::min(angular_z, m_angular_speed);
          else
            angular_z = std::max(angular_z, -m_angular_speed);
        }
        else{
          linear_x = 0;
          angular_z = 3.14/2;
        }

        if(global_move_flag == false){
          move(linear_x, angular_z);
        } else{
          move(0.0, 0.0);
        }
      } else {
        RCLCPP_INFO_STREAM(this->get_logger(),
                          "********** Goal reached **********");
        stop();
      }
    
      }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat hsv_image;
            cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

            // Define the range of red color in HSV
            cv::Scalar lower_red(0, 100, 100);
            cv::Scalar upper_red(10, 255, 255);

            // Threshold the HSV image to get only red colors
            cv::Mat red_mask;
            cv::inRange(hsv_image, lower_red, upper_red, red_mask);
            // Find contours in the binary image
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (const auto& contour : contours)
                {
                    // Calculate the area of the contour
                    double area = cv::contourArea(contour);

                    // Set your desired contour area threshold
                    double contourAreaThreshold = 80000.0; // Adjust as needed

                    // Check if the contour area exceeds the threshold
                    if (area > contourAreaThreshold)
                    {
                        // Draw the contour on the original image
                        // cv::drawContours(cv_ptr->image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);

                        // Display a message
                        RCLCPP_INFO(this->get_logger(), "Contour area: %s", std::to_string(area).c_str());
                        RCLCPP_INFO(this->get_logger(), "Red object detected with a large enough area!");

                        // Stop further processing or add your desired logic here

                        // For example, you can publish a message or set a flag to stop processing
                        std_msgs::msg::Bool move_flag_local;
                        move_flag_local.data = true;
                        // move(0, 0);
                        m_red_publisher_->publish(move_flag_local);
                        // msg.data = "Red object detected with a large enough area!";
                        // red_publisher_->publish(msg);

                        // You might want to add a break here to exit the loop if you only want to consider the first contour
                        // break;
                    }
                }
            // Check if any red pixels are present
            // if (cv::countNonZero(red_mask) > 0)
            // {
            //     RCLCPP_INFO(this->get_logger(), "Red object detected!");
            //     std_msgs::msg::String msg;
            //     // msg.data = "Red object detected!";
            //     // red_publisher_->publish(msg);
            // }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
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
        bool global_move_flag{false};
        bool obstacle;

        rclcpp::CallbackGroup::SharedPtr m_cbg;
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_goal_reached_publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_robot3_pose;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_subscriber_image;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriber_move_flag;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_red_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lidar_sub;
        std::pair<double, double> m_location;
        //   geometry_msgs::msg::Quaternion m_orientation;
        rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;
};









