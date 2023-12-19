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


class RobotSwarm : public rclcpp::Node {
 public:
  RobotSwarm(std::string node_name, std::string robot_name, bool navigate = false,
        double linear_speed = 1.0, double angular_speed = 0.0)
      : Node(node_name),
        robot_name_curr{robot_name},
        navigate_curr{navigate},
        linear_speed_curr{linear_speed},
        angular_speed_curr{angular_speed},
        roll_curr{0},
        pitch_curr{0},
        yaw_curr{0},
        // Kv{1},
        // Kh{1},
        goal_curr_x{0.0},
        goal_curr_y{0.0} {
    auto current_location = std::make_pair(3.0, 0.0);
    curr_loc = current_location;
    callback_grp = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    obstacle = false;
    auto command_topic_name = "/" + robot_name_curr + "/cmd_vel";
    auto pose_topic_name = "/" + robot_name_curr + "/odom";
    auto detection_topic_name = "/" + robot_name_curr + "/camera/image_raw";
    auto lidar_topic_name = "/" + robot_name_curr + "/scan";

    RCLCPP_INFO_STREAM(this->get_logger(), "RobotSwarm Constructor");
    vel_publisher_curr = this->create_publisher<geometry_msgs::msg::Twist>(
        command_topic_name, 10);
    goal_pub =
        this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
    robot_pose_curr =
        this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic_name, 10,
            std::bind(&RobotSwarm::robot_pose_callback, this,
                      std::placeholders::_1));

    lidar_sub_curr = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic_name, 10, std::bind(&RobotSwarm::lidar_callback, this, _1));

    cam_sub_curr =
        this->create_subscription<sensor_msgs::msg::Image>(
            detection_topic_name, 10,
            std::bind(&RobotSwarm::imageCallback, this,
                      std::placeholders::_1));
    subscriber_move_flag =
        this->create_subscription<std_msgs::msg::Bool>(
            "/red_object_detected", 10,
            std::bind(&RobotSwarm::move_flag_callback, this,
                      std::placeholders::_1));
    common_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/red_object_detected", 10);
    // Call on_timer function 5 times per second
    navigate_curr_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
        std::bind(&RobotSwarm::navigate_callback, this), callback_grp);
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
      navigate_curr = true;
      goal_curr_x = x;
      goal_curr_y = y;
      RCLCPP_INFO_STREAM(this->get_logger(), "Going to goal: [" << goal_curr_x << ","
                                                                << goal_curr_y
                                                              << "]");
  }

    void robot_pose_callback(const nav_msgs::msg::Odometry &msg) {
      curr_loc.first = msg.pose.pose.position.x;
      curr_loc.second = msg.pose.pose.position.y;
      m_orientation = msg.pose.pose.orientation;
    }

    double angle_resize_positive(double angle) {
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
      auto threshold = 2.0;
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

    double euclid_dist(const std::pair<double, double> &a,
                               const std::pair<double, double> &b) {
      return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
    }

    double yaw() {
      tf2::Quaternion q(m_orientation.x, m_orientation.y, m_orientation.z,
                        m_orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      return yaw;
    }
    double angle_resize(double angle) {
      const double result = fmod(angle + M_PI, 2.0 * M_PI);
      if (result <= 0.0) return result + M_PI;
      return result - M_PI;
    }

    void move(double linear, double angular) {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = linear;
      msg.angular.z = angular;
      vel_publisher_curr->publish(msg);
    }
    void stop() {
      navigate_curr = false;
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = 0;
      cmd_vel_msg.angular.z = 0;
      vel_publisher_curr->publish(cmd_vel_msg);

      std_msgs::msg::Bool goal_reached_msg;
      goal_reached_msg.data = true;
      goal_pub->publish(goal_reached_msg);
    }


    void navigate_callback() {
      if (!navigate_curr) return;

      std::pair<double, double> goal{goal_curr_x, goal_curr_y};
      double distance_to_goal = euclid_dist(curr_loc, goal);

      if (distance_to_goal > 0.1) {
        distance_to_goal = euclid_dist(curr_loc, goal);
        double angle_to_goal =
            std::atan2(goal_curr_y - curr_loc.second, goal_curr_x - curr_loc.first);

        if (angle_to_goal < 0)
          // angle_to_goal = 2 * M_PI + angle_to_goal;
          angle_to_goal = angle_resize_positive(angle_to_goal);

        // angle to rotate to face the goal
        double w = angle_to_goal - yaw();

        if (w > M_PI) {
          w = w - 2 * M_PI;
          // w = m_angle_resize_positive(w);
        }

        // proportional control for linear velocity
        // double velocity_x = std::min(Kv * distance_to_goal, linear_speed_curr);
          double velocity_x = linear_speed_curr;
        // proportional control for angular velocity
        // double angular_vel_z = Kh * w;
          double angular_vel_z = w;
        if (this->obstacle==false){
          if (angular_vel_z > 0)
            angular_vel_z = std::min(angular_vel_z, angular_speed_curr);
          else
            angular_vel_z = std::max(angular_vel_z, -angular_speed_curr);
        }
        else{
          velocity_x = 0;
          angular_vel_z = 3.14/2;
        }

        if(global_move_flag == false){
          move(velocity_x, angular_vel_z);
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
                    double contourAreaThreshold = 50000.0; // Adjust as needed

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
                        common_publisher_->publish(move_flag_local);
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
        std::string robot_name_curr;  
        bool navigate_curr;         
        double linear_speed_curr;    
        double angular_speed_curr; 
        double yaw_curr;  
        double roll_curr;            
        double pitch_curr;                        
        double goal_curr_x;
        double goal_curr_y;
        double m_distance_to_goal;
        bool global_move_flag{false};
        bool obstacle;

        rclcpp::CallbackGroup::SharedPtr callback_grp;
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_curr;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_curr;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_curr;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_move_flag;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr common_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_curr;
        std::pair<double, double> curr_loc;
        //   geometry_msgs::msg::Quaternion m_orientation;
        rclcpp::TimerBase::SharedPtr navigate_curr_timer;
};









