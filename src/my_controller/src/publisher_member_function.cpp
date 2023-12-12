// Copyright 2016 Open Source Robotics Foundation, Inc.

//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "my_dummy_lib_funct2.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

class MinimalPublisher : public rclcpp::Node {
public:

  MinimalPublisher()
    : Node("minimal_publisher"),
    count_(0)
  {
    // define topic name
    auto topicName = "topic";

    // creates publisher with buffer size of 10
    publisher_ = this->create_publisher<STRING>(topicName, 10);

    // creates 2 hz timer and ties the callback function
    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(&MinimalPublisher::timer_callback, this));
  }

private:

  size_t    count_;
  PUBLISHER publisher_;
  TIMER     timer_;

  void timer_callback()
  {
    // Create the message to publish
    auto message = STRING();

    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO_STREAM (this->get_logger(),
                        "Publishing: " << function2 (count_) << " " << message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[])
{
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}
