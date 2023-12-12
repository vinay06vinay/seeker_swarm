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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "my_dummy_lib_funct1.hpp"

using std::placeholders::_1;
using std_msgs::msg::String;

using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;

using STRING_MSG = std_msgs::msg::String;

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber(
    const std::string& node_name      = "my_node",
    const std::string& node_namespace = "/my_ns",
    const std::string& topic_name     = "my_topic")
    : Node(node_name, node_namespace)
  {
    for (int idx = 0; idx < numSubs; idx++)
    {
      std::string subName = "subscription" + std::to_string(idx);
      std::function<void(const STRING_MSG& msg)> callback =
        std::bind(&MinimalSubscriber::topic_callback, this, _1, subName);
      subscriptions_[idx] = this->create_subscription<String>(
        topic_name,
        10,
        callback);
    }

    function1 (23);             // test model library
  }

private:

  void topic_callback(const STRING_MSG& msg, std::string subName)
  {
    RCLCPP_INFO (this->get_logger(), "subName=%s, I heard : '%s'",
                 subName.c_str(), msg.data.c_str());
  }

  int numSubs                           = 5;
  std::vector<SUBSCRIBER>subscriptions_ = std::vector<SUBSCRIBER>(numSubs);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>("Listen_Node", "/", "topic"));
  rclcpp::shutdown();

  return 0;
}
