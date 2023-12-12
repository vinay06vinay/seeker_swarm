#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// #include "my_dummy_lib_funct2.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

class WarehouseHead: public rclcpp::Node {
public:

  WarehouseHead()
    : Node("WarehouseHead"),
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
        std::bind(&WarehouseHead::timer_callback, this));
  }

  // robot_list
  // alert_status  

private:
  //  Dummy Variables that will be removed later
  size_t    count_;
  PUBLISHER publisher_;
  TIMER     timer_;


  void timer_callback()
  {
    // Create the message to publish
    auto message = STRING();

    message.data = "Warehouse Head Node Called" + std::to_string(count_++);
    RCLCPP_INFO_STREAM (this->get_logger(),
                        "Publishing: " << " " << message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }
  void Receive_Alert(){}
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WarehouseHead>());

  rclcpp::shutdown();

  return 0;
}