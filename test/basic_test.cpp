// Description: Test if a simple task plan works

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <std_msgs/msg/string.hpp>

class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture()
      : node_(std::make_shared<rclcpp::Node>("basic_test"))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup should occur before every test instance.
    RCLCPP_INFO_STREAM(node_->get_logger(), "SETUP!!");

    /*
     * 1.) Define any ros2 package and exectuable you want to test
     *  example: package name = cpp_pubsub, node name = minimal_publisher, executable = talker
     */
    bool retVal = StartROSExec ("my_controller", "minimal_publisher", "talker");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    // Tear-down should occur after every test instance 
    RCLCPP_INFO_STREAM(node_->get_logger(), "TEARDOWN!!");

    // Stop the running ros2 node, if any.
    bool retVal = StopROSExec ();
    ASSERT_TRUE(retVal);

    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec (const char* pkg_name,
                     const char* node_name,
                     const char* exec_name)
  {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info " << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];  snprintf (execName, 16, "%s", exec_name); // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();
    
    // Start a ros2 node and wait for it to get ready:
    int retVal =  system (cmd_ss.str().c_str());
    if (retVal != 0)
      return false;
    
    // Wait for at most 10 seconds for the node to show up, otherwise it's an error!
    retVal = -1;
    int count  = 0;
    while ((count++ < 10) && (retVal != 0)) {
      retVal = system (cmdInfo_ss.str().c_str());
      sleep (1);
    }
    return (retVal == 0);
  }

  bool StopROSExec ()
  {
    // if node is not running, don't need to kill it
    if ((killCmd_ss.str().empty()) ||
        system (cmdInfo_ss.str().c_str()) != 0)
      return true;
    
    int retVal = system (killCmd_ss.str().c_str());
    return retVal == 0;
  }
  
};

TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic 
   */
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription = node_->create_subscription<String>
    ("topic", 10,
     // Lambda expression begins
     [&](const std_msgs::msg::String& msg) {
       RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
       hasData = true;
     } // end of lambda expression
     );

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);       // 2hz checks
  while ((elapsed_time < 3s) && !hasData)
    {
      rclcpp::spin_some(node_);
      rate.sleep();
      elapsed_time = timer::now() - clock_start;
    }
  EXPECT_TRUE (hasData);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
