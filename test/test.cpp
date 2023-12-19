#include <gtest/gtest.h>
#include "../src/node_robot_swarm.cpp"  

class RobotTest : public testing::Test {
 protected:
  std::shared_ptr<Robot> robot;
  // std::shared_ptr<Master> master;
};

///////////////////////////////////////////////////////////
/// Tests
///////////////////////////////////////////////////////////
// TEST_F(RobotTest, robot_initialization) {
//   auto r_namespace = "robot_" + std::to_string(1);
//   auto nodename = "robot_" + std::to_string(1) + "_controller";
//   robot = std::make_shared<Robot>(nodename, r_namespace);
//   robot->set_goal(5.0, 5.0);
//   robot->go_to_goal_callback();
//   EXPECT_EQ(1, 1);
// }


// Testing Number of Slaves Spawned Counting the publihsers
TEST_F(RobotTest, slave_spawn_testing_publishers) {
  int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace);
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

// // Testing Number of Slaves Spawned_Counting the subscribers
// TEST_F(RobotTest, slave_spawn_testing_subscribers) {
//   int nodes = 10;
//   int pub_count = 0;
//   rclcpp::executors::MultiThreadedExecutor exec;
//   std::vector<std::shared_ptr<Robot>> robot_array;
//   for (int i = 0; i < nodes; i++) {
//     auto r_namespace = "robot_" + std::to_string(i);
//     auto nodename = "robot_" + std::to_string(i) + "_controller";
//     auto robot = std::make_shared<Robot>(nodename, r_namespace);
//     exec.add_node(robot);
//     robot_array.push_back(robot);
//   }
//   for (int i = 0; i < nodes; i++) {
//     auto r_namespace = "robot_" + std::to_string(i);
//     auto number_of_subs =
//         robot_array[i]->count_subscribers("/" + r_namespace + "/odom");
//     pub_count = pub_count + static_cast<int>(number_of_subs);
//   }
//   EXPECT_EQ(nodes, pub_count);
// }

// Check Method Compute Distance
TEST_F(RobotTest, euclid_dist) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
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
  robot = std::make_shared<Robot>(nodename, r_namespace);
  double angle = robot->angle_resize(-10.14);
  double ex = 2.42637;
  EXPECT_NEAR(ex, angle, 0.1);
}

// Check Method Nomrmalize Angle Positive
TEST_F(RobotTest, test_angle_resize_positive) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  double angle = robot->angle_resize_positive(-7.29);
  double ex = 5.2763;
  EXPECT_NEAR(ex, angle, 0.1);
}

// Check Method Yaw From Quaternions
TEST_F(RobotTest, yaw) {
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
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
  robot = std::make_shared<Robot>(nodename, r_namespace);

  auto number_of_publishers =
      robot->count_publishers("/" + r_namespace + "/cmd_vel");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}