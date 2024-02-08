/// \file
/// \brief A node to test turtle control functionality.

#include "catch_ros2/catch_ros2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

int left_velocity;
int right_velocity;
// callback for wheel command subscriber
void wheel_cmd_callback(
    const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
  left_velocity = msg->left_velocity;
  right_velocity = msg->right_velocity;
}

TEST_CASE("pure translation", "[turtle control integration]") {
  auto node = rclcpp::Node::make_shared("turtle_control_test");

  // declare parameters
  node->declare_parameter<double>("test_duration");

  // create publishers
  auto cmd_vel_pub =
      node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // create subscribers
  auto wheel_cmd =
      node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
          "wheel_cmd", 10, &wheel_cmd_callback);

  // create twist message
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.01;
  twist.linear.z = 0.0;

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
      node->get_parameter("test_duration").get_parameter_value().get<double>();

  rclcpp::Time start_time = rclcpp::Clock().now();

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (rclcpp::ok() && ((rclcpp::Clock().now() - start_time) <
                          rclcpp::Duration::from_seconds(TEST_DURATION))) {
    cmd_vel_pub->publish(twist);
    rclcpp::spin_some(node);
  }
  // Test assertions - check that the dummy node was found
  REQUIRE(left_velocity == 12);
  REQUIRE(right_velocity == 12);
};
