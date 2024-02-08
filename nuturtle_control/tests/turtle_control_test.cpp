/// \file
/// \brief A node to test turtle control functionality.

#include "catch_ros2/catch_ros2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;

int left_velocity,right_velocity;
double left_position,right_position;
bool j_state = false;

// callback for wheel command subscriber
void wheel_cmd_callback(
    const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
  left_velocity = msg->left_velocity;
  right_velocity = msg->right_velocity;
}

void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  left_position = msg->position[0];
  right_position = msg->position[1];
  j_state = true;
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
  twist.angular.z = 0.0;

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

TEST_CASE("pure rotation", "[turtle control integration]") {
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
  twist.linear.x = 0.0;
  twist.angular.z = 1.0;

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
  REQUIRE(left_velocity == -101);
  REQUIRE(right_velocity == 101);
};

// TEST_CASE("sensor data to joint states", "[turtle control integration]") {
//   auto node = rclcpp::Node::make_shared("turtle_control_test");

//   // create publishers
//   auto sensor_data_pub_ =
//       node->create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

//   // create subscribers
//   auto joint_state =
//       node->create_subscription<sensor_msgs::msg::JointState>(
//           "joint_states", 10, &joint_state_callback);

//   // create sensor data message
//   nuturtlebot_msgs::msg::SensorData sensor_data;
//   sensor_data.stamp = rclcpp::Clock().now();
//   sensor_data.left_encoder = 1000;
//   sensor_data.right_encoder = 1000;

//   rclcpp::Time start_time = rclcpp::Clock().now();

//   // Keep test running only for the length of the "test_duration" parameter
//   // (in seconds)
//   while (rclcpp::ok() && j_state == false) {
//     sensor_data_pub_->publish(sensor_data);
//     rclcpp::spin_some(node);
//   }
//   // Test assertions - check that the dummy node was found
//  REQUIRE_THAT(left_position , Catch::Matchers::WithinRel(1.53398));
//  REQUIRE_THAT(right_position, Catch::Matchers::WithinRel(1.53398));
// };