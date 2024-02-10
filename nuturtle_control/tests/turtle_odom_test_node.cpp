/// \file
/// \brief A node to test odometry node functionality.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

TEST_CASE("tf listener", "[odometry integration]") {

  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // create publishers
  auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // create a joint state message
  std::vector<std::string> joint_names = {"left", "right"};
  std::vector<double> joint_positions = {0.0, 0.0};

  sensor_msgs::msg::JointState js;
  js.header.stamp = rclcpp::Clock().now();
  js.header.frame_id = "base_footprint";
  js.name = joint_names;
  js.position = joint_positions;

  // create a tf listener
  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  std::string fromFrameRel = "base_footprint";
  std::string toFrameRel = "odom";

  rclcpp::Time start_time = rclcpp::Clock().now();

  // initialize a transform
  geometry_msgs::msg::TransformStamped tr;

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    joint_pub->publish(js);
    try {
      tr = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
    }
    rclcpp::spin_some(node);
  }

  // Test assertions
  REQUIRE(tr.transform.translation.x == 0.0);
  REQUIRE(tr.transform.translation.y == 0.0);
  REQUIRE(tr.transform.translation.z == 0.0);
  REQUIRE(tr.transform.rotation.x == 0.0);
  REQUIRE(tr.transform.rotation.y == 0.0);
  REQUIRE(tr.transform.rotation.z == 0.0);
  REQUIRE(tr.transform.rotation.w == 1.0);
}

TEST_CASE("initial_pose service", "[odometry integration]") {

  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // create publishers
  auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // create a joint state message
  std::vector<std::string> joint_names = {"left", "right"};
  std::vector<double> joint_positions = {0.0, 0.0};

  sensor_msgs::msg::JointState js;
  js.header.stamp = rclcpp::Clock().now();
  js.header.frame_id = "base_footprint";
  js.name = joint_names;
  js.position = joint_positions;

  // create a tf listener
  auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  std::string fromFrameRel = "base_footprint";
  std::string toFrameRel = "odom";

  // initialize a transform
  geometry_msgs::msg::TransformStamped t;

  // Create a client for the service we're looking for
  auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

  // create a client message
  auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
  request->x = 1.0;
  request->y = 2.0;
  request->theta = 0.0;

  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    joint_pub->publish(js);
    // Repeatedly check for the dummy service until its found
    if (client->wait_for_service(0s)) {
      service_found = true;
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        auto flag = result.get()->success;
        if (flag) 
        {
          try {
              t = tf_buffer_->lookupTransform(
              toFrameRel, fromFrameRel,
              tf2::TimePointZero);
              } 
          catch (const tf2::TransformException & ex) {}
        }
      }
    }
    rclcpp::spin_some(node);
  }

  // Test assertions
  CHECK(service_found);
    // Test assertions
  REQUIRE(t.transform.translation.x == 1.0);
  REQUIRE(t.transform.translation.y == 2.0);
  REQUIRE(t.transform.translation.z == 0.0);
  REQUIRE(t.transform.rotation.x == 0.0);
  REQUIRE(t.transform.rotation.y == 0.0);
  REQUIRE(t.transform.rotation.z == 0.0);
  REQUIRE(t.transform.rotation.w == 1.0);
}
