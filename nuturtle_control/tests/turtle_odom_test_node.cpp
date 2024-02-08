/// \file
/// \brief A node to test odometry node functionality.

#include "catch_ros2/catch_ros2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

TEST_CASE("initial_pose service", "[odometry integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
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

  // Create a client for the service we're looking for
  auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");


  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Repeatedly check for the dummy service until its found
    if (client->wait_for_service(0s)) {
      service_found = true;
      break;
    }

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(service_found);
}