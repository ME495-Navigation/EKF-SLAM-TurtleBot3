/// \file
/// \brief A node to test circle node functionality.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int timer_count = 0;
int delta_timer = 1;
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr)
{
  timer_count++;
}

TEST_CASE("cmd_vel frequency", "[circle integration]") {

  auto node = rclcpp::Node::make_shared("circle_integration_test");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // create a subscription to the cmd_vel topic
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, &cmd_vel_callback);

  auto client = node->create_client<nuturtle_control::srv::Control>("control");

  auto request = std::make_shared<nuturtle_control::srv::Control::Request>();
  request->velocity = 2.0;
  request->radius = 1.0;

  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Repeatedly check for the dummy service until its found
    if (client->wait_for_service(0s)) {
      service_found = true;
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
      {
        auto flag = result.get()->success;
        if (flag) {
        // reset the timer count
        timer_count = 0;
        // wait for 100 milliseconds
        rclcpp::Time time_now = rclcpp::Clock().now();
        while ( rclcpp::Clock().now() < time_now + 100ms){rclcpp::spin_some(node);}
        delta_timer = timer_count;
      }
      }
      break;
    }
  }
  CHECK(service_found);
  REQUIRE_THAT(delta_timer, Catch::Matchers::WithinAbs(20.0,0.5));
}