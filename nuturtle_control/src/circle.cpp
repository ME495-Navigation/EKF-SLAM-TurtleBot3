/// \file
/// \brief A control node to drive the turtlebot along a circular arc.

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief A control node to drive the turtlebot along a circular arc.
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("frequency", 100.0, timer_param_desc);
    double timer_rate = get_parameter("frequency").as_double();
    std::chrono::milliseconds frequency =
      std::chrono::milliseconds(int(1000.0 / timer_rate));

    // Create services
    control_ = create_service<nuturtle_control::srv::Control>(
      "control", std::bind(
        &Circle::control_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    reverse_ = create_service<std_srvs::srv::Empty>(
      "reverse", std::bind(
        &Circle::reverse_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    stop_ = create_service<std_srvs::srv::Empty>(
      "stop", std::bind(
        &Circle::stop_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Create publishers
    velocity_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create timer
    timer_ =
      create_wall_timer(frequency, std::bind(&Circle::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  size_t timer_count_;
  geometry_msgs::msg::Twist vel_;
  bool pub_start_ = false;

  /// \brief The timer callback
  void timer_callback()
  {
    if (pub_start_) {
      velocity_publisher_->publish(vel_);
    }
  }

  /// \brief The control callback
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr req,
    nuturtle_control::srv::Control::Response::SharedPtr res)
  {
    vel_.linear.x = (req->velocity) * (req->radius);
    vel_.angular.z = req->velocity;
    res->success = true;
    pub_start_ = true;
  }

  /// \brief The reverse callback
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    vel_.linear.x = -vel_.linear.x;
    vel_.angular.z = -vel_.angular.z;
  }
  /// \brief The stop callback
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // stop publishing

    pub_start_ = false;

    // publish zero velocity once and stop publishing
    vel_.linear.x = 0.0;
    vel_.angular.z = 0.0;
    velocity_publisher_->publish(vel_);
  }
};

/// \brief The main fucntion.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
