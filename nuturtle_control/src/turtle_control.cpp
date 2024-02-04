/// \file
/// \brief Control commands for the turtlebot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// \brief Control commands for the turtlebot.
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("rate", 200.0, timer_param_desc);
    double timer_rate = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = std::chrono::milliseconds(int(1000.0 / timer_rate));

    declare_parameter("wheel_radius", -1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if (wheel_radius < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter wheel_radius was not set");
      rclcpp::shutdown();
    }
    declare_parameter("track_width", -1.0);
    track_width = get_parameter("track_width").as_double();
    if (track_width < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter track_width was not set");
      rclcpp::shutdown();
    }
    declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_max < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter motor_cmd_max was not set");
      rclcpp::shutdown();
    }
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter motor_cmd_per_rad_sec was not set");
      rclcpp::shutdown();
    }
    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter encoder_ticks_per_rad was not set");
      rclcpp::shutdown();
    }
    declare_parameter("collision_radius", -1.0);
    collision_radius = get_parameter("collision_radius").as_double();
    if (collision_radius < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter collision radius was not set");
      rclcpp::shutdown();
    }

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&TurtleControl::timer_callback, this));
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t timer_count_;
  double wheel_radius, track_width, motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;

  /// \brief The timer callback
  void timer_callback()
  {
    timer_count_++;
  }
};

/// \brief The main fucntion.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
