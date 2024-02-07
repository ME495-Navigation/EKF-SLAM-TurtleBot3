/// \file
/// \brief Odometry node for the turtlebot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/// \brief Odometry node for the turtlebot.
class Odometry : public rclcpp::Node
{
public:
  Odometry() : Node("odometry"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("rate", 200.0, timer_param_desc);
    double timer_rate = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = std::chrono::milliseconds(int(1000.0 / timer_rate));

    declare_parameter("body_id", "base_link");
    body_id = get_parameter("body_id").as_string();
    if (body_id == "base_link"){
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter body_id was not set");
      rclcpp::shutdown();
    }

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&Odometry::timer_callback, this));
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::string body_id;
  size_t timer_count_;

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
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}