/// \file
/// \brief Control commands for the turtlebot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&TurtleControl::timer_callback, this));
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
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
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
