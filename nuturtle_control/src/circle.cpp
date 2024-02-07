/// \file
/// \brief A control node to drive the turtlebot along a circular arc.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


/// \brief A control node to drive the turtlebot along a circular arc.
class Circle : public rclcpp::Node
{
public:
  Circle() : Node("circle"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("frequency", 100.0, timer_param_desc);
    double timer_rate = get_parameter("frequency").as_double();
    std::chrono::milliseconds frequency = std::chrono::milliseconds(int(1000.0 / timer_rate));
    
    // Create timer
    timer_ = create_wall_timer(
      frequency, std::bind(&Circle::timer_callback, this));
    
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
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
