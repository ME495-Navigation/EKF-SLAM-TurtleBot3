#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class NuSim : public rclcpp::Node
{
  public:
    NuSim()
    : Node("nusim"), timer_count_(0)
    {
      auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      timer_param_desc.description = "Timer frequency";
      declare_parameter("rate", 200.0, timer_param_desc);
      double timer_rate = get_parameter("rate").as_double();
      std::chrono::milliseconds rate = std::chrono::milliseconds(int(1000.0/timer_rate));

      timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

      reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

      // Initialize the transform broadcaster
      tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      timer_ = create_wall_timer(
      rate, std::bind(&NuSim::timer_callback, this));
    }

  private:
    void timer_callback()
    {   
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "nusim/world";
        t.child_frame_id = "red/base_footprint";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        //publish the current timestep
        auto message = std_msgs::msg::UInt64();
        message.data = timer_count_;
        timestep_publisher_->publish(message);
        timer_count_++;
    }

    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr, 
    std_srvs::srv::Empty::Response::SharedPtr)
    {
        timer_count_ = 0;
    }
    
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t timer_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}