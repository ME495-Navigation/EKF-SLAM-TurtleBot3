#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

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

        declare_parameter("x0", 0.0);
        x_tele = get_parameter("x0").as_double();
        reset_x = get_parameter("x0").as_double();

        declare_parameter("y0", 0.0);
        y_tele = get_parameter("y0").as_double();
        reset_y = get_parameter("y0").as_double();
        
        declare_parameter("theta0", 0.0);
        theta_tele = get_parameter("theta0").as_double(); 
        reset_theta = get_parameter("theta0").as_double();
        
        timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        reset_ = create_service<std_srvs::srv::Empty>(
        "~/reset", std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        teleport_ = create_service<nusim::srv::Teleport>(
        "~/teleport", std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize the transform broadcaster
        tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = create_wall_timer(
        rate, std::bind(&NuSim::timer_callback, this));
    }

  private:
    void timer_callback()
    {   
        //publish the current timestep
        auto message = std_msgs::msg::UInt64();
        message.data = timer_count_;
        timestep_publisher_->publish(message);
        timer_count_++;
        transform_publisher();
    }

    void reset_callback(std_srvs::srv::Empty::Request::SharedPtr, 
    std_srvs::srv::Empty::Response::SharedPtr)
    {
        timer_count_ = 0;
        x_tele = reset_x;
        y_tele = reset_y;
        theta_tele = reset_theta;
    }

    void transform_publisher()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "nusim/world";
        t.child_frame_id = "red/base_footprint";

        t.transform.translation.x = x_tele;
        t.transform.translation.y = y_tele;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = theta_tele;
        t.transform.rotation.w = 1.0;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void teleport_callback(nusim::srv::Teleport::Request::SharedPtr request, 
    nusim::srv::Teleport::Response::SharedPtr response)
    {
    x_tele = request->x0;
    y_tele = request->y0;
    theta_tele = request->theta0;
    RCLCPP_INFO_STREAM(get_logger(), "Teleporting to x:" << x_tele << " y:" << y_tele << " theta:" << theta_tele);
    response->success = true;
    }
    
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_tele,y_tele,theta_tele,reset_x,reset_y,reset_theta;
    size_t timer_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSim>());
    rclcpp::shutdown();
    return 0;
}