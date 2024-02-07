/// \file
/// \brief Odometry node for the turtlebot.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "turtlelib/diff_drive.hpp"
using turtlelib::DiffDrive;
using turtlelib::WheelVelocities;
using turtlelib::Twist2D;
using turtlelib::WheelConfig;


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

    declare_parameter("body_id", "");
    body_id = get_parameter("body_id").as_string();
    if (body_id.empty()){
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter body_id was not set");
      rclcpp::shutdown();
    }

    declare_parameter("odom_id", "odom");
    odom_id = get_parameter("odom_id").as_string();

    declare_parameter("wheel_left", "");
    wheel_left = get_parameter("wheel_left").as_string();
    if (wheel_left.empty()){
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter wheel_left was not set");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_right", "");
    wheel_right = get_parameter("wheel_right").as_string();
    if (wheel_right.empty()){
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter wheel_right was not set");
      rclcpp::shutdown();
    }

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

    // Create subscribers
    joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",10,std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Initialize the transform broadcaster
    odom_tf_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the odometry message
    odom_msg_.header.frame_id = odom_id;
    odom_msg_.child_frame_id = body_id;
    
    // Initialize diff_drive class
    nuturtle_ = DiffDrive{track_width/2.0, wheel_radius};

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&Odometry::timer_callback, this));
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_;
  nav_msgs::msg::Odometry odom_msg_;
  std::string body_id, odom_id, wheel_left, wheel_right;
  double wheel_radius, track_width;
  size_t timer_count_;
  DiffDrive nuturtle_{0.0,0.0};

  /// \brief The timer callback
  void timer_callback()
  {
    timer_count_++;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {
    // use fk to update the robot's configuration
    const auto updated_config = nuturtle_.forward_kinematics(WheelConfig {msg.position.at(0), msg.position.at(1)});

    // update the odometry message
    odom_msg_.header.stamp = msg.header.stamp;
    odom_msg_.pose.pose.position.x = updated_config.translation().x;
    odom_msg_.pose.pose.position.y = updated_config.translation().y;
    odom_msg_.pose.pose.orientation.z = updated_config.rotation();

    const auto robot_twist = nuturtle_.robot_body_twist(WheelConfig {msg.position.at(0), msg.position.at(1)});
    odom_msg_.twist.twist.linear.x = robot_twist.x;
    odom_msg_.twist.twist.linear.y = robot_twist.y;
    odom_msg_.twist.twist.angular.z = robot_twist.omega;

    // publish the odometry message
    odom_pub_->publish(odom_msg_);

    // publish the robot's transform
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header = odom_msg_.header;
    t.child_frame_id = odom_msg_.child_frame_id;

    t.transform.translation.x = odom_msg_.pose.pose.position.x;
    t.transform.translation.y = odom_msg_.pose.pose.position.y ;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
    t.transform.rotation.w = 1.0;

    // Send the transformation
    odom_tf_->sendTransform(t);
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