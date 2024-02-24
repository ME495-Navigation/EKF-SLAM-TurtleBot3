/// \file
/// \brief Control commands for the turtlebot.
///
/// PARAMETERS:
///     rate (double):The frequency of the turtle_control node timer.
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The separation between the wheels.
///     motor_cmd_max (double): The maximum value of the motor commands.
///     motor_cmd_per_rad_sec (double): The maximum value of the motor commands per radian per second.
///     encoder_ticks_per_rad (double): The number of encoder ticks per radian.
///     collision_radius (double): The collision radius of the robot.
///
/// PUBLISHERS:
///    wheel_cmd (nuturtlebot_msgs/msg/WheelCmd): Subscribes to the wheel commands.
///    joint_states (sensor_msgs/msg/JointState): The joint states of the turtlebot.
///
/// SUBSCRIBERS:
///     cmd_vel (geometry_msgs/msg//Twist): Publishes the commands of the turtelbot.
///     red/sensor_data (nuturtlebot_msgs/msg//SensorData): Publishes the sensor data of the turtlebot.

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
using turtlelib::DiffDrive;
using turtlelib::Twist2D;
using turtlelib::WheelConfig;
using turtlelib::WheelVelocities;

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
    std::chrono::milliseconds rate =
      std::chrono::milliseconds(int(1000.0 / timer_rate));

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
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter motor_cmd_per_rad_sec was not set");
      rclcpp::shutdown();
    }
    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter encoder_ticks_per_rad was not set");
      rclcpp::shutdown();
    }
    declare_parameter("collision_radius", -1.0);
    collision_radius = get_parameter("collision_radius").as_double();
    if (collision_radius < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter collision radius was not set");
      rclcpp::shutdown();
    }

    // Create subscribers
    velocity_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(
        &TurtleControl::velocity_callback, this,
        std::placeholders::_1));
    s_data_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10,
      std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    // Create publishers
    wheel_cmd_pub_ =
      create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Initialize diff_drive class
    nuturtle_ = DiffDrive{track_width / 2.0, wheel_radius};

    // initialize joint states for lw and rw
    joint_state.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state.position = {0.0, 0.0};
    joint_state.velocity = {0.0, 0.0};

    // Create timer
    timer_ = create_wall_timer(
      rate,
      std::bind(&TurtleControl::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr
    s_data_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr
    wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  sensor_msgs::msg::JointState joint_state, prev_joint_state;
  DiffDrive nuturtle_{0.0, 0.0};
  size_t timer_count_;
  double wheel_radius, track_width, motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  bool js_state = false;

  /// \brief The timer callback
  void timer_callback() {timer_count_++;}

  /// \brief The velocity callback
  ///        Convert cmd_vel msgs to wheel_cmd msgs and publish
  /// \param msg - cmd_vel message
  void velocity_callback(const geometry_msgs::msg::Twist & msg)
  {

    // create a Twist2D object from the Twist msg
    Twist2D body_twist{msg.angular.z, msg.linear.x, msg.linear.y};

    // using ik, calculate the wheel velocities
    WheelVelocities wheel_vel = nuturtle_.inverse_kinematics(body_twist);

    // create a wheel_cmd message
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    // calculate vel in mcu and convert to int
    wheel_cmd.left_velocity =
      static_cast<int>(wheel_vel.lw / motor_cmd_per_rad_sec);
    wheel_cmd.right_velocity =
      static_cast<int>(wheel_vel.rw / motor_cmd_per_rad_sec);
    // throttle vel to max and min values if request is outside range
    if (wheel_cmd.left_velocity < -motor_cmd_max) {
      wheel_cmd.left_velocity = -motor_cmd_max;
    }
    if (wheel_cmd.left_velocity > motor_cmd_max) {
      wheel_cmd.left_velocity = motor_cmd_max;
    }
    if (wheel_cmd.right_velocity < -motor_cmd_max) {
      wheel_cmd.right_velocity = -motor_cmd_max;
    }
    if (wheel_cmd.right_velocity > motor_cmd_max) {
      wheel_cmd.right_velocity = motor_cmd_max;
    }
    wheel_cmd_pub_->publish(wheel_cmd);
  }

  /// \brief The sensor data callback
  ///        Convert sensor_data msgs to joint_state msgs and publish
  /// \param msg - sensor_data message
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    // calculate wheel configuration in radians
    WheelConfig wheel_config;
    wheel_config.lw =
      static_cast<double>(msg.left_encoder) / encoder_ticks_per_rad;
    wheel_config.rw =
      static_cast<double>(msg.right_encoder) / encoder_ticks_per_rad;

    // build joint_state message
    joint_state.header.stamp = msg.stamp;
    joint_state.position.at(0) = wheel_config.lw;
    joint_state.position.at(1) = wheel_config.rw;

    if (js_state == true) {
      // calculate delta t
      double time_curr = static_cast<double>(joint_state.header.stamp.sec) +
        static_cast<double>(joint_state.header.stamp.nanosec) * 1e-9;
      double time_prev = static_cast<double>(prev_joint_state.header.stamp.sec) +
        static_cast<double>(prev_joint_state.header.stamp.nanosec) * 1e-9;
      double dt = time_curr - time_prev;

      // form updated joint state message with velocity
      joint_state.velocity[0] =
        (joint_state.position[0] - prev_joint_state.position[0]) / dt;
      joint_state.velocity[1] =
        (joint_state.position[1] - prev_joint_state.position[1]) / dt;
    } else {
      js_state = true;
    }
    joint_pub_->publish(joint_state);
    prev_joint_state = joint_state;
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
