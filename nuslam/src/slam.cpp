/// \file
/// \brief Slam/Odometry node for the turtlebot.
///
/// PARAMETERS:
///     rate (double):The frequency of the odometry node timer.
///     x0 (double): The initial x position of the turtlebot.
///     y0 (double): The initial y position of the turtlebot.
///     theta0 (double): The initial orientation of the turtlebot.
///     body_id (string): The id of the body.
///     odom_id (string): The id of the odom frame.
///     wheel_left (string): The name of the left wheel.
///     wheel_right (string): The name of the right wheel.
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The separation between the wheels.
///
/// PUBLISHERS:
///     odom (nav_msgs/msg/Odometry): The turtlebot odometry message.
///
/// SUBSCRIBERS:
///    joint_states (sensor_msgs/msg/JointState): The joint states of the turtlebot.
///
/// SERVICES:
///     initial_pose (nuslam/srv/InitialPose): The initial pose of the turtle.

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "nuslam/srv/initial_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using turtlelib::DiffDrive;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::WheelConfig;
using turtlelib::WheelVelocities;
using turtlelib::almost_equal;
using namespace std::chrono_literals;

// Constants
constexpr int MAX_OBSTACLES = 30; // Maximum number of obstacles
constexpr int STATE_SIZE = MAX_OBSTACLES * 2 + 3; // State size for the EKF

/// \brief Slam node for the turtlebot.
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("rate", 200.0, timer_param_desc);
    double timer_rate = get_parameter("rate").as_double();
    std::chrono::milliseconds rate =
      std::chrono::milliseconds(int(1000.0 / timer_rate));

    declare_parameter("x0", 0.0);
    x_tele = get_parameter("x0").as_double();

    declare_parameter("y0", 0.0);
    y_tele = get_parameter("y0").as_double();

    declare_parameter("theta0", 0.0);
    theta_tele = get_parameter("theta0").as_double();

    declare_parameter("body_id", "");
    body_id = get_parameter("body_id").as_string();
    if (body_id.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter body_id was not set");
      rclcpp::shutdown();
    }

    declare_parameter("odom_id", "odom");
    odom_id = get_parameter("odom_id").as_string();

    declare_parameter("wheel_left", "");
    wheel_left = get_parameter("wheel_left").as_string();
    if (wheel_left.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter wheel_left was not set");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_right", "");
    wheel_right = get_parameter("wheel_right").as_string();
    if (wheel_right.empty()) {
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
      "joint_states", 10,
      std::bind(
        &Slam::joint_state_callback, this,
        std::placeholders::_1));
    
    // create subcriber to the fake sensor topic
    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10,
      std::bind(
        &Slam::fake_sensor_callback, this,
        std::placeholders::_1));

    // Create publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    //create a path publisher
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("blue/path", 10);
    path_msg.header.frame_id = "odom";

    // Create services
    initial_pose_ = create_service<nuslam::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Slam::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Initialize the transform broadcaster
    odom_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    map_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the odometry message
    odom_msg_.header.frame_id = odom_id;
    odom_msg_.child_frame_id = body_id;

    // Initialize diff_drive class
    nuturtle_ =
      DiffDrive{track_width / 2.0, wheel_radius, {0.0, 0.0}, {{x_tele, y_tele}, theta_tele}};

    // Create timer
    timer_ =
      create_wall_timer(rate, std::bind(&Slam::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Service<nuslam::srv::InitialPose>::SharedPtr initial_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_tf_;
  tf2::Quaternion body_quaternion;
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Path path_msg;
  std::string body_id, odom_id, wheel_left, wheel_right;
  double wheel_radius, track_width;
  double x_tele, y_tele, theta_tele;
  size_t timer_count_;
  DiffDrive nuturtle_{0.0, 0.0};
  arma::vec state {STATE_SIZE, arma::fill::zeros}; // slam state
  arma::mat covar {STATE_SIZE, STATE_SIZE, arma::fill::eye}; // covariance
  WheelConfig prev_wheel_config {}; // previous wheel configuration

  /// \brief The timer callback
  void timer_callback() {
    timer_count_++;
    path_publisher();
  }

  /// \brief Update the robot configuration and publish the odometry
  /// message/transform
  /// \param msg The joint states message
  void joint_state_callback(const sensor_msgs::msg::JointState & msg)
  {

    // use fk to update the robot's configuration
    const auto updated_config = nuturtle_.forward_kinematics(
      WheelConfig{msg.position.at(0), msg.position.at(1)});

    // Create a quaternion to hold the rotation of the turtlebot
    body_quaternion.setRPY(0, 0, updated_config.rotation());

    const auto robot_twist = nuturtle_.robot_body_twist(
      WheelConfig{msg.position.at(0), msg.position.at(1)});
    odom_msg_.twist.twist.linear.x = robot_twist.x;
    odom_msg_.twist.twist.linear.y = robot_twist.y;
    odom_msg_.twist.twist.angular.z = robot_twist.omega;

    // update the odometry message
    odom_msg_.header.stamp = msg.header.stamp;
    odom_msg_.pose.pose.position.x = updated_config.translation().x;
    odom_msg_.pose.pose.position.y = updated_config.translation().y;
    odom_msg_.pose.pose.orientation.x = body_quaternion.x();
    odom_msg_.pose.pose.orientation.y = body_quaternion.y();
    odom_msg_.pose.pose.orientation.z = body_quaternion.z();
    odom_msg_.pose.pose.orientation.w = body_quaternion.w();

    // publish the odometry message
    odom_pub_->publish(odom_msg_);

    // publish the robot's transform
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header = odom_msg_.header;
    t.child_frame_id = odom_msg_.child_frame_id;

    t.transform.translation.x = odom_msg_.pose.pose.position.x;
    t.transform.translation.y = odom_msg_.pose.pose.position.y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = body_quaternion.x();
    t.transform.rotation.y = body_quaternion.y();
    t.transform.rotation.z = body_quaternion.z();
    t.transform.rotation.w = body_quaternion.w();

    // Send the transformation
    odom_tf_->sendTransform(t);
  }

  /// \brief Callback for the fake sensor
  /// EKF SLAM logic is implemented here
  /// \param msg The fake sensor message
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // Get the robot's wheel configuration
    const auto wheel_config = nuturtle_.get_wheel_config();

    // update the state
    // state(0) = nuturtle_.get_robot_config().rotation();
    // state(1) = nuturtle_.get_robot_config().translation().x;
    // state(2) = nuturtle_.get_robot_config().translation().y;

    // Get the robot's twist
    const auto robot_twist = nuturtle_.wheel_twist(wheel_config, prev_wheel_config);
    // update the previous wheel configuration
    prev_wheel_config = wheel_config;

    // EKF prediction
    EKF_Slam_predict(state, covar, robot_twist);

     // Create a transform to hold the robot's configuration
    Transform2D map_tf {{state(1), state(2)}, state(0)};

    // broadcast the robot's map transform
    geometry_msgs::msg::TransformStamped map_t;
    map_t.header.stamp = this->get_clock()->now();
    map_t.header.frame_id = "map";
    map_t.child_frame_id = "green/base_footprint";
    map_t.transform.translation.x = map_tf.translation().x;
    map_t.transform.translation.y = map_tf.translation().y;
    map_t.transform.translation.z = 0.0;

    // Create a quaternion to hold the rotation of the turtlebot
    body_quaternion.setRPY(0, 0, map_tf.rotation());
    map_t.transform.rotation.x = body_quaternion.x();
    map_t.transform.rotation.y = body_quaternion.y();
    map_t.transform.rotation.z = body_quaternion.z();
    map_t.transform.rotation.w = body_quaternion.w();

    // Send the transformation
    map_tf_->sendTransform(map_t);

  }

  /// \brief EKF SLAM prediction step
  /// updates the state and covariance
  /// \param state The state vector
  /// \param covar The covariance matrix
  /// \param twist The robot's twist
  void EKF_Slam_predict(arma::vec & state, arma::mat & covar, const Twist2D & twist)
  {
    // Create identity matrix of state_size
    arma::mat I = arma::eye<arma::mat>(STATE_SIZE, STATE_SIZE);

    // Create the state transition model
    // Update the estimate using the model (odometry)
    // check if the angular component of the twist is zero
    if (almost_equal(twist.omega, 0.0)) {
      // if the angular component is zero
      state(1) += twist.x * std::cos(state(0));
      state(2) += twist.x * std::sin(state(0));
    } else {
      // if the angular component is non-zero
      state(1) += (twist.x / twist.omega) * (std::sin(state(0) + twist.omega) - std::sin(state(0)));
      state(2) += (twist.x / twist.omega) * (-std::cos(state(0) + twist.omega) + std::cos(state(0)));
      state(0) += twist.omega;
    }

    // Update the covariance
    // Initialize the A_t matrix
    arma::mat A_t (STATE_SIZE, STATE_SIZE, arma::fill::zeros);
    // check if angular component of twist is zero
    if (almost_equal(twist.omega, 0.0)){
      // if the angular component is zero
      A_t(1,0) = -twist.x * std::sin(state(0));
      A_t(2,0) = twist.x * std::cos(state(0));
      A_t = I + A_t;
    }
    else{
      // if the angular component is non-zero
      A_t(1,0) = (twist.x / twist.omega) * (std::cos(state(0) + twist.omega) - std::cos(state(0)));
      A_t(2,0) = (twist.x / twist.omega) * (std::sin(state(0) + twist.omega) - std::sin(state(0)));
      A_t = I + A_t;
    }

    covar = A_t * covar * A_t.t();

  }
  

  /// \brief Publishes the path of the turtlebot
  void path_publisher()
  {
    path_msg.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.stamp = rclcpp::Clock().now();
    pose_stamp.header.frame_id = "odom";
    pose_stamp.pose.position.x = nuturtle_.get_robot_config().translation().x;
    pose_stamp.pose.position.y = nuturtle_.get_robot_config().translation().y;
    pose_stamp.pose.position.z = 0.0;

    // Create a quaternion to hold the rotation of the turtlebot
    body_quaternion.setRPY(0, 0, nuturtle_.get_robot_config().rotation());
    pose_stamp.pose.orientation.x = body_quaternion.x();
    pose_stamp.pose.orientation.y = body_quaternion.y();
    pose_stamp.pose.orientation.z = body_quaternion.z();
    pose_stamp.pose.orientation.w = body_quaternion.w();

    path_msg.poses.push_back(pose_stamp);
    path_publisher_->publish(path_msg);
  }

  /// \brief Callback for the initial pose service
  /// \param req The initial pose request
  /// \param res The successful response boolean
  void initial_pose_callback(
    nuslam::srv::InitialPose::Request::SharedPtr req,
    nuslam::srv::InitialPose::Response::SharedPtr res)
  {
    // update the robot's configuration to the specified initial pose
    nuturtle_.set_robot_config(Transform2D{{req->x, req->y}, req->theta});
    res->success = true;
  }
};

/// \brief The main fucntion.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
