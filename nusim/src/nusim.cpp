/// \file
/// \brief Simulator for the turtlebot in an rvix scene.
///
/// PARAMETERS:
///     rate (double):The frequency of simulation frame update in Hz.
///     x0 (double): The initial x position of the turtlebot.
///     y0 (double): The initial y position of the turtlebot.
///     theta0 (double): The initial orientation of the turtlebot.
///     arena_x_length (double): The length of the arena.
///     arena_y_length (double): The width of the arena.
///     obstacles_x (std::vector<double>): The x coordinates of the obstacles in the scene in the form of a list.
///     obstacles_y (std::vector<double>): The y coordinates of the obstacles in the scene in the form of a list.
///     obstacles_r (double): The radius of the cylindrical obstacles.
///     wheel_radius (double): The radius of the wheels.
///     track_width (double): The separation between the wheels.
///     motor_cmd_max (double): The maximum value of the motor commands.
///     motor_cmd_per_rad_sec (double): The maximum value of the motor commands per radian per second.
///     encoder_ticks_per_rad (double): The number of encoder ticks per radian.
///     collision_radius (double): The collision radius of the robot.
///
/// PUBLISHERS:
///     ~/time_step (std_msgs/msg/UInt64): Publishes the current timestep.
///     ~/obstacles (visualization_msgs/msg/MarkerArray): Publishes the obstacles as markers to rviz.
///     ~/walls (visualization_msgs/msg/MarkerArray):  Publishes the walls of the arena as markers to rviz.
///     red/sensor_data (nuturtlebot_msgs/msg//SensorData): Publishes the sensor data of the turtlebot.
///
/// SUBSCRIBERS:
///    red/wheel_cmd (nuturtlebot_msgs/msg/WheelCmd): Subscribes to the wheel commands.
///
/// SERVICES:
///     ~/reset (std_srvs/srv/Empty): Resets the state of the simulation to the starting state.
///     ~/teleport (nusim/srv/Teleport): Teleports the turtlebot to the requested pose.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "std_srvs/srv/empty.hpp"
#include "nusim/srv/teleport.hpp"

using turtlelib::DiffDrive;
using turtlelib::Twist2D;
using turtlelib::WheelConfig;
using turtlelib::WheelVelocities;
using turtlelib::Transform2D;

using namespace std::chrono_literals;

/// \brief Turtlebot simulator.
class NuSim : public rclcpp::Node
{
public:
  /// \brief Constructor for the NuSim node class.
  NuSim()
  : Node("nusim"), timer_count_(0)
  {
    // Declare parameters
    auto timer_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timer_param_desc.description = "Timer frequency";
    declare_parameter("rate", 200.0, timer_param_desc);
    double timer_rate = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = std::chrono::milliseconds(int(1000.0 / timer_rate));
    sim_timestep = 1.0 / timer_rate;

    declare_parameter("x0", 0.0);
    x_tele = get_parameter("x0").as_double();
    reset_x = get_parameter("x0").as_double();

    declare_parameter("y0", 0.0);
    y_tele = get_parameter("y0").as_double();
    reset_y = get_parameter("y0").as_double();

    declare_parameter("theta0", 0.0);
    theta_tele = get_parameter("theta0").as_double();
    reset_theta = get_parameter("theta0").as_double();

    declare_parameter("arena_x_length", 10.0);
    arena_x = get_parameter("arena_x_length").as_double();

    declare_parameter("arena_y_length", 15.0);
    arena_y = get_parameter("arena_y_length").as_double();

    declare_parameter("obstacles.x", obstacles_x);
    obstacles_x = get_parameter("obstacles.x").as_double_array();

    declare_parameter("obstacles.y", obstacles_y);
    obstacles_y = get_parameter("obstacles.y").as_double_array();

    declare_parameter("obstacles.r", 0.1);
    obstacles_r = get_parameter("obstacles.r").as_double();

    declare_parameter("wheel_radius", -1.0);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if (wheel_radius < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter wheel_radius was not set");
    }
    declare_parameter("track_width", -1.0);
    track_width = get_parameter("track_width").as_double();
    if (track_width < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter track_width was not set");
    }
    declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_max < 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Parameter motor_cmd_max was not set");
    }
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter motor_cmd_per_rad_sec was not set");
    }
    declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter encoder_ticks_per_rad was not set");
    }
    declare_parameter("collision_radius", -1.0);
    collision_radius = get_parameter("collision_radius").as_double();
    if (collision_radius < 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Parameter collision radius was not set");
    }

    // Initialize diff_drive class
    robot_ = DiffDrive{track_width / 2.0, wheel_radius, {0.0, 0.0}, {{x_tele, y_tele}, theta_tele}};

    // Set QoS settings for the Marker topic
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local();

    // Create subscribers
    wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NuSim::wheel_cmd_callback, this, std::placeholders::_1));

    // Create publishers
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    arena_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos);
    obstacle_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    
    //create a path publisher
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    path_msg.header.frame_id = "nusim/world";

    // Create services
    reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&NuSim::timer_callback, this));
  }

private:
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arena_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Quaternion body_quaternion;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path_msg;
  double x_tele, y_tele, theta_tele, reset_x, reset_y, reset_theta;
  double arena_x, arena_y, wall_thickness = 0.5;
  double wheel_radius, track_width, motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  std::vector<double> obstacles_x{}, obstacles_y{};
  double obstacles_r, sim_timestep;
  WheelVelocities wheel_vels {0.0, 0.0};
  WheelConfig wheel_position {0.0, 0.0};
  DiffDrive robot_ {0.0, 0.0, {0.0, 0.0}, {{x_tele, y_tele}, theta_tele}};
  size_t timer_count_;

  /// \brief The timer callback
  void timer_callback()
  {
    // std::cout << "robot x: " << robot_.get_robot_config().translation().x << std::endl;
    // std::cout << "x tele: " << x_tele << std::endl;
    // publish the current timestep
    auto message = std_msgs::msg::UInt64();
    message.data = timer_count_;
    timestep_publisher_->publish(message);
    timer_count_++;
    wheel_position_update();
    update_robot_config(wheel_position);
    sensor_data_publisher();
    transform_publisher();
    path_publisher();
    // publish walls and obstacles
    walls_publisher();
    obstacles_publisher();
  }

  /// \brief Updated robot config frame publisher
  /// \param wheel The wheel config to be published
  void update_robot_config(const WheelConfig wheel)
  {
    const auto robot_configuration = robot_.forward_kinematics(wheel);
    // std::cout << "left wheel: " << robot_.get_wheel_config().lw << std::endl;
    // std::cout << "right wheel: " << robot_.get_wheel_config().rw << std::endl;
    // std::cout << "robot rot: " << robot_.get_robot_config().rotation() << std::endl;
    x_tele = robot_configuration.translation().x;
    y_tele = robot_configuration.translation().y;
    theta_tele = robot_configuration.rotation();
  }

  /// \brief Sensor data publisher
  void sensor_data_publisher()
  {
    auto sen_msg = nuturtlebot_msgs::msg::SensorData();
    sen_msg.stamp = rclcpp::Clock().now();
    sen_msg.left_encoder = (robot_.get_wheel_config().lw) * encoder_ticks_per_rad;
    sen_msg.right_encoder = (robot_.get_wheel_config().rw) * encoder_ticks_per_rad;
    sensor_data_publisher_->publish(sen_msg);
  }

  /// \brief Wheel position update
  void wheel_position_update()
  {
    // update the wheel configurations at each timestep
    wheel_position.lw += wheel_vels.lw * sim_timestep;
    wheel_position.rw += wheel_vels.rw * sim_timestep;
    // std::cout << "left wheel: " << wheel_vels.lw * sim_timestep << std::endl;
  }

  /// \brief The wheel command callback - sets wheel velocities
  /// \param msg The wheel command message
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // update the wheel velocities
    wheel_vels.lw = static_cast<double>(msg->left_velocity) * motor_cmd_per_rad_sec;
    wheel_vels.rw = static_cast<double>(msg->right_velocity) * motor_cmd_per_rad_sec;
    // std::cout << "left wheel: " << wheel_position.lw << std::endl;
  }

  /// \brief Callback for the reset service.
  /// Resets the simulation to the starting state.
  void reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    timer_count_ = 0;
    x_tele = reset_x;
    y_tele = reset_y;
    theta_tele = reset_theta;
    // reset the robot configuration
    Transform2D robot_pose {{x_tele, y_tele}, theta_tele};
    robot_.set_robot_config(robot_pose);
  }

  /// \brief Broadcasts the transform betweem world and
  /// the turtlebot base footprint.
  void transform_publisher()
  {
    // Create a quaternion to hold the rotation of the turtlebot
    body_quaternion.setRPY(0, 0, robot_.get_robot_config().rotation());

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";

    t.transform.translation.x = robot_.get_robot_config().translation().x;
    t.transform.translation.y = robot_.get_robot_config().translation().y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = body_quaternion.x();
    t.transform.rotation.y = body_quaternion.y();
    t.transform.rotation.z = body_quaternion.z();
    t.transform.rotation.w = body_quaternion.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  void path_publisher()
  {
    path_msg.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.stamp = this->get_clock()->now();
    pose_stamp.header.frame_id = "nusim/world";
    pose_stamp.pose.position.x = robot_.get_robot_config().translation().x;
    pose_stamp.pose.position.y = robot_.get_robot_config().translation().y;
    pose_stamp.pose.position.z = 0.0;

    // Create a quaternion to hold the rotation of the turtlebot
    body_quaternion.setRPY(0, 0, robot_.get_robot_config().rotation());
    pose_stamp.pose.orientation.x = body_quaternion.x();
    pose_stamp.pose.orientation.y = body_quaternion.y();
    pose_stamp.pose.orientation.z = body_quaternion.z();
    pose_stamp.pose.orientation.w = body_quaternion.w();

    path_msg.poses.push_back(pose_stamp);
    path_publisher_->publish(path_msg);
  }

  ///\brief Callback for the teleport service.
  /// Teleports the turtlebot to the requested pose.
  /// \param request - the requested pose
  /// \param response - the boolean success value
  void teleport_callback(
    nusim::srv::Teleport::Request::SharedPtr request,
    nusim::srv::Teleport::Response::SharedPtr response)
  {
    x_tele = request->x0;
    y_tele = request->y0;
    theta_tele = request->theta0;
    RCLCPP_INFO_STREAM(
      get_logger(), "Teleporting to x:" << x_tele << " y:" << y_tele << " theta:" << theta_tele);
    response->success = true;
    Transform2D pose {{x_tele, y_tele}, theta_tele};
    robot_.set_robot_config(pose);
  }

  /// \brief Arena walls publisher.
  void walls_publisher()
  {
    visualization_msgs::msg::MarkerArray array;

    auto current_time = rclcpp::Clock().now();

    // loop through the walls
    for (int i = 0; i < 4; i++) {
      visualization_msgs::msg::Marker wall;
      wall.header.frame_id = "nusim/world";
      wall.header.stamp = current_time;
      wall.id = i;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.action = visualization_msgs::msg::Marker::ADD;

      // east and west facing walls
      if (i == 0 or i == 2) {
        if (i == 0) {
          wall.pose.position.x = arena_x / 2.0 + wall_thickness / 2.0;
        } else {
          wall.pose.position.x = -(arena_x / 2.0 + wall_thickness / 2.0);
        }
        wall.pose.position.y = 0.0;
        wall.scale.x = wall_thickness;
        wall.scale.y = arena_y;
        wall.scale.z = 0.25;
      }
      // north and south facing walls
      else {
        wall.pose.position.x = 0.0;
        if (i == 1) {
          wall.pose.position.y = -(arena_y / 2.0 + wall_thickness / 2.0);
        } else {
          wall.pose.position.y = arena_y / 2.0 + wall_thickness / 2.0;
        }
        wall.scale.x = arena_x + 2 * wall_thickness;
        wall.scale.y = wall_thickness;
        wall.scale.z = 0.25;
      }
      wall.pose.orientation.x = 0.0;
      wall.pose.orientation.y = 0.0;
      wall.pose.orientation.z = 0.0;
      wall.pose.orientation.w = 1.0;
      wall.color.r = 1.0;
      wall.color.g = 0.0;
      wall.color.b = 0.0;
      wall.color.a = 1.0;
      array.markers.push_back(wall);
    }
    arena_publisher_->publish(array);
  }

  /// \brief Arena obstacles publisher.
  void obstacles_publisher()
  {
    if (obstacles_x.size() != obstacles_y.size()) {
      throw std::runtime_error("x and y coordinate lists are not the same size.");
    }
    visualization_msgs::msg::MarkerArray ob_array;

    // loop through all the obstacles in the list
    for (size_t i = 0; i < obstacles_x.size(); i++) {
      visualization_msgs::msg::Marker ob;
      ob.header.frame_id = "nusim/world";
      ob.header.stamp = get_clock()->now();
      ob.id = i;
      ob.type = visualization_msgs::msg::Marker::CYLINDER;
      ob.action = visualization_msgs::msg::Marker::ADD;
      ob.pose.position.x = obstacles_x.at(i);
      ob.pose.position.y = obstacles_y.at(i);
      ob.pose.position.z = 0.25 / 2.0;
      ob.pose.orientation.x = 0.0;
      ob.pose.orientation.y = 0.0;
      ob.pose.orientation.z = 0.0;
      ob.pose.orientation.w = 1.0;
      ob.scale.x = obstacles_r * 2.0;
      ob.scale.y = obstacles_r * 2.0;
      ob.scale.z = 0.25;
      ob.color.r = 1.0;
      ob.color.g = 0.0;
      ob.color.b = 0.0;
      ob.color.a = 1.0;
      ob_array.markers.push_back(ob);
    }
    obstacle_publisher_->publish(ob_array);
  }
};

/// \brief The main fucntion.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
