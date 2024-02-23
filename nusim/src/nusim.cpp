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
///     input_noise (double): The noise to be added to the wheel velocities.
///     slip_fraction (double): The fraction of slip in the wheels.
///     basic_sensor_variance (double): The basic sensor variance.
///     max_range (double): The maximum range of the fake sensor.
///
/// PUBLISHERS:
///     ~/time_step (std_msgs/msg/UInt64): Publishes the current timestep.
///     ~/obstacles (visualization_msgs/msg/MarkerArray): Publishes the obstacles as markers to rviz.
///     ~/walls (visualization_msgs/msg/MarkerArray):  Publishes the walls of the arena as markers to rviz.
///     red/sensor_data (nuturtlebot_msgs/msg//SensorData): Publishes the sensor data of the turtlebot.
///     red/path (nav_msgs/msg/Path): Publishes the path of the turtlebot.
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
#include <random>

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
#include "sensor_msgs/msg/laser_scan.hpp"

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


/// \brief A random number generator 
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

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
    declare_parameter("input_noise", 0.0);
    input_noise = get_parameter("input_noise").as_double();
    declare_parameter("slip_fraction", 0.0);
    slip_fraction = get_parameter("slip_fraction").as_double();
    declare_parameter("basic_sensor_variance", 0.0);
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    declare_parameter("lidar_noise", 0.0);
    lidar_noise = get_parameter("lidar_noise").as_double();

    declare_parameter("max_range", 0.0);
    max_range = get_parameter("max_range").as_double();


    declare_parameter("lidar_range_max", 3.5);
    lidar_range_max = get_parameter("lidar_range_max").as_double();
    declare_parameter("lidar_range_min", 0.12);
    lidar_range_min = get_parameter("lidar_range_min").as_double();
    declare_parameter("lidar_angle_increment", 0.01745329238474369);
    lidar_angle_increment = get_parameter("lidar_angle_increment").as_double();
    declare_parameter("lidar_angle_min", 0.0);
    lidar_angle_min = get_parameter("lidar_angle_min").as_double();
    declare_parameter("lidar_angle_max", 6.2657318115234375);
    lidar_angle_max = get_parameter("lidar_angle_max").as_double();
    declare_parameter("lidar_resolution", 0.0);
    lidar_resolution = get_parameter("lidar_resolution").as_double();

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
    fake_sensor_obs_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", qos);
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
    lidar_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "red/lidar",
      10);
    
    //create a path publisher
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    path_msg.header.frame_id = "nusim/world";

    // pb distribution functions
    wheel_vel_db = std::normal_distribution<>(0.0, input_noise);
    wheel_pos_db = std::uniform_real_distribution<>(-slip_fraction, slip_fraction);
    fake_obs_db = std::normal_distribution<>(0.0, basic_sensor_variance);
    lidar_db = std::normal_distribution<>(0.0, lidar_noise);

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

    // create marker array publishing timer
    five_hz_timer_ = create_wall_timer(
      200ms, std::bind(&NuSim::five_hz_timer_callback, this));

    // Create timer
    timer_ = create_wall_timer(
      rate, std::bind(&NuSim::timer_callback, this));
  }

private:
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arena_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_obs_publisher_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr five_hz_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2::Quaternion body_quaternion;
  nav_msgs::msg::Path path_msg;
  double x_tele, y_tele, theta_tele, reset_x, reset_y, reset_theta;
  double arena_x, arena_y, wall_thickness = 0.5;
  double wheel_radius, track_width, motor_cmd_max;
  double motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
  double input_noise, slip_fraction, basic_sensor_variance, max_range;
  double lidar_noise, lidar_range_max, lidar_range_min, lidar_angle_increment, lidar_angle_min, lidar_angle_max, lidar_resolution;
  std::vector<double> obstacles_x{}, obstacles_y{};
  double obstacles_r, sim_timestep;
  WheelVelocities wheel_vels {0.0, 0.0};
  WheelConfig wheel_position_actual {0.0, 0.0};
  WheelConfig wheel_position_sim {0.0, 0.0};
  DiffDrive robot_ {0.0, 0.0, {0.0, 0.0}, {{x_tele, y_tele}, theta_tele}};
  size_t timer_count_;
  std::normal_distribution<double> wheel_vel_db;
  std::normal_distribution<double> fake_obs_db;
  std::normal_distribution<double> lidar_db;
  std::uniform_real_distribution<double> wheel_pos_db;
  int col_detect_index;
  Transform2D base_lidar_transform {{-0.032, 0.0}, 0.0};

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
    wheel_position_actual_update();
    wheel_position_sim_update();
    update_robot_config(wheel_position_actual);
    // detect collision
    col_detect_index = detect_collision();
    if (col_detect_index != -1) {
      update_robot_config_post_collision(col_detect_index);
    }
    sensor_data_publisher();
    transform_publisher();
    path_publisher();
    // publish walls and obstacles
    walls_publisher();
    obstacles_publisher();
  }

  /// \brief The marker timer callback
  void five_hz_timer_callback()
  {
    fake_sensor_marker_publisher();
    lidar_scan_publisher();
  }

  /// \brief Updated robot config frame publisher
  /// \param wheel The wheel config to be published
  void update_robot_config(const WheelConfig wheel)
  {
    const auto robot_configuration = robot_.forward_kinematics(wheel);
    x_tele = robot_configuration.translation().x;
    y_tele = robot_configuration.translation().y;
    theta_tele = robot_configuration.rotation();
  }

  /// \brief Detect collision with the obstacles
  int detect_collision()
  {
    for (size_t i = 0; i < obstacles_x.size(); i++) {
      if (distance(x_tele, y_tele, obstacles_x.at(i), obstacles_y.at(i)) < collision_radius + obstacles_r) {
        // return the index of the obstacle that the robot has collided with
        return static_cast<int>(i);
      }
    }
    return -1;    
  }

  /// \brief Update the robot configuration post collision
  void update_robot_config_post_collision(int i)
  {
    // update the robot configuration post collision
    double m, del_x, del_y, dist;
    m = (y_tele - obstacles_y.at(i)) / (x_tele - obstacles_x.at(i));
    // b = y_tele - m * x_tele;

    dist = collision_radius + obstacles_r;

    // calculate distance to move the robot along the line
    // input negative distance depending on the direction along the line
    if (x_tele > obstacles_x.at(i)) {
      del_x = dist / std::sqrt(1 + std::pow(m, 2));
      del_y = m * del_x;
    } else {
      del_x = -dist / std::sqrt(1 + std::pow(m, 2));
      del_y = m * del_x;
    }

    // update the robot configuration
    Transform2D robot_pose {{obstacles_x.at(i) + del_x, obstacles_y.at(i) + del_y}, theta_tele};
    robot_.set_robot_config(robot_pose);
    x_tele = robot_pose.translation().x;
    y_tele = robot_pose.translation().y;
  }  

  /// \brief Sensor data publisher
  void sensor_data_publisher()
  {
    auto sen_msg = nuturtlebot_msgs::msg::SensorData();
    sen_msg.stamp = rclcpp::Clock().now();
    sen_msg.left_encoder = wheel_position_sim.lw * encoder_ticks_per_rad;
    sen_msg.right_encoder = wheel_position_sim.rw * encoder_ticks_per_rad;
    sensor_data_publisher_->publish(sen_msg);
  }

  /// \brief Actual wheel position update
  void wheel_position_actual_update()
  {
    // update the wheel configurations at each timestep with noise
    wheel_position_actual.lw += wheel_vels.lw * (1 + wheel_pos_db(get_random())) * sim_timestep;
    wheel_position_actual.rw += wheel_vels.rw * (1 + wheel_pos_db(get_random())) * sim_timestep;
  }

  /// \brief Sim wheel position update
  void wheel_position_sim_update()
  {
    // update the wheel configurations at each timestep
    wheel_position_sim.lw += wheel_vels.lw * sim_timestep;
    wheel_position_sim.rw += wheel_vels.rw * sim_timestep;
  }

  /// \brief The wheel command callback - sets wheel velocities
  /// \param msg The wheel command message
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // update the wheel velocities
    wheel_vels.lw = static_cast<double>(msg->left_velocity) * motor_cmd_per_rad_sec;
    wheel_vels.rw = static_cast<double>(msg->right_velocity) * motor_cmd_per_rad_sec;
    add_wheel_vel_noise();
  }

  /// \brief Adds noise to the wheel velocities
  void add_wheel_vel_noise(){
    // define gaussian noise with variance of input_noise
    if (wheel_vels.lw != 0.0 or wheel_vels.rw != 0.0) {
      if (input_noise > 0.0) {
        // RCLCPP_INFO_STREAM(get_logger(), "Adding noise to the wheel velocities");
        wheel_vels.lw += wheel_vel_db(get_random());
        wheel_vels.rw += wheel_vel_db(get_random());
      }
    }     
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

    t.header.stamp = rclcpp::Clock().now();
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

  /// \brief Publishes the path of the turtlebot
  void path_publisher()
  {
    path_msg.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.stamp = rclcpp::Clock().now();
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
      ob.header.stamp = rclcpp::Clock().now();
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

  /// \brief Fake sensor marker publisher.
  void fake_sensor_marker_publisher()
  {
    if (obstacles_x.size() != obstacles_y.size()) {
      throw std::runtime_error("x and y coordinate lists are not the same size.");
    }
    visualization_msgs::msg::MarkerArray fake_ob_array;

    // loop through all the obstacles in the list
    for (size_t i = 0; i < obstacles_x.size(); i++) {
      visualization_msgs::msg::Marker fake_ob;
      fake_ob.header.frame_id = "nusim/world";
      fake_ob.header.stamp = rclcpp::Clock().now();
      fake_ob.id = i;
      fake_ob.type = visualization_msgs::msg::Marker::CYLINDER;
      fake_ob.pose.position.x = obstacles_x.at(i) + fake_obs_db(get_random());
      fake_ob.pose.position.y = obstacles_y.at(i) + fake_obs_db(get_random());
      fake_ob.pose.position.z = 0.25 / 2.0;
      fake_ob.pose.orientation.x = 0.0;
      fake_ob.pose.orientation.y = 0.0;
      fake_ob.pose.orientation.z = 0.0;
      fake_ob.pose.orientation.w = 1.0;
      fake_ob.scale.x = obstacles_r * 2.0;
      fake_ob.scale.y = obstacles_r * 2.0;
      fake_ob.scale.z = 0.25;
      // set color yellow
      fake_ob.color.r = 1.0;
      fake_ob.color.g = 1.0;
      fake_ob.color.b = 0.0;
      fake_ob.color.a = 0.5;
      fake_ob.action = visualization_msgs::msg::Marker::ADD;
      
      if (distance(x_tele, y_tele, fake_ob.pose.position.x, fake_ob.pose.position.y) > max_range) {
        fake_ob.action = visualization_msgs::msg::Marker::DELETE;
      } 
      else {
        fake_ob.action = visualization_msgs::msg::Marker::ADD;
      }

      fake_ob_array.markers.push_back(fake_ob);
    }
    fake_sensor_obs_publisher_->publish(fake_ob_array);
  }

  /// \brief Lidar scan publisher.
  void lidar_scan_publisher()
  {
    sensor_msgs::msg::LaserScan lidar_scan;
    lidar_scan.header.frame_id = "red/base_scan";
    lidar_scan.header.stamp = rclcpp::Clock().now();
    lidar_scan.angle_min = lidar_angle_min;
    lidar_scan.angle_max = lidar_angle_max;
    lidar_scan.angle_increment = lidar_angle_increment;
    lidar_scan.range_min = lidar_range_min;
    lidar_scan.range_max = lidar_range_max;

    // loop through each lidar laser ray
    for (double i = lidar_scan.angle_min; i < lidar_scan.angle_max; i += lidar_scan.angle_increment) {
      double x_start = x_tele + base_lidar_transform.translation().x;
      double y_start = y_tele + base_lidar_transform.translation().y;
      double theta = theta_tele + base_lidar_transform.rotation();
      double x_end = x_start + lidar_range_max * std::cos(theta + i);
      double y_end = y_start + lidar_range_max * std::sin(theta + i);
      std::vector<double> intersection_points = lidar_obstacle_intersection(x_start, y_start, x_end, y_end);
      if (intersection_points.size() > 0) {
        double range = distance(x_start, y_start, intersection_points.at(0), intersection_points.at(1));
        range += lidar_db(get_random());
        lidar_scan.ranges.push_back(range);   
      }
      else {
        lidar_scan.ranges.push_back(0.0);
      }
    }
    lidar_publisher_->publish(lidar_scan);
  }

  /// \brief Check for intersection of lidar scan and cylindrical obstacles
  /// \param x_start The x coordinate of the start of the lidar scan
  /// \param y_start The y coordinate of the start of the lidar scan
  /// \param x_end The x coordinate of the end of the lidar scan
  /// \param y_end The y coordinate of the end of the lidar scan
  std::vector<double> lidar_obstacle_intersection(double x_start, double y_start, double x_end, double y_end)
  {
    std::vector<double> intersection_points;
    double m, b, cx, cy;
    for (size_t i = 0; i < obstacles_x.size(); i++) {
      // calculate the line equation
      m = (y_end - y_start) / (x_end - x_start);
      b = y_start - m * x_start;
      // calculate the center of the obstacle
      cx = obstacles_x.at(i);
      cy = obstacles_y.at(i);
      // calculate the perpendicular distance from the center of the obstacle to the line
      double p_dist = std::abs(m * cx - cy + b) / std::sqrt(std::pow(m, 2) + 1);
      // check if the perpendicular distance is less than the radius of the obstacle
      if (p_dist <= obstacles_r) {
        // calculate the coordinates of the perpendicular line intersection
        double x_int = (cx + m * cy - m * b) / (1 + std::pow(m, 2));
        double y_int = m * x_int + b;
        // check if the intersection point is within the line segment
        if (x_int < std::max(x_start, x_end) and x_int > std::min(x_start, x_end) and
          y_int < std::max(y_start, y_end) and y_int > std::min(y_start, y_end)) {  
          // calculate the distance from the perpendicular line intersection to the intersection points
          double d = std::sqrt(std::pow(obstacles_r, 2) - std::pow(p_dist, 2));
          // calculate the intersection points
          double x1 = x_int + d * std::cos(std::atan(m));
          double y1 = y_int + d * std::sin(std::atan(m));
          double x2 = x_int - d * std::cos(std::atan(m));
          double y2 = y_int - d * std::sin(std::atan(m));
          // check which intersection point is closer to the start of the line
          if (distance(x_start, y_start, x1, y1) < distance(x_start, y_start, x2, y2)) {
            intersection_points.push_back(x1);
            intersection_points.push_back(y1);
          } else {
            intersection_points.push_back(x2);
            intersection_points.push_back(y2);
          }
        }
      }
    }
    if (intersection_points.size() > 2) {
      // calculate the closest intersection point to the start of the line
      for (size_t i = 0; i < intersection_points.size(); i += 2) {
        if (distance(x_start, y_start, intersection_points.at(i), intersection_points.at(i + 1)) < 
        distance(x_start, y_start, intersection_points.at(0), intersection_points.at(1))) {
          intersection_points.at(0) = intersection_points.at(i);
          intersection_points.at(1) = intersection_points.at(i + 1);
        }
      }
    }
    std::vector<double> closest_intersection;
    if (intersection_points.size() > 0) {
      closest_intersection.push_back(intersection_points.at(0));
      closest_intersection.push_back(intersection_points.at(1));
    }
    return closest_intersection;
  }


  /// \brief Calculate the distance between two points
  /// \param x1 The x coordinate of the first point
  /// \param y1 The y coordinate of the first point
  /// \param x2 The x coordinate of the second point
  /// \param y2 The y coordinate of the second point
  /// \return The distance between the two points
  double distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
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
