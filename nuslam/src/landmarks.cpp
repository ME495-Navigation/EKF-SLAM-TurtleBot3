/// \file 
/// \brief Detect landmarks in the laser scan data

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


/// @brief  Detect landmarks in the laser scan data
class landmarks : public rclcpp::Node
{
  public:
    landmarks()
    : Node("landmarks")
    {
      // create subscriber to laser scan data
      laser_scan_data_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "red/lidar", 10, std::bind(&landmarks::laser_scan_callback, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_data_;

    /// \brief Callback function for the laser scan data
    /// \param msg The laser scan data
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
    }

};

/// \brief The main fucntion.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}
