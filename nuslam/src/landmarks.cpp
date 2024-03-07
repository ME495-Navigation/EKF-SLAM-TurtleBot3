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

// constants
/// \brief The minimum distance between two points to be considered part of the same cluster
constexpr double DISTANCE_THRESH = 0.25;
/// \brief The minimum number of points in a cluster to be considered a landmark
constexpr int MIN_CLUSTER_SIZE = 3;


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

    /// \brief Detect clusters of points in the laser scan data
    /// \param msg The laser scan data
    /// \return A vector of detected clusters
    void detect_clusters(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      //create a vector of the clusters
      std::vector<std::vector<std::vector<double>>> clusters;
      // create a vector of the x and y coordinates
      std::vector<std::vector<double>> coordinates;

      // convert the laser scan range data to cartesian coordinates
      // iterate through the range data
      for (size_t i=0; i<msg->ranges.size(); i++)
      {
        // convert the range data to relative x and y coordinates
        double x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);
        double y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);

        // store the x and y coordinates in a vector
        std::vector<double> point = {x, y};
        coordinates.push_back(point);
      }

      // iterate through the coordinates
      // calculate distance between each point and every other point
      for (size_t i=0; i<coordinates.size(); i++)
      {
        // create a vector to store the cluster
        std::vector<std::vector<double>> cluster;

        for (size_t j=i; j<coordinates.size(); j++)
        {
          // calculate the distance between the two points
          double distance = sqrt(pow(coordinates[i][0] - coordinates[j][0], 2) + pow(coordinates[i][1] - coordinates[j][1], 2));

          // if the distance is less than the threshold, add the point to the cluster
          if (distance < DISTANCE_THRESH)
          {
            cluster.push_back(coordinates[j]);
          }
        }

        // check if the size of the cluster is greater than minimum cluster size
        if (cluster.size() > MIN_CLUSTER_SIZE)
        {
          // add the cluster to the vector of clusters
          clusters.push_back(cluster);
        } 
      }
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
