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
#include "visualization_msgs/msg/marker_array.hpp"

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

      // Set QoS settings for the Marker topic
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.transient_local();

      // create a publisher to visualize the clusters
      cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("clusters", qos);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_data_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;

    /// \brief Callback function for the laser scan data
    /// \param msg The laser scan data
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      // detect clusters in the laser scan data
      std::vector<std::vector<std::vector<double>>> clusters = detect_clusters(msg);

      // publish the clusters as markers
      publish_cluster_markers(clusters);
    }

    /// \brief Detect clusters of points in the laser scan data
    /// \param msg The laser scan data
    /// \return A vector of detected clusters
    std::vector<std::vector<std::vector<double>>> detect_clusters(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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

      // create a vector to store the cluster
      std::vector<std::vector<double>> cluster;
      // add the first point to the cluster
      cluster.push_back(coordinates[0]);

      // iterate through the coordinates
      for (size_t i=0; i<coordinates.size()-1; i++)
      {
        // calculate the distance to the next point
        double dist = distance(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]);

        // if the distance is less than the threshold, add the point to the cluster
        if (dist < DISTANCE_THRESH && dist > 0.0)
        {
          cluster.push_back(coordinates[i+1]);
        }
        else
        {
          // check if the size of the cluster is greater than minimum cluster size
          if (cluster.size() > MIN_CLUSTER_SIZE)
          {
            // add the cluster to the vector of clusters
            clusters.push_back(cluster);
          }
          // clear the cluster
          cluster.clear();
          // add the next point to the cluster
          cluster.push_back(coordinates[i+1]);
        }
      }

      // At this stage, the cluster variable contains either the last point or the last cluster
      // check for wrap around
      // calculate the distance between the last and first points
      double dist = distance(coordinates[coordinates.size()-1][0], coordinates[coordinates.size()-1][1], coordinates[0][0], coordinates[0][1]);
      // log the wrap around distance
      RCLCPP_INFO(this->get_logger(), "Wrap around distance: %f", dist);
      // log the coordinates of the last and first points
      RCLCPP_INFO(this->get_logger(), "Last point: %f, %f", coordinates[coordinates.size()-1][0], coordinates[coordinates.size()-1][1]);
      RCLCPP_INFO(this->get_logger(), "First point: %f, %f", coordinates[0][0], coordinates[0][1]);

      // if the distance is less than the threshold, add the first point to the cluster
      if (dist < DISTANCE_THRESH && dist > 0.0)
      {
        cluster.push_back(coordinates[0]);
      }

      // iterate through the coordinates till a break is found
      for (size_t i=0; i<coordinates.size()-1; i++)
      {
        // calculate the distance to the next point
        double dist = distance(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]);

        // if the distance is less than the threshold, add the point to the cluster
        if (dist < DISTANCE_THRESH && dist > 0.0)
        {
          cluster.push_back(coordinates[i+1]);
        }
        else
        {
          // check if the size of the cluster is greater than minimum cluster size
          if (cluster.size() > MIN_CLUSTER_SIZE)
          {
            // add the cluster to the vector of clusters
            clusters.push_back(cluster);
          }
          // clear the cluster
          cluster.clear();
          // break the loop
          break;    
        }
      }

      // check if there is overlap between the last and first clusters
      // check if the last points of both clusters are the same
      double x0 = clusters[clusters.size()-1][clusters[clusters.size()-1].size()-1][0];
      double y0 = clusters[clusters.size()-1][clusters[clusters.size()-1].size()-1][1];
      double x1 = clusters[0][0][0];
      double y1 = clusters[0][0][1];

      if (x0 == x1 && y0 == y1)
      {
        // log the removal
        RCLCPP_INFO(this->get_logger(), "Removing the first cluster");
        // remove the first cluster
        clusters.erase(clusters.begin());
      }
    
    // log the clusters size
    // RCLCPP_INFO(this->get_logger(), "Number of clusters: %ld", clusters.size());
    return clusters;
    }

    /// \brief Publish the clusters as markers
    /// \param clusters The clusters to be published
    void publish_cluster_markers(const std::vector<std::vector<std::vector<double>>> & clusters)
    {
      // create a marker array
      visualization_msgs::msg::MarkerArray marker_array;

      // iterate through the clusters
      for (size_t i=0; i<clusters.size(); i++)
      {
        // create a marker for each cluster
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "red/base_scan";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "landmarks";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.color.r = 1.0;
        marker.color.a = 1.0;

// ############################## Begin_Citation [10] ##############################
        // iterate through the points in the cluster
        for (size_t j=0; j<clusters[i].size(); j++)
        {
          // create a point for each point in the cluster
          geometry_msgs::msg::Point point;
          point.x = clusters[i][j][0];
          point.y = clusters[i][j][1];
          point.z = 0;
          marker.points.push_back(point);
        }
// ############################## End_Citation [10] ################################
        // add the marker to the marker array
        marker_array.markers.push_back(marker);
      }

      // publish the marker array
      cluster_pub_->publish(marker_array);
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
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}
