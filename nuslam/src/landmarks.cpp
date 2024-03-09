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
      qos.best_effort();

      // create a publisher to visualize the clusters
      cluster_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("clusters", qos);

      // creare a publisher to visualize the landmarks
      landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", qos);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_data_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;

    /// \brief Callback function for the laser scan data
    /// \param msg The laser scan data
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      // detect clusters in the laser scan data
      std::vector<std::vector<std::vector<double>>> clusters = detect_clusters(msg);

      // publish the clusters as markers
      publish_cluster_markers(clusters);

      // fit circles to the clusters
      std::vector<std::vector<double>> circle_params = circle_fit(clusters);

      // publish the landmarks as markers
      publish_landmark_markers(circle_params);
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
      // add the first point to the cluster if it is not 0,0
      if (coordinates[0][0] != 0 || coordinates[0][1] != 0)
      {
        cluster.push_back(coordinates[0]);
      }

      // iterate through the coordinates
      for (size_t i=0; i<coordinates.size()-1; i++)
      {
        // calculate the distance to the next point
        double dist = distance(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]);

        // if the distance is less than the threshold, add the point to the cluster
        if (dist < DISTANCE_THRESH && dist > 0.0)
        {
          // add the point to the cluster if it is not 0,0
          if (coordinates[i+1][0] != 0 || coordinates[i+1][1] != 0)
          {
            cluster.push_back(coordinates[i+1]);
          }
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
          // add the next point to the cluster if it is not 0,0
          if (coordinates[i+1][0] != 0 || coordinates[i+1][1] != 0)
          {
            cluster.push_back(coordinates[i+1]);
          }
        }
      }

      // At this stage, the cluster variable contains either the last point or the last cluster
      // check for wrap around
      // calculate the distance between the last and first points
      double dist = distance(coordinates[coordinates.size()-1][0], coordinates[coordinates.size()-1][1], coordinates[0][0], coordinates[0][1]);

      // if the distance is less than the threshold, add the first point to the cluster
      if (dist < DISTANCE_THRESH && dist > 0.0)
      {
        // add the first point to the cluster if it is not 0,0
        if (coordinates[0][0] != 0 || coordinates[0][1] != 0)
        {
          cluster.push_back(coordinates[0]);
        }
      }

      // iterate through the coordinates till a break is found
      for (size_t i=0; i<coordinates.size()-1; i++)
      {
        // calculate the distance to the next point
        double dist = distance(coordinates[i][0], coordinates[i][1], coordinates[i+1][0], coordinates[i+1][1]);

        // if the distance is less than the threshold, add the point to the cluster
        if (dist < DISTANCE_THRESH && dist > 0.0)
        {
          // add the point to the cluster if it is not 0,0
          if (coordinates[i+1][0] != 0 || coordinates[i+1][1] != 0)
          {
            cluster.push_back(coordinates[i+1]);
          }
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

      // check if number of clusters is greater than 0
      if (clusters.size() > 1)
      {
        // check if there is overlap between the last and first clusters
        // check if the last points of both clusters are the same
        double x0 = clusters[clusters.size()-1][clusters[clusters.size()-1].size()-1][0];
        double y0 = clusters[clusters.size()-1][clusters[clusters.size()-1].size()-1][1];
        double x1 = clusters[0][clusters[0].size()-1][0];
        double y1 = clusters[0][clusters[0].size()-1][1];

        if (x0 == x1 && y0 == y1)
        {
          // log the removal
          RCLCPP_INFO(this->get_logger(), "Removing the first cluster");
          // remove the first cluster
          clusters.erase(clusters.begin());
        }
      }

    // check if any of the clusters has 0,0
    for (size_t i=0; i<clusters.size(); i++)
    {
      for (size_t j=0; j<clusters[i].size(); j++)
      {
        if (clusters[i][j][0] == 0 && clusters[i][j][1] == 0)
        {
          // log the zero point and the cluster id
          RCLCPP_INFO(this->get_logger(), "Zero point: Cluster id: %ld", i);
        }
      }
    }
    // log the clusters size
    RCLCPP_INFO(this->get_logger(), "Number of clusters: %ld", clusters.size());
    return clusters;
    }

    /// \brief Circle fitting algorithm
    /// \param clusters The clusters to be fitted
    /// \return The center and radius of the fitted circle
    std::vector<std::vector<double>> circle_fit(const std::vector<std::vector<std::vector<double>>> & clusters)
    {
      // create a vector to store the centers and radii of the circles
      std::vector<std::vector<double>> circle_params;

      // process each cluster separately
      for (size_t i=0; i < clusters.size(); i++)
      {
        // find the mean of the x coordinates
        double x_mean = 0;
        for (size_t j=0; j < clusters[i].size(); j++)
        {
          x_mean += clusters[i][j][0];
        }
        x_mean /= clusters[i].size();

        // find the mean of the y coordinates
        double y_mean = 0;
        for (size_t j=0; j < clusters[i].size(); j++)
        {
          y_mean += clusters[i][j][1];
        }
        y_mean /= clusters[i].size();

        // shift the coordinates so that centroid is at the origin
        std::vector<std::vector<double>> shifted_coordinates;
        for (size_t j=0; j < clusters[i].size(); j++)
        {
          std::vector<double> point = {clusters[i][j][0] - x_mean, clusters[i][j][1] - y_mean};
          shifted_coordinates.push_back(point);
        }

        // compute z_i
        std::vector<double> z_i;
        for (size_t j=0; j < shifted_coordinates.size(); j++)
        {
          z_i.push_back(std::pow(shifted_coordinates[j][0], 2) + std::pow(shifted_coordinates[j][1], 2));
        }

        // compute the mean of z_i
        double z_mean = 0;
        for (size_t j=0; j < z_i.size(); j++)
        {
          z_mean += z_i[j];
        }
        z_mean /= z_i.size();

        // form the data matrix Z
        arma::mat Z(shifted_coordinates.size(), 4, arma::fill::ones);
        // make the first column equal to z_i
        // make second and third columns equal to x_i and y_i
        for (size_t j=0; j < z_i.size(); j++)
        {
          Z(j, 0) = z_i[j];
          Z(j, 1) = shifted_coordinates[j][0];
          Z(j, 2) = shifted_coordinates[j][1];
        }

        // form the moment matrix M
        arma::mat M = Z.t() * Z;
        M /= clusters[i].size();

        // form the constraint matrix H
        arma::mat H = {{8*z_mean, 0, 0, 2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {2, 0, 0, 0}};

        // compute H inverse
        arma::mat H_inv = H.i();

        // compute the svd of Z
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U, s, V, Z);

        // initilaize the A vector
        arma::vec A(4, arma::fill::ones);

        // check if the value of the smallest singular value
        // is less than 10^-12
        if (s(s.size()-1) < 1e-12)
        {
          // set the A vector to the last column of V
          A = V.col(V.n_cols-1);
        }
        else
        {
          // compute the Y matrix
          arma::mat Y = V * arma::diagmat(s) * V.t();

          // compute the Q vector
          arma::mat Q = Y * H_inv * Y;

          // compute the eigenvalues and eigenvectors of Q
          arma::vec eigval;
          arma::mat eigvec;
          arma::eig_sym(eigval, eigvec, Q);

          // initialize the A_hat vector
          arma::vec A_hat(4, arma::fill::ones);

          // find the eigenvector corresponding to the smallest eigenvalue
          // sort the eigenvalues
          arma::vec sorted_eigval = arma::sort(eigval);
          for (size_t j=0; j < sorted_eigval.size(); j++)
          {
            if (eigval(j) > 0)
            {
              A_hat = eigvec.col(j);
              break;
            }
          }

          // compute the A vector
          A = Y.i() * A_hat;
        }

        // compute the center and radius of the circle
        double x_center = -A(1) / (2*A(0)) + x_mean;
        double y_center = -A(2) / (2*A(0)) + y_mean;
        double radius = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4*A(0)*A(3)) / (4*std::pow(A(0), 2)));

        // log the center and radius of the circle
        RCLCPP_INFO(this->get_logger(), "Center: (%f, %f), Radius: %f", x_center, y_center, radius);

        // create a vector to store the center and radius of the circle
        std::vector<double> circle = {x_center, y_center, radius};
        circle_params.push_back(circle);
      }
      return circle_params;
    }

    /// \brief Publish the clusters as markers
    /// \param clusters The clusters to be published
    void publish_cluster_markers(const std::vector<std::vector<std::vector<double>>> & clusters)
    {
      // create a marker array
      visualization_msgs::msg::MarkerArray marker_array;

      // create a time stamp
      rclcpp::Time time = rclcpp::Clock().now();

      // iterate through the clusters
      for (size_t i=0; i<clusters.size(); i++)
      {
        // create a marker for each cluster
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "red/base_scan";
        marker.header.stamp = time;
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

          // check if any of the points is 0,0
          if (point.x == 0 && point.y == 0)
          {
            // log the zero point and the cluster id
            RCLCPP_INFO(this->get_logger(), "Zero point: Cluster id: %ld", i);
          }
        }
// ############################## End_Citation [10] ################################
        // add the marker to the marker array
        marker_array.markers.push_back(marker);
      }

      // publish the marker array
      cluster_pub_->publish(marker_array);
    }

    /// \brief Publish the landmarks as markers
    /// \param landmarks The landmarks to be published
    void publish_landmark_markers(const std::vector<std::vector<double>> & landmarks)
    {
      // create a marker array
      visualization_msgs::msg::MarkerArray marker_array;

      // create a time stamp
      rclcpp::Time time = rclcpp::Clock().now();

      // iterate through the landmarks
      for (size_t i=0; i<landmarks.size(); i++)
      {
        // create a marker for each landmark
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "red/base_scan";
        marker.header.stamp = time;
        marker.ns = "landmarks";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2*landmarks[i][2];
        marker.scale.y = 2*landmarks[i][2];
        marker.scale.z = 0.1;
        marker.color.g = 1.0;
        marker.color.a = 1.0;
        marker.pose.position.x = landmarks[i][0];
        marker.pose.position.y = landmarks[i][1];
        marker.pose.position.z = 0;

        // add the marker to the marker array
        marker_array.markers.push_back(marker);
      }

      // publish the marker array
      landmark_pub_->publish(marker_array);
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

/// \brief The main function.
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<landmarks>());
  rclcpp::shutdown();
  return 0;
}
