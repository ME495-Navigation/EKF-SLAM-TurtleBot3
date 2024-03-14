#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <armadillo>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

/// \brief Circle fitting algorithm
/// \param clusters The clusters to be fitted
/// \return The center and radius of the fitted circle
std::vector<std::vector<double>> circle_fit(
  const std::vector<std::vector<std::vector<double>>> & clusters)
{
// create a vector to store the centers and radii of the circles
  std::vector<std::vector<double>> circle_params;

// process each cluster separately
  for (size_t i = 0; i < clusters.size(); i++) {
// find the mean of the x coordinates
    double x_mean = 0;
    for (size_t j = 0; j < clusters[i].size(); j++) {
      x_mean += clusters[i][j][0];
    }
    x_mean /= clusters[i].size();

// find the mean of the y coordinates
    double y_mean = 0;
    for (size_t j = 0; j < clusters[i].size(); j++) {
      y_mean += clusters[i][j][1];
    }
    y_mean /= clusters[i].size();

// shift the coordinates so that centroid is at the origin
    std::vector<std::vector<double>> shifted_coordinates;
    for (size_t j = 0; j < clusters[i].size(); j++) {
      std::vector<double> point = {clusters[i][j][0] - x_mean, clusters[i][j][1] - y_mean};
      shifted_coordinates.push_back(point);
    }

// compute z_i
    std::vector<double> z_i;
    for (size_t j = 0; j < shifted_coordinates.size(); j++) {
      z_i.push_back(
        std::pow(
          shifted_coordinates[j][0],
          2) + std::pow(shifted_coordinates[j][1], 2));
    }

// compute the mean of z_i
    double z_mean = 0;
    for (size_t j = 0; j < z_i.size(); j++) {
      z_mean += z_i[j];
    }
    z_mean /= z_i.size();

// form the data matrix Z
    arma::mat Z(shifted_coordinates.size(), 4, arma::fill::ones);
// make the first column equal to z_i
// make second and third columns equal to x_i and y_i
    for (size_t j = 0; j < z_i.size(); j++) {
      Z(j, 0) = z_i[j];
      Z(j, 1) = shifted_coordinates[j][0];
      Z(j, 2) = shifted_coordinates[j][1];
    }

// form the moment matrix M
    arma::mat M = Z.t() * Z;
    M /= clusters[i].size();

// form the constraint matrix H
    arma::mat H = {{8 * z_mean, 0, 0, 2}, {0, 1, 0, 0}, {0, 0, 1, 0}, {2, 0, 0, 0}};

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
    if (s(s.size() - 1) < 1e-12) {
      // set the A vector to the last column of V
      A = V.col(V.n_cols - 1);
    } else {
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
      for (size_t j = 0; j < eigval.size(); j++) {
        if (eigval(j) > 0) {
          A_hat = eigvec.col(j);
          break;
        }
      }

      // compute the A vector
      A = Y.i() * A_hat;
    }

// compute the center and radius of the circle
    double x_center = -A(1) / (2 * A(0)) + x_mean;
    double y_center = -A(2) / (2 * A(0)) + y_mean;
    double radius =
      std::sqrt(
      (std::pow(
        A(1),
        2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2)));

// log the center and radius of the circle
// RCLCPP_INFO(this->get_logger(), "Center: (%f, %f), Radius: %f", x_center, y_center, radius);

// create a vector to store the center and radius of the circle
    std::vector<double> circle = {x_center, y_center, radius};
    circle_params.push_back(circle);
  }
  return circle_params;
}

TEST_CASE("circle fit 1", "[circle fit]")
{

  std::vector<std::vector<std::vector<double>>> clusters;
  // add a cluster
  std::vector<std::vector<double>> cluster;
  cluster.push_back({1.0, 7.0});
  cluster.push_back({2.0, 6.0});
  cluster.push_back({5.0, 8.0});
  cluster.push_back({7.0, 7.0});
  cluster.push_back({9.0, 5.0});
  cluster.push_back({3.0, 7.0});
  clusters.push_back(cluster);

  std::vector<std::vector<double>> circle_params = circle_fit(clusters);

  REQUIRE_THAT(circle_params[0][0], Catch::Matchers::WithinAbs(4.615482, 1e-4));
  REQUIRE_THAT(circle_params[0][1], Catch::Matchers::WithinAbs(2.807354, 1e-4));
  REQUIRE_THAT(circle_params[0][2], Catch::Matchers::WithinAbs(4.8275, 1e-4));
}

TEST_CASE("circle fit 2", "[circle fit]")
{

  std::vector<std::vector<std::vector<double>>> clusters;
  // add a cluster
  std::vector<std::vector<double>> cluster;
  cluster.push_back({-1.0, 0.0});
  cluster.push_back({-0.3, -0.06});
  cluster.push_back({0.3, 0.1});
  cluster.push_back({1.0, 0.0});
  clusters.push_back(cluster);

  std::vector<std::vector<double>> circle_params = circle_fit(clusters);

  REQUIRE_THAT(circle_params[0][0], Catch::Matchers::WithinAbs(0.4908357, 1e-4));
  REQUIRE_THAT(circle_params[0][1], Catch::Matchers::WithinAbs(-22.15212, 1e-4));
  REQUIRE_THAT(circle_params[0][2], Catch::Matchers::WithinAbs(22.17979, 1e-4));
}
