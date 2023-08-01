#ifndef CIRCLE_DETECTION_INCLUDE_GUARD_HPP
#define CIRCLE_DETECTION_INCLUDE_GUARD_HPP

#include <vector>
#include <armadillo>

namespace turtlelib
{
    /// \brief Struct for points in clusters configuration
    struct Point{
        /// \brief the x position of a laser scan
        double x = 0.0;

        /// \brief the y position of a laser scan
        double y = 0.0;
    };

    /// \brief Struct for center and radius of a circle
    struct Circle{
        /// \brief the center point of the estimated circle
        Point center;

        /// \brief the radius point of the estimated circle
        double radius;
    };

    /// \brief compute the centroid of a cluster of points
    /// \param cluster - a cluster of points
    /// \return the centroid of a cluster of points
    Point compute_centroid(std::vector<Point> cluster);

    /// \brief compute the z mean of a cluster for the circle algorithm
    /// \param cluster_shifted - a cluster of points that is shifted over to center for circle
    /// \return the z mean
    double compute_mean_z(std::vector<Point> cluster_shifted);

    /// \brief create the Z matrix for the circle algorithm
    /// \param cluster_shifted - a cluster of points that is shifted over to center for circle
    /// \return the Z matrix
    arma::mat create_z_matrix(std::vector<Point> cluster_shifted);

    /// \brief compute the circle dimension given the A matrix
    /// \param A - the A matrix in the circle algorithm
    /// \return the circle center and radius
    Circle find_center_dim(arma::vec A);

    /// \brief find the center and radius of a circle fitted to a cluster of points
    /// \param cluster - a cluster of points
    /// \return the center and radius of a circle fitted to a cluster of points
    Circle find_circle(std::vector<Point> cluster);
}

#endif