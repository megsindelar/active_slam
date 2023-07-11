#ifndef TRANSFORM_INCLUDE_GUARD_HPP
#define TRANSFORM_INCLUDE_GUARD_HPP
/// \file
/// \brief Calculating the discriminant of a quadratic.

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>

namespace img_transform
{
    // 2D vector struct of x, y, and theta
    struct Vector2D{
        double x;
        double y;
        double theta;
    };

    // struct for VERTEX_SE2 for g2o files
    struct Vertex{
        int id;
        struct Vector2D state;
    };

    // struct for EDGE_SE2 for g2o files
    struct Edge{
        int id_a;
        int id_b;
        struct Vector2D transform;
        Eigen::Matrix<double, 2, 3> info_matrix;
    };

    // compare keypoint distances to see if smaller
    bool compare(cv::DMatch i, cv::DMatch j);

    // convert a 2D Transformation to a vector of x, y, theta
    Vector2D trans_to_vec(Eigen::Matrix<double, 4, 4> Transform);
}
#endif