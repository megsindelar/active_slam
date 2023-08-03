#ifndef TRANSFORM_INCLUDE_GUARD_HPP
#define TRANSFORM_INCLUDE_GUARD_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"

namespace img_transform
{
    // 2D vector struct of x, y, and theta
    struct Vector2D{
        SESync::Scalar x;
        SESync::Scalar y;
        SESync::Scalar theta;
    };

    // struct for VERTEX_SE2 for g2o files
    struct Vertex{
        std::string type;
        std::size_t id;
        struct Vector2D state;
    };

    // struct for EDGE_SE2 for g2o files
    struct Edge{
        std::string type;
        std::size_t id_a;
        std::size_t id_b;
        struct Vector2D transform;
        Eigen::Matrix<SESync::Scalar, 2, 3> info_matrix;
    };

    // convert rotation mat to yaw
    int Rot2Theta(Eigen::MatrixXd rot);

    // compare keypoint distances to see if smaller
    bool compare(cv::DMatch i, cv::DMatch j);

    // convert a 2D Transformation to a vector of x, y, theta
    Vector2D trans_to_vec(Eigen::Matrix<double, 4, 4> Transform);
}
#endif