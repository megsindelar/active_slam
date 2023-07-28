#include "img_transform/transform.hpp"
#include <Eigen/Geometry>

namespace img_transform
{
    bool compare(cv::DMatch i, cv::DMatch j)
    {
        return (i.distance < j.distance);
    }

    Vector2D trans_to_vec(Eigen::Matrix<double, 4, 4> Transform)
    {
        Vector2D vec;
        Eigen::Matrix3d rotation = Transform.block<3, 3>(0, 0);
        Eigen::Vector3d euler = rotation.eulerAngles(2,1,0);
        vec.theta = euler(0);
        vec.x = Transform(0,3);
        vec.y = Transform(1,3);
        return vec;
    }

    int Rot2Theta(Eigen::MatrixXd rot)
    {
        return atan2(rot(1,0), rot(0,0));
    }

} // namespace img_transform