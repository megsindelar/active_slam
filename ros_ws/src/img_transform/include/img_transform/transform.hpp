#ifndef TRANSFORM_INCLUDE_GUARD_HPP
#define TRANSFORM_INCLUDE_GUARD_HPP
/// \file
/// \brief Calculating the discriminant of a quadratic.

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace img_transform
{
    bool compare(cv::DMatch i, cv::DMatch j);
}
#endif