#include "img_transform/transform.hpp"

bool img_transform::compare(cv::DMatch i, cv::DMatch j)
{
    return (i.distance < j.distance);
}