#ifndef GRAPH_INCLUDE_GUARD_HPP
#define GRAPH_INCLUDE_GUARD_HPP

#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

typedef std::vector<visualization_msgs::msg::MarkerArray> MarkerArrayVec;

namespace img_transform
{
    MarkerArrayVec visualize_graph(std::vector<std::vector<double>> nodes, std_msgs::msg::Header header);
}

#endif