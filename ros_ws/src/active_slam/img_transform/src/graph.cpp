#include "img_transform/graph.hpp"

namespace img_transform
{
    MarkerArrayVec visualize_graph(std::vector<std::vector<double>> nodes, std_msgs::msg::Header header){
        // visualization_msgs::msg::MarkerArray node_markers;
        // visualization_msgs::msg::Marker node;
        // node.header.frame_id = "world";
        // node.header.stamp = header.stamp;
        // node.id = 0;
        // node.action = visualization_msgs::msg::Marker::ADD;
        // node.pose.orientation.w = 1.0;
        // node.pose.position.x = 0.0;
        // node.pose.position.y = 0.0;
        // node.type = visualization_msgs::msg::Marker::SPHERE;
        // node.scale.x = 0.1;
        // node.scale.y = 0.1;
        // node.scale.z = 0.1;
        // node.color.a = 1.0;
        // node.color.r = 0.3;
        // node.color.g = 0.9;
        // node.color.b = 0.3;


        visualization_msgs::msg::MarkerArray edge_markers;
        visualization_msgs::msg::Marker edge_m;
        edge_m.header.frame_id = "world";
        edge_m.header.stamp = header.stamp;
        edge_m.id = 0;
        edge_m.action = visualization_msgs::msg::Marker::ADD;
        edge_m.pose.orientation.w = 1.0;
        edge_m.pose.position.x = 0.0;
        edge_m.pose.position.y = 0.0;
        edge_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        edge_m.scale.x = 0.1;
        edge_m.scale.y = 0.1;
        edge_m.scale.z = 0.1;
        edge_m.color.a = 1.0;
        edge_m.color.r = 0.9;
        edge_m.color.g = 0.3;
        edge_m.color.b = 0.3;

        int id = 0;
        // for (int i = 0; i < nodes.size(); i++){
        //     node.id = id;
        //     node.pose.position.x = nodes[i][0];
        //     node.pose.position.y = nodes[i][1];
        //     node_markers.markers.push_back(node);
        //     id++;
        // }

        id = 0;
        // TODO: change later when having loop back to num_edges.size instead of x positions
        for (int i = 0; i < (nodes.size() - 1); i++){
            edge_m.id = id;
            geometry_msgs::msg::Point p;
            p.x = nodes[i][0];
            p.y = nodes[i][1];
            edge_m.points.push_back(p);
            p.x = nodes[i+1][0];
            p.y = nodes[i+1][1];
            edge_m.points.push_back(p);
            edge_markers.markers.push_back(edge_m);
            id++;
        }

    MarkerArrayVec v;
    // v.push_back(node_markers);
    v.push_back(edge_markers);
    
    return v;
    }

} // namespace img_transform