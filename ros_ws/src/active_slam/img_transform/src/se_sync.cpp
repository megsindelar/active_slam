#include "rclcpp/rclcpp.hpp"
#include "img_transform/transform.hpp"
#include "img_transform/msg/transform.hpp"
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"
#include <Eigen/Core>

// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>

using namespace std;
using namespace SESync;

class SE_SYNC : public rclcpp::Node
{
public:
  SE_SYNC()
  : Node("se_sync")
  {
    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&SE_SYNC::timer_callback, this));

    // subscriber to transform between two frames
    sub_transform_ = this->create_subscription<img_transform::msg::Transform>(
      "/transform", 10, std::bind(
        &SE_SYNC::transform_callback,
        this, std::placeholders::_1));

    // pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //   "nodes", 10);

    // pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //   "edges", 10);

    // // subscriber to robot state
    // sub_robot_state_ = this->create_subscription<img_transform::msg::Transform>(
    //   "/robot_state", 10, std::bind(
    //     &SE_SYNC::robot_state_callback,
    //     this, std::placeholders::_1));

    // pub_slam_path_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);
  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    ////////////////////////////////////////////////////////////////////////////////////////////
    // SE_SYNC
    ////////////////////////////////////////////////////////////////////////////////////////////
    size_t num_poses = 0;
    measurements_t measurements = read_g2o_file("/home/megsindelar/Final_Project/ros_ws/src/active_slam/img_transform/data/city10000.g2o", num_poses);

    if (measurements.size() == 0) {
        throw std::logic_error("Error: No measurements were read! Are you sure the file exists?");
    }

    SESyncOpts opts;
    opts.verbose = true; // Print output to stdout

    // Initialization method
    // Options are:  Chordal, Random
    opts.initialization = Initialization::Chordal;

    // Specific form of the synchronization problem to solve
    // Options are: Simplified, Explicit, SOSync
    opts.formulation = Formulation::Simplified;

    //   Initial
    opts.num_threads = 4;

    // #ifdef GPERFTOOLS
    // ProfilerStart("SE-Sync.prof");
    // #endif

    //   / RUN SE-SYNC!
    SESyncResult results = SESync::SESync(measurements, opts);

    ///////////////////////////////////////////////////////////////////////////////////////////
    // SE_SYNC - done
    ///////////////////////////////////////////////////////////////////////////////////////////

    Eigen::MatrixXd xhat = results.xhat;
    int trans_size = xhat.row(0).size()/3;
    for (int i = 0; i < trans_size; i++){
        double x = xhat.col(i)(0);
        double y = xhat.col(i)(1);
        Eigen::MatrixXd R = xhat.block(0, ((trans_size-1) + i), 2, 2);
        double theta = img_transform::Rot2Theta(R);
    }

//     visualization_msgs::msg::MarkerArray node_markers;
//     visualization_msgs::msg::Marker node;
//     node.header.frame_id = "world";
//     node.header.stamp = this->get_clock()->now();
//     node.id = 0;
//     node.action = visualization_msgs::msg::Marker::ADD;
//     node.pose.orientation.w = 1.0;
//     node.pose.position.x = 0.0;
//     node.pose.position.y = 0.0;
//     node.type = visualization_msgs::msg::Marker::SPHERE;
//     node.scale.x = 0.1;
//     node.scale.y = 0.1;
//     node.scale.z = 0.1;
//     node.color.a = 1.0;
//     node.color.r = 0.9;
//     node.color.g = 0.3;
//     node.color.b = 0.3;


//     visualization_msgs::msg::MarkerArray edge_markers;
//     visualization_msgs::msg::Marker edge;
//     edge.header.frame_id = "world";
//     edge.header.stamp = this->get_clock()->now();
//     edge.id = 0;
//     edge.action = visualization_msgs::msg::Marker::ADD;
//     edge.pose.orientation.w = 1.0;
//     edge.pose.position.x = 0.0;
//     edge.pose.position.y = 0.0;
//     edge.type = visualization_msgs::msg::Marker::SPHERE;
//     edge.scale.x = 0.1;
//     edge.scale.y = 0.1;
//     edge.scale.z = 0.1;
//     edge.color.a = 1.0;
//     edge.color.r = 0.9;
//     edge.color.g = 0.3;
//     edge.color.b = 0.3;



//     int id = 0;
//     for (int i = 0; i < ; i++){
        
//         node_markers.markers.push_back(node);
//         id++;
//     }

//     for (int i = 0; i < ; i++){
        
//         edge_markers.markers.push_back(edge);
//     }

//     pub_nodes_->publish(node_markers);
//     pub_edges_->publish(edge_markers);
//   }


    // // publish path to see how the robot has moved
    // geometry_msgs::msg::PoseStamped pos;
    // path.header.stamp = this->get_clock()->now();
    // path.header.frame_id = "world";
    // pos.header.stamp = this->get_clock()->now();
    // pos.header.frame_id = "world";
    // pos.pose.position.x = ;
    // pos.pose.position.y = ;

    // Eigen::Quaterniond q(rotation_current);
    // geometry_msgs::msg::Quaternion msg_quaternion_path = tf2::toMsg(q);
    // pos.pose.orientation.x = msg_quaternion_path.x;
    // pos.pose.orientation.y = msg_quaternion_path.y;
    // pos.pose.orientation.z = msg_quaternion_path.z;
    // pos.pose.orientation.w = msg_quaternion_path.w;
    // path.poses.push_back(pos);
    // pub_slam_path_->publish(path);


  }

  void transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
    id_trans = msg->id;
    Eigen::Matrix<double, 4, 4> transformation;
    transformation.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    transformation.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    transformation.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    transformation.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);

    img_transform::Vector2D trans;
    trans = img_transform::trans_to_vec(transformation);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 2, 3> information_matrix;
    information_matrix.row(0) << a*1, 0, 0;
    information_matrix.row(1) << a*1, 0, a*1;

    img_transform::Edge edge;
    edge.id_a = id_trans - 1;
    edge.id_b = id_trans;
    edge.transform = trans;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);
  }

//   void robot_state_callback(
//     const img_transform::msg::Transform::ConstSharedPtr& msg
//   ){
//     id_rob_state = msg->id;
//     Eigen::Matrix<double, 4, 4> state;
//     state.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
//     state.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
//     state.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
//     state.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);
//     rob_state = img_transform::trans_to_vec(state);

//     img_transform::Vertex vertex;
//     vertex.id = id_rob_state;
//     vertex.state = rob_state;
//     vertex.type = "VERTEX_SE2";
//     vertices.push_back(vertex);
//   }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_transform_;
//   rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_robot_state_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_slam_path_;

  img_transform::Vector2D transform;
  img_transform::Vector2D rob_state;
  std::vector<img_transform::Edge> edges;
  std::vector<img_transform::Vertex> vertices{{"VERTEX_SE2", 0, {0.0, 0.0, 0.0}}};
  int id_trans = 0;
  int id_rob_state = 0;
  int a = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SE_SYNC>());
  try {
    rclcpp::spin(std::make_shared<SE_SYNC>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      std::make_shared<SE_SYNC>()->get_logger(),
      "ERROR: " << e.what());
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
