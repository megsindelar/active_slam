#include "rclcpp/rclcpp.hpp"
#include "img_transform/transform.hpp"
#include "img_transform/msg/transform.hpp"
#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"
#include <Eigen/Core>

#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>
#include <Eigen/SPQRSupport>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/point.hpp"

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

    pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nodes", 10);

    pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "edges", 10);

    // subscriber to robot state
    sub_robot_state_ = this->create_subscription<img_transform::msg::Transform>(
      "/robot_state", 10, std::bind(
        &SE_SYNC::robot_state_callback,
        this, std::placeholders::_1));

    // pub_slam_path_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);
  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    ////////////////////////////////////////////////////////////////////////////////////////////
    // SE_SYNC
    ////////////////////////////////////////////////////////////////////////////////////////////

    // measurements_t measurements = read_g2o_file("/home/megsindelar/Final_Project/ros_ws/src/active_slam/img_transform/data/city10000.g2o", num_poses);
    // Preallocate output vector
    RelativePoseMeasurement measurement;

    // if (first){
    //     // Initialize
    //     // Fill in elements of this measurement

    //     // Pose ids
    //     measurement.i = i;
    //     measurement.j = j;

    //     // Raw measurements
    //     measurement.t.resize(2);
    //     measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
    //     measurement.R.resize(2,2);
    //     measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

    //     Eigen::Matrix<Scalar, 2, 2> TranInfo;
    //     TranInfo << I11, I12, I12, I22;
    //     measurement.tau = 2 / TranInfo.inverse().trace();

    //     measurement.kappa = I33;
        
    //     // Update maximum value of poses found so far
    //     size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

    //     measurements.push_back(measurement);
        // first = false;
    // }

    if (edges.size() != 0){
        // A single measurement, whose values we will fill in

        // A string used to contain the contents of a single line
        // string line;

        // A string used to extract tokens from each line one-by-one
        // string token;

        // Preallocate various useful quantities

        // while (std::getline(infile, line)) {

        // This is a 2D pose measurement

        /** The g2o format specifies a 2D relative pose measurement in the
         * following form:
         *
         * EDGE_SE2 id1 id2 dx dy dtheta, I11, I12, I13, I22, I23, I33
         *
         */

        // Extract formatted output
        // strstrm >> i >> j >> dx >> dy >> dtheta >> I11 >> I12 >> I13 >> I22 >>
        //     I23 >> I33;
        i_ = edges[i_].id_a;
        j_ = edges[i_].id_b;
        dx = edges[i_].transform.x;
        dy = edges[i_].transform.y;
        dtheta = edges[i_].transform.theta;
        I11 = edges[i_].info_matrix.row(0)(0);
        I12 = edges[i_].info_matrix.row(0)(1);
        I13 = edges[i_].info_matrix.row(0)(2);
        I22 = edges[i_].info_matrix.row(1)(1);
        I23 = edges[i_].info_matrix.row(1)(2);
        I33 = edges[i_].info_matrix.row(2)(2);

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i_;
        measurement.j = j_;

        // Raw measurements
        measurement.t.resize(2);
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R.resize(2,2);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;
        
        // Update maximum value of poses found so far
        size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        measurements.push_back(measurement);
    }


    if (measurements.size() == 0) {
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Error: No measurements were read!");
        RCLCPP_INFO(rclcpp::get_logger(" "), " ");
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("message"), "Measurement: %d", measurements.size());
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

        // make an id for each pose and each pose pair for edges (like connecting non-sequential nodes for looping back)

        Eigen::MatrixXd xhat = results.xhat;
        vector<double> x;
        vector<double> y;
        vector<double> theta;
        vector<int> id_1;
        vector<int> id_2;
        int trans_size = xhat.row(0).size()/3;
        for (int i = 0; i < trans_size; i++){
            id_1.push_back(i_);
            id_2.push_back(j_);
            x.push_back(xhat.col(i)(0));
            y.push_back(xhat.col(i)(1));

            //TODO maybe change if needed
            Eigen::MatrixXd R = xhat.block(0, ((trans_size-1) + i), 2, 2);
            theta.push_back(img_transform::Rot2Theta(R));
        }

        visualization_msgs::msg::MarkerArray node_markers;
        visualization_msgs::msg::Marker node;
        node.header.frame_id = "world";
        node.header.stamp = this->get_clock()->now();
        node.id = 0;
        node.action = visualization_msgs::msg::Marker::ADD;
        node.pose.orientation.w = 1.0;
        node.pose.position.x = 0.0;
        node.pose.position.y = 0.0;
        node.type = visualization_msgs::msg::Marker::SPHERE;
        node.scale.x = 0.1;
        node.scale.y = 0.1;
        node.scale.z = 0.1;
        node.color.a = 1.0;
        node.color.r = 0.9;
        node.color.g = 0.3;
        node.color.b = 0.3;


        visualization_msgs::msg::MarkerArray edge_markers;
        visualization_msgs::msg::Marker edge_m;
        edge_m.header.frame_id = "world";
        edge_m.header.stamp = this->get_clock()->now();
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
        for (int i = 0; i < vertices.size(); i++){
            node.id = id;
            node.pose.position.x = vertices[i].state.x;
            node.pose.position.y = vertices[i].state.y;
            node_markers.markers.push_back(node);
            id++;
        }

        id = 0;
        // TODO: change later when having loop back to num_edges.size instead of x positions
        for (int i = 0; i < (x.size() - 1); i++){
            edge_m.id = id;
            geometry_msgs::msg::Point p;
            p.x = x[i];
            p.y = y[i];
            edge_m.points.push_back(p);
            p.x = x[i+1];
            p.y = y[i+1];
            edge_m.points.push_back(p);
            edge_markers.markers.push_back(edge_m);
            id++;
        }

        pub_nodes_->publish(node_markers);
        pub_edges_->publish(edge_markers);
    }


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
    RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
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
    information_matrix.row(1) << a*1, 0, a*2;

    img_transform::Edge edge;
    edge.id_a = id_trans - 1;
    edge.id_b = id_trans;
    edge.transform = trans;
    edge.type = "EDGE_SE2";
    // edge.info_matrix = information_matrix;
    edges.push_back(edge);
    RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
  }

  void robot_state_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    id_rob_state = msg->id;
    Eigen::Matrix<double, 4, 4> state;
    state.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    state.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    state.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    state.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);
    rob_state = img_transform::trans_to_vec(state);

    img_transform::Vertex vertex;
    vertex.id = id_rob_state;
    vertex.state = rob_state;
    vertex.type = "VERTEX_SE2";
    vertices.push_back(vertex);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_robot_state_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_slam_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nodes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;

  img_transform::Vector2D transform;
  img_transform::Vector2D rob_state;
  std::vector<img_transform::Edge> edges;
  std::vector<img_transform::Vertex> vertices{{"VERTEX_SE2", 0, {0.0, 0.0, 0.0}}};
  int id_trans = 0;
  int id_rob_state = 0;
  int a = 50;
  size_t num_poses = 0;
  measurements_t measurements;
  size_t i_ = 0;
  size_t j_ = 1;
  Scalar dx = 0.0;
  Scalar dy = 0.0;
  Scalar dtheta = 0.0;
  Scalar I11 = 50;
  Scalar I12 = 0;
  Scalar I13 = 0;
  Scalar I22 = 50;
  Scalar I23 = 0;
  Scalar I33 = 100;

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
