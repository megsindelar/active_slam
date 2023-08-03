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

#include "std_srvs/srv/empty.hpp"

#include "std_msgs/msg/string.hpp"

#include "img_transform/graph.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

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
    sub_image_transform_ = this->create_subscription<img_transform::msg::Transform>(
      "/image_transform", 10, std::bind(
        &SE_SYNC::image_transform_callback,
        this, std::placeholders::_1));

    sub_wheel_transform_ = this->create_subscription<img_transform::msg::Transform>(
      "/wheel_transform", 10, std::bind(
        &SE_SYNC::wheel_transform_callback,
        this, std::placeholders::_1));

    pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nodes", 10);

    pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "edges", 10);

    sub_sesync_trigger_ = this->create_subscription<std_msgs::msg::String>(
      "/sesync_trigger", 10, std::bind(
        &SE_SYNC::start,
        this, std::placeholders::_1));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // pub_slam_path_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);
  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    // MarkerArrayVec marker_vec = img_transform::visualize_graph(nodes, header);

    if (reconstruction)
    {
        reconstruction = false;
        ////////////////////////////////////////////////////////////////////////////////////////////
        // SE_SYNC
        ////////////////////////////////////////////////////////////////////////////////////////////

        // measurements_t measurements = read_g2o_file("/home/megsindelar/Final_Project/ros_ws/src/active_slam/img_transform/data/city10000.g2o", num_poses);
        // Preallocate output vector
        RCLCPP_INFO(rclcpp::get_logger("message"), "SESync starting");
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
        RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
        for (int i = 0; i < edges.size(); i++){
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
            i_ = edges[i].id_a;
            j_ = edges[i].id_b;
            dx = edges[i].transform.x;
            dy = edges[i].transform.y;
            dtheta = edges[i].transform.theta;
            I11 = 50; //edges[i].info_matrix.row(0)(0);
            I12 = 0; //edges[i].info_matrix.row(0)(1);
            I13 = 0; //edges[i].info_matrix.row(0)(2);
            I22 = 50; //edges[i].info_matrix.row(1)(1);
            I23 = 0; //edges[i].info_matrix.row(1)(2);
            I33 = 100; //edges[i].info_matrix.row(2)(2);

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
            RCLCPP_INFO(rclcpp::get_logger("message"), "No measurements!");
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
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 1");

            // #ifdef GPERFTOOLS
            // ProfilerStart("SE-Sync.prof");
            // #endif

            //   / RUN SE-SYNC!
            SESyncResult results = SESync::SESync(measurements, opts);
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2");

            ///////////////////////////////////////////////////////////////////////////////////////////
            // SE_SYNC - done
            ///////////////////////////////////////////////////////////////////////////////////////////

            // make an id for each pose and each pose pair for edges (like connecting non-sequential nodes for looping back)

            Eigen::MatrixXd xhat = results.xhat;

            std::vector<std::vector<double>> nodes;

            double x;
            double y;
            double theta;
            int trans_size = xhat.row(0).size()/3;
            for (int i = 0; i < trans_size; i++){
                x = xhat.col(i)(0);
                y = xhat.col(i)(1);

                //TODO maybe change if needed
                Eigen::MatrixXd R = xhat.block(0, ((trans_size-1) + i), 2, 2);
                theta = img_transform::Rot2Theta(R);

                nodes.push_back({x, y, theta});
            }
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 3");

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();

            // MarkerArrayVec marker_vec = img_transform::visualize_graph(nodes, header);

            geometry_msgs::msg::TransformStamped t;
            for (int i = 0; i < nodes.size(); i++){
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "world";
                t.child_frame_id = "frame" + to_string(i);
                t.transform.translation.x = nodes[i][0];
                t.transform.translation.y = nodes[i][1];

                tf2::Quaternion q;
                q.setRPY(0, 0, nodes[i][2]);
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(t);
            }


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







            RCLCPP_INFO(rclcpp::get_logger("message"), "Publishing markers!");

            // pub_nodes_->publish(marker_vec[0]);
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
  }

  void start(
    const std_msgs::msg::String::ConstSharedPtr& msg
  ){
    reconstruction = true;
  }

  void image_transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
    // id_trans = msg->id;
    Eigen::Matrix<double, 4, 4> image_transformation;
    image_transformation.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    image_transformation.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    image_transformation.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    image_transformation.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);

    img_transform::Vector2D trans;
    trans = img_transform::trans_to_vec(image_transformation);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 2, 3> information_matrix;
    information_matrix.row(0) << a*1, 0, 0;
    information_matrix.row(1) << a*1, 0, a*1;

    img_transform::Edge edge;
    edge.id_a = id_trans;
    edge.id_b = 0;
    edge.transform = trans;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
  }

  void wheel_transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
    id_trans = msg->id;
    Eigen::Matrix<double, 4, 4> wheel_transformation;
    wheel_transformation.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    wheel_transformation.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    wheel_transformation.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    wheel_transformation.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);

    img_transform::Vector2D trans;
    trans = img_transform::trans_to_vec(wheel_transformation);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 2, 3> information_matrix;
    information_matrix.row(0) << I11, I12, I13;
    information_matrix.row(1) << I22, I23, I33;


    img_transform::Edge edge;
    edge.id_a = id_trans - 1;
    edge.id_b = id_trans;
    edge.transform = trans;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_image_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_wheel_transform_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sesync_trigger_;
//   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_slam_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nodes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;
//   rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reconstruction_srv;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

  bool reconstruction = false;

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
