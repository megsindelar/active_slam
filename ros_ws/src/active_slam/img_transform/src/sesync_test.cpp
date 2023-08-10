#include "rclcpp/rclcpp.hpp"
#include "img_transform/transform.hpp"
#include "img_transform/msg/odom.hpp"
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

#include "sophus/se3.hpp"

#include "math.h"

using namespace std;
using namespace SESync;

class SE_SYNC : public rclcpp::Node
{
public:
  SE_SYNC()
  : Node("se_sync")
  {
    /// \brief parameter that is the difference between wheel base and cam in x-direction
    /// \param dx_cam - convert encoder ticks to radians
    this->declare_parameter("dx_cam", -1.0);
    dx_cam =
      this->get_parameter("dx_cam").get_parameter_value().get<double>();
    if (dx_cam == -1.0) {throw std::logic_error("All parameters are not set!");}

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

    sub_odom_ = this->create_subscription<img_transform::msg::Odom>(
      "/odom_orientation", 10, std::bind(
        &SE_SYNC::odom_callback,
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

    // publish current wheel base and cam frame
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "cam_frame";
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_pos);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::TransformStamped t_cam;
    t_cam.header.stamp = this->get_clock()->now();
    t_cam.header.frame_id = "cam_frame";
    t_cam.child_frame_id = "wheel_base_frame";
    t_cam.transform.translation.x = -dx_cam;
    t_cam.transform.translation.y = 0.0;

    tf2::Quaternion q_cam;
    q_cam.setRPY(0, 0, 0.0);
    t_cam.transform.rotation.x = q_cam.x();
    t_cam.transform.rotation.y = q_cam.y();
    t_cam.transform.rotation.z = q_cam.z();
    t_cam.transform.rotation.w = q_cam.w();

    tf_broadcaster_->sendTransform(t_cam);

    if (reconstruction)
    // if (count < 100)
    {
        Eigen::Matrix<double, 3, 3> rot_;
        Eigen::Matrix<double, 3, 1> trans_;
        rot_.row(0) << cos(M_PI), -sin(M_PI), 0.0;
        rot_.row(1) << sin(M_PI), cos(M_PI), 0.0;
        rot_.row(2) << 0.0, 0.0, 1.0;
        trans_(0) = 0.0;
        trans_(1) = 0.0;
        trans_(2) = 0.0;

        Sophus::SE3d T_fix(rot_, trans_);


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
        //     first = false;
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
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 0");
            i_ = edges[i].id_a;
            j_ = edges[i].id_b;
            // dx = edges[i].transform.x;
            // dy = edges[i].transform.y;
            // dtheta = edges[i].transform.theta;
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 0.5");
            Eigen::Matrix<double, 3, 3> rotations = edges[i].rot;
            Eigen::Matrix<double, 3, 1> translations = edges[i].trans;

            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 0.7");

            Sophus::SE3 T_edge(rotations, translations);

            RCLCPP_INFO(rclcpp::get_logger("message"), "T_edge row 1: %f, %f, %f, %f", T_edge.matrix()(0,0), T_edge.matrix()(0,1), T_edge.matrix()(0,2), T_edge.matrix()(0,3));
            RCLCPP_INFO(rclcpp::get_logger("message"), "T_edge row 2: %f, %f, %f, %f", T_edge.matrix()(1,0), T_edge.matrix()(1,1), T_edge.matrix()(1,2), T_edge.matrix()(1,3));
            RCLCPP_INFO(rclcpp::get_logger("message"), "T_edge row 3: %f, %f, %f, %f", T_edge.matrix()(2,0), T_edge.matrix()(2,1), T_edge.matrix()(2,2), T_edge.matrix()(2,3));
            RCLCPP_INFO(rclcpp::get_logger("message"), "T_edge row 4: %f, %f, %f, %f", T_edge.matrix()(3,0), T_edge.matrix()(3,1), T_edge.matrix()(3,2), T_edge.matrix()(3,3));

            RCLCPP_INFO(rclcpp::get_logger("message"), "dx: %f", dx);
            RCLCPP_INFO(rclcpp::get_logger("message"), "dy: %f", dy);
            RCLCPP_INFO(rclcpp::get_logger("message"), "dtheta: %f", dtheta);


            I11 = edges[i].info_matrix(0,0);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I11: %f", I11);
            I12 = edges[i].info_matrix(0,1);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I12: %f", I12);
            I13 = edges[i].info_matrix(0,2);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I13: %f", I13);
            I22 = edges[i].info_matrix(1,1);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I22: %f", I22);
            I23 = edges[i].info_matrix(1,2);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I23: %f", I23);
            I33 = edges[i].info_matrix(2,2);
            RCLCPP_INFO(rclcpp::get_logger("message"), "I33: %f", I33);

            // Fill in elements of this measurement

            // Pose ids
            measurement.i = i_;
            measurement.j = j_;

            // Raw measurements
            measurement.t.resize(2);
            measurement.t = T_edge.translation().block(0,0,2,1);
            measurement.R.resize(2,2);
            measurement.R = T_edge.rotationMatrix().block(0,0,2,2);

            RCLCPP_INFO(rclcpp::get_logger("message"), "Measurement 0: %f, %f", measurement.R(0,0), measurement.R(0,1));
            RCLCPP_INFO(rclcpp::get_logger("message"), "Measurement 1: %f, %f", measurement.R(1,0), measurement.R(1,1));
            RCLCPP_INFO(rclcpp::get_logger("message"), "Trans 1: %f, %f", measurement.t(0), measurement.t(1));

            Eigen::Matrix<Scalar, 2, 2> TranInfo;
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 1");
            TranInfo << I11, I12, I12, I22;
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2");
            measurement.tau = 2 / TranInfo.inverse().trace();

            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 3");

            measurement.kappa = I33;
            
            // Update maximum value of poses found so far
            size_t max_pair = std::max<size_t>(measurement.i, measurement.j);
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 4");

            measurements.push_back(measurement);
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 5");
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
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 1/2");

            // #ifdef GPERFTOOLS
            // ProfilerStart("SE-Sync.prof");
            // #endif

            //   / RUN SE-SYNC!
            SESyncResult results = SESync::SESync(measurements, opts);
            RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2/3");

            ///////////////////////////////////////////////////////////////////////////////////////////
            // SE_SYNC - done
            ///////////////////////////////////////////////////////////////////////////////////////////

            // make an id for each pose and each pose pair for edges (like connecting non-sequential nodes for looping back)

            Eigen::MatrixXd xhat = results.xhat;
            RCLCPP_INFO(rclcpp::get_logger("message"), "T 1: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", results.xhat.col(0)(0), results.xhat.col(1)(0), results.xhat.col(2)(0), results.xhat.col(3)(0), results.xhat.col(4)(0),results.xhat.col(5)(0), results.xhat.col(6)(0), results.xhat.col(7)(0), results.xhat.col(8)(0), results.xhat.col(9)(0), results.xhat.col(10)(0), results.xhat.col(11)(0), results.xhat.col(12)(0));
            RCLCPP_INFO(rclcpp::get_logger("message"), "T 2: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", results.xhat.col(0)(1), results.xhat.col(1)(1), results.xhat.col(2)(1), results.xhat.col(3)(1), results.xhat.col(4)(1),results.xhat.col(5)(1), results.xhat.col(6)(1), results.xhat.col(7)(1), results.xhat.col(8)(1), results.xhat.col(9)(1), results.xhat.col(10)(1), results.xhat.col(11)(1), results.xhat.col(12)(1));

            std::vector<std::vector<double>> nodes;

            Eigen::Matrix<double, 4,4> T_updated;
            int trans_size = edges.size();
            RCLCPP_INFO(rclcpp::get_logger("message"), "size: %d", trans_size);

            double x;
            double y;
            double theta;

            for (int i = 0; i < trans_size; i++){
                T_updated.row(0) << xhat.col(trans_size + i)(0), xhat.col(trans_size + 1 + i)(0), 0.0, xhat.col(i)(0);
                // RCLCPP_INFO(rclcpp::get_logger("message"), "T 1: %f, %f, %f, %f", results.xhat.col((trans_size-1) + i)(0), results.xhat.col(trans_size + i)(0), 0.0, results.xhat.col(i)(0));
                T_updated.row(1) << xhat.col(trans_size + i)(1), xhat.col(trans_size + 1 + i)(1), 0.0, xhat.col(i)(1);
                // RCLCPP_INFO(rclcpp::get_logger("message"), "T 2: %f, %f, %f, %f", results.xhat.col((trans_size-1) + i)(1), results.xhat.col(trans_size + i)(1), 0.0, results.xhat.col(i)(1));
                T_updated.row(2) << 0.0, 0.0, 1.0, 0.0;
                T_updated.row(3) << 0.0, 0.0, 0.0, 1.0;

                std::cout << "T0: " << std::endl;
                Eigen::Matrix<double, 4, 4> xhat_ = T_fix.matrix()*T_updated;

                x = xhat_(0,3); //xhat.col(i)(0);
                y = xhat_(1,3); //xhat.col(i)(1);

                //TODO maybe change if needed
                // Eigen::MatrixXd R = xhat.block(0, ((trans_size-1) + i), 2, 2);
                Eigen::Matrix<double, 2, 2> R = xhat_.block(0, 0, 2, 2);
                theta = img_transform::Rot2Theta(R);

                nodes.push_back({x, y, theta});
            }

            // update tf frame for after image recognition
            x_pos = x;
            y_pos = y;
            theta_pos = theta;


            // for (int i = 0; i < trans_size; i++){
            //     x = xhat.col(i)(0);
            //     y = xhat.col(i)(1);

            //     //TODO maybe change if needed
            //     Eigen::MatrixXd R = xhat.block(0, ((trans_size-1) + i), 2, 2);
            //     theta = img_transform::Rot2Theta(R);

            //     nodes.push_back({x, y, theta});
            // }

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();

            // MarkerArrayVec marker_vec = img_transform::visualize_graph(nodes, header);

            // geometry_msgs::msg::TransformStamped t;
            // RCLCPP_INFO(rclcpp::get_logger("message"), "nodes size: %d", nodes.size());
            // for (int i = 0; i < nodes.size(); i++){
            //     t.header.stamp = this->get_clock()->now();
            //     t.header.frame_id = "world";
            //     t.child_frame_id = "frame" + to_string(i);
            //     t.transform.translation.x = nodes[i][0];
            //     t.transform.translation.y = nodes[i][1];

            //     tf2::Quaternion q;
            //     q.setRPY(0, 0, nodes[i][2]);
            //     t.transform.rotation.x = q.x();
            //     t.transform.rotation.y = q.y();
            //     t.transform.rotation.z = q.z();
            //     t.transform.rotation.w = q.w();

            //     tf_broadcaster_->sendTransform(t);
            // }


            // visualization_msgs::msg::Marker edge_m;
            // edge_m.header.frame_id = "world";
            // edge_m.header.stamp = header.stamp;
            // edge_m.id = 0;
            // edge_m.action = visualization_msgs::msg::Marker::ADD;
            // edge_m.pose.orientation.w = 1.0;
            // edge_m.pose.position.x = 0.0;
            // edge_m.pose.position.y = 0.0;
            // edge_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            // edge_m.scale.x = 0.01;
            // edge_m.scale.y = 0.01;
            // edge_m.scale.z = 0.01;
            // edge_m.color.a = 1.0;
            // edge_m.color.r = 0.3;
            // edge_m.color.g = 0.9;
            // edge_m.color.b = 0.3;




            int id = 0;
            // TODO: change later when having loop back to num_edges.size instead of x positions
            // TODO: fix later for id's with loop closure (loop won't always mean back to 0,0 position)
            for (int i = 0; i < (nodes.size() - 1); i++){
                node_markers.markers[i].pose.position.x = nodes[i][0];
                node_markers.markers[i].pose.position.y = nodes[i][1];

                // edge_m.id = id;
                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0_: %f, %f", nodes[i][0], nodes[i][1]);
                edge_markers.markers[i].points[0].x = nodes[i][0];
                edge_markers.markers[i].points[0].y = nodes[i][1];
                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 1_: %f, %f", nodes[i+1][0], nodes[i+1][1]);
                RCLCPP_INFO(rclcpp::get_logger("message"), "i: %d", i);
                RCLCPP_INFO(rclcpp::get_logger("message"), "nodes size: %d", nodes.size());
                edge_markers.markers[i].points[1].x = nodes[i+1][0];
                edge_markers.markers[i].points[1].y = nodes[i+1][1];
                // edge_markers.markers.push_back(edge_m);
                id++;
            }

            double id_last = nodes.size() - 1;
            // edge_m.id = id;

            // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge last: %f, %f", nodes[id_last][0], nodes[id_last][1]);
            // edge_m.points.push_back(p);
            node_markers.markers[id_last].pose.position.x = nodes[id_last][0];
            node_markers.markers[id_last].pose.position.y = nodes[id_last][1];

            edge_markers.markers[id_last].points[0].x = nodes[id_last][0];
            edge_markers.markers[id_last].points[0].y = nodes[id_last][1];
            // // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0: %f, %f", nodes[0][0], nodes[0][1]);
            // // edge_m.points.push_back(p);
            edge_markers.markers[id_last].points[1].x = nodes[0][0];
            edge_markers.markers[id_last].points[1].y = nodes[0][1];

            // edge_markers.markers.push_back(edge_m);


            RCLCPP_INFO(rclcpp::get_logger("message"), "Publishing markers!");

            // pub_nodes_->publish(marker_vec[0]);
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
    // else{
        // visualization_msgs::msg::Marker node_m;
        // node_m.header.frame_id = "world";
        // node_m.header.stamp = header.stamp;
        // node_m.id = 0;
        // node_m.action = visualization_msgs::msg::Marker::ADD;
        // node_m.pose.orientation.w = 1.0;
        // node_m.pose.position.x = 0.0;
        // node_m.pose.position.y = 0.0;
        // node_m.type = visualization_msgs::msg::Marker::SPHERE;
        // node_m.scale.x = 0.03;
        // node_m.scale.y = 0.03;
        // node_m.scale.z = 0.03;
        // node_m.color.a = 1.0;
        // node_m.color.r = 0.9;
        // node_m.color.g = 0.3;
        // node_m.color.b = 0.3;

    //     int id = 0;
    //     // TODO: change later when having loop back to num_edges.size instead of x positions

    //     for (int i = 0; i < edges.size(); i++){

    //     }

    //     // for (int i = 0; i < (edges.size() - 1); i++){
    //     //     edge_m.id = id;
    //     //     geometry_msgs::msg::Point p;
    //     //     p.x = nodes[i][0];
    //     //     p.y = nodes[i][1];
    //     //     edge_m.points.push_back(p);
    //     //     p.x = nodes[i+1][0];
    //     //     p.y = nodes[i+1][1];
    //     //     edge_m.points.push_back(p);
    //     //     edge_markers.markers.push_back(edge_m);
    //     //     id++;
    //     // }
    // }
  }

  void odom_callback(
    const img_transform::msg::Odom::ConstSharedPtr& msg
  ){
    theta_pos = msg->theta;
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
    r_i_updated.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2);
    r_i_updated.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2);
    r_i_updated.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2);
    t_i_updated(0) = msg->row_1.at(3);
    t_i_updated(1) = msg->row_2.at(3);
    t_i_updated(2) = msg->row_3.at(3);

    // r_i_updated.row(0) << 1.0, 0.0, 0.0;
    // r_i_updated.row(1) << 0.0, 1.0, 0.0;
    // r_i_updated.row(2) << 0.0, 0.0, 1.0;
    // t_i_updated(0) = 0.0;
    // t_i_updated(1) = 0.0;
    // t_i_updated(2) = 0.0;

    // img_transform::Vector2D trans;
    // trans = img_transform::trans_to_vec(image_transformation);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 3, 3> information_matrix;
    information_matrix.row(0) << I11, I12, I13;
    information_matrix.row(1) << I21, I22, I23;
    information_matrix.row(2) << I31, I32, I33;

    img_transform::Edge edge;
    edge.id_a = id_trans;
    edge.id_b = 0;
    edge.rot = r_i_updated;
    edge.trans = t_i_updated;
    // edge.transform = trans;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    visualization_msgs::msg::Marker edge_i;
    edge_i.header.frame_id = "world";
    edge_i.header.stamp = header.stamp;
    edge_i.id = id_trans + 1;
    edge_i.action = visualization_msgs::msg::Marker::ADD;
    edge_i.pose.orientation.w = 1.0;
    edge_i.pose.position.x = 0.0;
    edge_i.pose.position.y = 0.0;
    edge_i.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_i.scale.x = 0.01;
    edge_i.scale.y = 0.01;
    edge_i.scale.z = 0.01;
    edge_i.color.a = 1.0;
    edge_i.color.r = 0.3;
    edge_i.color.g = 0.3;
    edge_i.color.b = 0.9;
    geometry_msgs::msg::Point p;
    p.x = edge_markers.markers[id_trans - 1].points[1].x;
    p.y = edge_markers.markers[id_trans - 1].points[1].y;
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0_: %f, %f", nodes[i][0], nodes[i][1]);
    edge_i.points.push_back(p);
    p.x = edge_markers.markers[0].points[0].x;  // TODO: change later to be position of id from image recognition
    p.y = edge_markers.markers[0].points[0].y;  // TODO: change later to be position of id from image recognition
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 1_: %f, %f", nodes[i+1][0], nodes[i+1][1]);
    edge_i.points.push_back(p);
    edge_markers.markers.push_back(edge_i);

    // pub_nodes_->publish(node_markers);
    pub_edges_->publish(edge_markers);

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
  }

  void wheel_transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
    // if (done == false){
        RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
        id_trans = msg->id;
        Eigen::Matrix<double, 4, 4> wheel_transformation;
        r_w_updated.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2);
        r_w_updated.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2);
        r_w_updated.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2);
        t_w_updated(0) = msg->row_1.at(3);
        t_w_updated(1) = msg->row_2.at(3);
        t_w_updated(2) = msg->row_3.at(3);

        prev_w_x = msg->x.at(0);
        prev_w_y = msg->y.at(0);
        x_pos = msg->x.at(1);
        y_pos = msg->y.at(1);
        theta_pos = msg->theta;

        RCLCPP_INFO(rclcpp::get_logger("message"), "Prev x and y: %f, %f", prev_w_x, prev_w_y);
        RCLCPP_INFO(rclcpp::get_logger("message"), "Current x and y: %f, %f", x_pos, y_pos);

        // img_transform::Vector2D trans;
        // trans = img_transform::trans_to_vec(wheel_transformation);

        // RCLCPP_INFO(rclcpp::get_logger("message"), "t_x: %f", trans.x);
        // RCLCPP_INFO(rclcpp::get_logger("message"), "t_y: %f", trans.y);
        // RCLCPP_INFO(rclcpp::get_logger("message"), "t_theta: %f", trans.theta);
        // RCLCPP_INFO(rclcpp::get_logger("message"), "id a: %d", id_trans-1);
        RCLCPP_INFO(rclcpp::get_logger("message"), "id: %d", id_trans);


        // Identity matrix times penalty weight
        Eigen::Matrix<double, 3, 3> information_matrix;
        information_matrix.row(0) << 0.001*I11, 0.001*I12, 0.001*I13;
        information_matrix.row(1) << 0.001*I21, 0.001*I22, 0.001*I23;
        information_matrix.row(2) << 0.001*I31, 0.001*I32, 0.0001*I33;


        img_transform::Edge edge;
        edge.id_a = id_trans - 1;
        edge.id_b = id_trans;
        // edge.transform = trans;
        edge.rot = r_w_updated;
        edge.trans = t_w_updated;
        edge.type = "EDGE_SE2";
        edge.info_matrix = information_matrix;
        edges.push_back(edge);

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();

        if (first){
            visualization_msgs::msg::Marker node_w;
            node_w.header.frame_id = "world";
            node_w.header.stamp = header.stamp;
            node_w.id = 0;
            node_w.ns = "nodes";
            node_w.action = visualization_msgs::msg::Marker::ADD;
            node_w.pose.orientation.w = 1.0;
            node_w.pose.position.x = prev_w_x;
            node_w.pose.position.y = prev_w_y;
            node_w.type = visualization_msgs::msg::Marker::SPHERE;
            node_w.scale.x = 0.03;
            node_w.scale.y = 0.03;
            node_w.scale.z = 0.03;
            node_w.color.a = 1.0;
            node_w.color.r = 0.9;
            node_w.color.g = 0.3;
            node_w.color.b = 0.3;
            node_markers.markers.push_back(node_w);

            first = false;
        }

        visualization_msgs::msg::Marker node_w;
        node_w.header.frame_id = "world";
        node_w.header.stamp = header.stamp;
        node_w.id = id_trans;
        node_w.ns = "nodes";
        node_w.action = visualization_msgs::msg::Marker::ADD;
        node_w.pose.orientation.w = 1.0;
        node_w.pose.position.x = x_pos;
        node_w.pose.position.y = y_pos;
        node_w.type = visualization_msgs::msg::Marker::SPHERE;
        node_w.scale.x = 0.03;
        node_w.scale.y = 0.03;
        node_w.scale.z = 0.03;
        node_w.color.a = 1.0;
        node_w.color.r = 0.9;
        node_w.color.g = 0.3;
        node_w.color.b = 0.3;
        node_markers.markers.push_back(node_w);

        visualization_msgs::msg::Marker edge_w;
        edge_w.header.frame_id = "world";
        edge_w.header.stamp = header.stamp;
        edge_w.id = id_trans;
        edge_w.ns = "edges";
        edge_w.action = visualization_msgs::msg::Marker::ADD;
        edge_w.pose.orientation.w = 1.0;
        edge_w.pose.position.x = 0.0;
        edge_w.pose.position.y = 0.0;
        edge_w.type = visualization_msgs::msg::Marker::LINE_STRIP;
        edge_w.scale.x = 0.01;
        edge_w.scale.y = 0.01;
        edge_w.scale.z = 0.01;
        edge_w.color.a = 1.0;
        edge_w.color.r = 0.3;
        edge_w.color.g = 0.9;
        edge_w.color.b = 0.3;
        geometry_msgs::msg::Point p;
        p.x = prev_w_x;
        p.y = prev_w_y;
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0_: %f, %f", nodes[i][0], nodes[i][1]);
        edge_w.points.push_back(p);
        p.x = x_pos;
        p.y = y_pos;
        RCLCPP_INFO(rclcpp::get_logger("message"), "x_pos y_pos: %f, %f", x_pos, y_pos);
        edge_w.points.push_back(p);
        edge_markers.markers.push_back(edge_w);

        pub_nodes_->publish(node_markers);
        pub_edges_->publish(edge_markers);

    //     count++;

    //     RCLCPP_INFO(rclcpp::get_logger("message"), "Count: %d ", count);

    //     if (count > 3){
    //         RCLCPP_INFO(rclcpp::get_logger("message"), "Reconstruction");
    //         reconstruction = true;
    //         done = true;

    //         r_w_updated.row(0) << 1.0, 0.0, 0.0;
    //         r_w_updated.row(1) << 0.0, 1.0, 0.0;
    //         r_w_updated.row(2) << 0.0, 0.0, 1.0;
    //         t_w_updated(0) = 0.03;
    //         t_w_updated(1) = 0.03;
    //         t_w_updated(2) = 0.0;

    //         // img_transform::Vector2D trans;
    //         // trans = img_transform::trans_to_vec(image_transformation);
    //         // trans.x = 0.0;
    //         // trans.y = 0.0;
    //         // trans.theta = (2.0*M_PI);

    //         // RCLCPP_INFO(rclcpp::get_logger("message"), "t_x: %f", trans.x);
    //         // RCLCPP_INFO(rclcpp::get_logger("message"), "t_y: %f", trans.y);
    //         // RCLCPP_INFO(rclcpp::get_logger("message"), "t_theta: %f", trans.theta);
    //         RCLCPP_INFO(rclcpp::get_logger("message"), "id a: %d", id_trans);
    //         RCLCPP_INFO(rclcpp::get_logger("message"), "id b: %d", 0);

    //         // Identity matrix times penalty weight
    //         Eigen::Matrix<double, 3, 3> information_matrix;
    //         information_matrix.row(0) << I11, I12, I13;
    //         information_matrix.row(1) << I21, I22, I23;
    //         information_matrix.row(2) << I31, I32, I33;

    //         img_transform::Edge edge;
    //         edge.id_a = id_trans;
    //         edge.id_b = 0;
    //         // edge.transform = trans;
    //         edge.rot = r_w_updated;
    //         edge.trans = t_w_updated;
    //         edge.type = "EDGE_SE2";
    //         edge.info_matrix = information_matrix;
    //         edges.push_back(edge);


    //         visualization_msgs::msg::Marker edge_i;
    //         edge_i.header.frame_id = "world";
    //         edge_i.header.stamp = header.stamp;
    //         edge_i.id = id_trans+2;
    //         edge_i.ns = "loop_edges";
    //         edge_i.action = visualization_msgs::msg::Marker::ADD;
    //         edge_i.pose.orientation.w = 1.0;
    //         edge_i.pose.position.x = 0.0;
    //         edge_i.pose.position.y = 0.0;
    //         edge_i.type = visualization_msgs::msg::Marker::LINE_STRIP;
    //         edge_i.scale.x = 0.01;
    //         edge_i.scale.y = 0.01;
    //         edge_i.scale.z = 0.01;
    //         edge_i.color.a = 1.0;
    //         edge_i.color.r = 0.3;
    //         edge_i.color.g = 0.3;
    //         edge_i.color.b = 0.9;
    //         geometry_msgs::msg::Point p;
    //         p.x = edge_markers.markers[id_trans - 1].points[1].x;
    //         p.y = edge_markers.markers[id_trans - 1].points[1].y;
    //         // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0_: %f, %f", nodes[i][0], nodes[i][1]);
    //         edge_i.points.push_back(p);
    //         p.x = edge_markers.markers[0].points[0].x;  // TODO: change later to be position of id from image recognition
    //         p.y = edge_markers.markers[0].points[0].y;  // TODO: change later to be position of id from image recognition
    //         // RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 1_: %f, %f", nodes[i+1][0], nodes[i+1][1]);
    //         edge_i.points.push_back(p);
    //         edge_markers.markers.push_back(edge_i);

    //         pub_nodes_->publish(node_markers);
    //         pub_edges_->publish(edge_markers);
    //     }
    // }
    // else{
        
    // }
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());
  }

  double dx_cam;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_image_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_wheel_transform_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sesync_trigger_;
  rclcpp::Subscription<img_transform::msg::Odom>::SharedPtr sub_odom_;
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
  Scalar I21 = 0;
  Scalar I22 = 50;
  Scalar I23 = 0;
  Scalar I31 = 0;
  Scalar I32 = 0;
  Scalar I33 = 100;

  double prev_w_x = 0.0;
  double prev_w_y = 0.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta_pos = 0.0;


  Eigen::Matrix<double, 4, 1> rob_pos_vec;

  visualization_msgs::msg::MarkerArray node_markers;
  visualization_msgs::msg::MarkerArray edge_markers;

  bool reconstruction = false;

  int count = 0;

  bool done = false;

  bool first = true;

  Eigen::Matrix<double, 3, 3> r_w_updated;
  Eigen::Matrix<double, 3, 1> t_w_updated;
  Eigen::Matrix<double, 3, 3> r_i_updated;
  Eigen::Matrix<double, 3, 1> t_i_updated;

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