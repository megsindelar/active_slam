#include "rclcpp/rclcpp.hpp"
#include "img_transform/transform.hpp"
#include "img_transform/msg/odom.hpp"
#include "img_transform/msg/transform.hpp"
#include "img_transform/msg/nodes.hpp"
#include "img_transform/msg/frame_id.hpp"
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

    /// \brief subscriber to transform between two frames
    sub_image_transform_ = this->create_subscription<img_transform::msg::Transform>(
      "/image_transform", 10, std::bind(
        &SE_SYNC::image_transform_callback,
        this, std::placeholders::_1));

    /// \brief subscriber to transforms from wheel odom
    sub_wheel_transform_ = this->create_subscription<img_transform::msg::Transform>(
      "/wheel_transform", 10, std::bind(
        &SE_SYNC::wheel_transform_callback,
        this, std::placeholders::_1));

    /// \brief publish red spheres as markers for nodes
    pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nodes", 10);

    /// \brief publish line as markers for edges
    pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "edges", 10);

    // pub_id_vals_ = this->create_publisher<img_transform::msg::Nodes>(
    //   "id_vals", 10);

    /// \brief publish position of node to loop back to for active learning
    pub_loop_back_ = this->create_publisher<geometry_msgs::msg::Pose>("loop_back", 10);

    /// \brief subscriber to odom orientation
    sub_odom_ = this->create_subscription<img_transform::msg::Odom>(
      "/odom_orientation", 10, std::bind(
        &SE_SYNC::odom_callback,
        this, std::placeholders::_1));

    /// \brief tf2 broadcaster to publish real-time robot position
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// \brief publish frameId from odom to registration
    pub_frame_id_ = this->create_publisher<img_transform::msg::FrameID>(
        "frame_id", 10);

    /// \brief publish odom update from image registration and se-sync
    pub_odom_update_ = this->create_publisher<img_transform::msg::Odom>(
      "odom_update", 10);
  }

private:
  /// \brief timer callback running at a set frequency
  // publish line as markers for edges
  // topic: /edges   type: visualization_msgs::msg::MarkerArray
  // publish red spheres as markers for nodes
  // topic: /nodes   type: visualization_msgs::msg::MarkerArray
  void timer_callback()
  {
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

    // double theta_w = (M_PI/2.0) - theta_pos;

    // geometry_msgs::msg::TransformStamped t_cam;
    // t_cam.header.stamp = this->get_clock()->now();
    // t_cam.header.frame_id = "world";
    // t_cam.child_frame_id = "wheel_base_frame";
    // t_cam.transform.translation.x = x_pos - dx_cam*sin(theta_pos);
    // t_cam.transform.translation.y = y_pos - dx_cam*cos(theta_pos);

    // tf2::Quaternion q_cam;
    // q_cam.setRPY(0, 0, theta_pos);
    // t_cam.transform.rotation.x = q_cam.x();
    // t_cam.transform.rotation.y = q_cam.y();
    // t_cam.transform.rotation.z = q_cam.z();
    // t_cam.transform.rotation.w = q_cam.w();

    // tf_broadcaster_->sendTransform(t_cam);


    // initialize no odometry update from registration
    img_transform::msg::Odom pos_update;
    pos_update.update = false;


    if (registration)
    {
        // Transform for rotating the output of SE-Sync to same quadrant as robot
        Eigen::Matrix<double, 3, 3> rot_;
        Eigen::Matrix<double, 3, 1> trans_;
        rot_.row(0) << cos(M_PI), -sin(M_PI), 0.0;
        rot_.row(1) << sin(M_PI), cos(M_PI), 0.0;
        rot_.row(2) << 0.0, 0.0, 1.0;
        trans_(0) = 0.0;
        trans_(1) = 0.0;
        trans_(2) = 0.0;

        Sophus::SE3d T_fix(rot_, trans_);


        registration = false;
        ////////////////////////////////////////////////////////////////////////////////////////////
        // SE_SYNC
        ////////////////////////////////////////////////////////////////////////////////////////////

        // Preallocate output vector
        measurements_t measurements;
        RCLCPP_INFO(rclcpp::get_logger("message"), "SESync starting");
        RelativePoseMeasurement measurement;


        RCLCPP_INFO(rclcpp::get_logger("message"), "Edges size: %ld", edges.size());

        // Fill in elements of measurement for SE-Sync
        for (int i = 0; i < edges.size(); i++){

            i_ = edges[i].id_a;
            j_ = edges[i].id_b;

            Eigen::Matrix<double, 3, 3> rotations = edges[i].rot;
            Eigen::Matrix<double, 3, 1> translations = edges[i].trans;

            Sophus::SE3 T_edge(rotations, translations);

            // fill in information matrix
            I11 = edges[i].info_matrix(0,0);
            I12 = edges[i].info_matrix(0,1);
            I13 = edges[i].info_matrix(0,2);
            I22 = edges[i].info_matrix(1,1);
            I23 = edges[i].info_matrix(1,2);
            I33 = edges[i].info_matrix(2,2);

            // Pose ids
            measurement.i = i_;
            measurement.j = j_;

            // Raw measurements
            measurement.t.resize(2);
            measurement.t = T_edge.translation().block(0,0,2,1);
            measurement.R.resize(2,2);
            measurement.R = T_edge.rotationMatrix().block(0,0,2,2);

            Eigen::Matrix<Scalar, 2, 2> TranInfo;
            TranInfo << I11, I12, I12, I22;
            measurement.tau = 2 / TranInfo.inverse().trace();

            measurement.kappa = I33;
            
            // Update maximum value of poses found so far
            size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

            measurements.push_back(measurement);
        }



        if (measurements.size() == 0) {
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

            // results from SE-Sync (form of [t | R])
            Eigen::MatrixXd xhat = results.xhat;

            int size_trans = 0;

            // figure out where translation ends and rotation begins in long 2 by x xhat matrix
            for (int i = 0; i < xhat.row(0).size() - 1; i++){
                if (abs(xhat.row(0)(i)) == abs(xhat.row(1)(i+1)) && abs(xhat.row(1)(i)) == abs(xhat.row(0)(i+1))){
                    if ((abs(xhat.row(0)(i)) + abs(xhat.row(0)(i+1)) + abs(xhat.row(1)(i)) + abs(xhat.row(1)(i+1)) > 0.0))
                        size_trans = i;
                        break;
                }
            }

            int translation_num = size_trans;

            std::vector<std::vector<double>> nodes;

            Eigen::Matrix<double, 4,4> T_updated;
            Eigen::Matrix<double, 3,3> r_updated;
            Eigen::Matrix<double, 3,1> t_updated;

            double x;
            double y;
            double theta;

            // create an anchor for the first point to remain in it's location
            r_updated.row(0) << xhat.col(translation_num)(0), xhat.col(translation_num + 1)(0), 0.0;
            r_updated.row(1) << xhat.col(translation_num)(1), xhat.col(translation_num + 1)(1), 0.0;
            r_updated.row(2) << 0.0, 0.0, 1.0;
            t_updated(0) = xhat.col(0)(0);
            t_updated(1) = xhat.col(0)(1);
            t_updated(1) = 0.0;

            Sophus::SE3d T_SE_0(r_updated, t_updated);

            Eigen::Vector<double,6> v_s(dx_cam, 0.0, 0.0, 0.0, 0.0, 0.0);
            Sophus::SE3d T_anchor = Sophus::SE3d::exp(v_s)*T_SE_0.inverse();


            // put xhat results in the form of (x, y, theta) nodess
            for (int i = 0; i < translation_num; i++){

                r_updated.block(0, 0, 2, 2) = xhat.block(0, translation_num + i*2, 2, 2);

                t_updated(0) = xhat.col(i)(0);
                t_updated(1) = xhat.col(i)(1);
                t_updated(2) = 0.0;

                Sophus::SE3d T_xhat_(r_updated, t_updated);

                Sophus::SE3d T_node = T_anchor*T_xhat_;
                x = T_node.translation()(0);
                y = T_node.translation()(1);

                Eigen::Matrix<double, 2, 2> R = T_node.rotationMatrix().block(0, 0, 2, 2);
                theta = img_transform::Rot2Theta(R);

                nodes.push_back({x, y, theta});
            }

            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();

            int id = 0;

            // update the markers with new node positions from SE-Sync
            for (int i = 0; i < (nodes.size() - 1); i++){
                node_markers.markers[i].pose.position.x = nodes[i][0];
                node_markers.markers[i].pose.position.y = nodes[i][1];

                edge_markers.markers[i].points[0].x = nodes[i][0];
                edge_markers.markers[i].points[0].y = nodes[i][1];

                edge_markers.markers[i].points[1].x = nodes[i+1][0];
                edge_markers.markers[i].points[1].y = nodes[i+1][1];

                pub_edges_->publish(edge_markers);

                id++;
            }

            // fill in all additional markers with last nodes until loops are added in
            int node_ind_fill = nodes.size() - 2;
            for (int i = nodes.size(); i < edge_markers.markers.size(); i++){
                node_markers.markers[i].pose.position.x = nodes[node_ind_fill][0];
                node_markers.markers[i].pose.position.y = nodes[node_ind_fill][1];

                edge_markers.markers[i].points[0].x = nodes[node_ind_fill][0];
                edge_markers.markers[i].points[0].y = nodes[node_ind_fill][1];

                edge_markers.markers[i].points[1].x = nodes[node_ind_fill+1][0];
                edge_markers.markers[i].points[1].y = nodes[node_ind_fill+1][1];
            }

            // loops fill in loops
            int loop_count = 1;

            // before adding loop, shift down nodes in marker list and add loop in corresponding spot
            for (int j = 0; j < loop_pairs.size(); j++){
                RCLCPP_INFO(rclcpp::get_logger("message"), "loop nums %d", loop_nums);
                RCLCPP_INFO(rclcpp::get_logger("message"), "loop_pairs %d", loop_pairs.size());
                RCLCPP_INFO(rclcpp::get_logger("message"), "node markers size %d", node_markers.markers.size());
                RCLCPP_INFO(rclcpp::get_logger("message"), "edge markers size %d", edge_markers.markers.size());

                // edge_m.points.push_back(p);
                int id1 = loop_pairs[j][0] + 1;
                int id2 = loop_pairs[j][1];
                int id1_m = id1;
                int id2_m = id2;


                int loop_counter = 0;
                if (loop_counter < loop_pairs.size()){
                    id1 = loop_pairs[j][0] + 1;
                    id1_m = id1 + loop_count - 1;
                    id2 = loop_pairs[j][1];


                    for (int p = id1; p < nodes.size()-1; p++){
                        node_markers.markers[id1_m+1].pose.position.x = nodes[p][0];
                        node_markers.markers[id1_m+1].pose.position.y = nodes[p][1];

                        edge_markers.markers[id1_m+1].points[0].x = nodes[p][0];
                        edge_markers.markers[id1_m+1].points[0].y = nodes[p][1];

                        edge_markers.markers[id1_m+1].points[1].x = nodes[p+1][0];
                        edge_markers.markers[id1_m+1].points[1].y = nodes[p+1][1];

                        edge_markers.markers[id1_m+1].color.r = 0.3;
                        edge_markers.markers[id1_m+1].color.g = 0.9;
                        edge_markers.markers[id1_m+1].color.b = 0.3;

                        id1_m++;
                    }
                }


                // add loop to marker list
                id1 = loop_pairs[j][0] + 1;
                id1_m = id1 + loop_count - 1;
                id2 = loop_pairs[j][1];

                RCLCPP_INFO(rclcpp::get_logger("message"), "id_1 and id_2: %d, %d", id_1, id_2);
                RCLCPP_INFO(rclcpp::get_logger("message"), "id1 and id2: %d, %d", id1, id2);
                RCLCPP_INFO(rclcpp::get_logger("message"), "id1_m: %d", id1_m);

                node_markers.markers[id1_m].pose.position.x = nodes[id1][0];
                node_markers.markers[id1_m].pose.position.y = nodes[id1][1];

                edge_markers.markers[id1_m].points[0].x = nodes[id1][0];
                edge_markers.markers[id1_m].points[0].y = nodes[id1][1];

                edge_markers.markers[id1_m].points[1].x = nodes[id2][0];
                edge_markers.markers[id1_m].points[1].y = nodes[id2][1];

                edge_markers.markers[id1_m].color.r = 0.3;
                edge_markers.markers[id1_m].color.g = 0.3;
                edge_markers.markers[id1_m].color.b = 0.9;

                int node_ind = node_markers.markers.size() - 1;
                int last_node = nodes.size() - 1;
                node_markers.markers[node_ind].pose.position.x = nodes[last_node][0];
                node_markers.markers[node_ind].pose.position.y = nodes[last_node][1];


                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 0_: %f, %f", nodes[id1][0], nodes[id1][1]);

                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge 1_: %f, %f", nodes[id2][0], nodes[id2][1]);
                loop_count++;
                loop_counter++;
            }


            for (int i = 0; i < edge_markers.markers.size(); i++){
                RCLCPP_INFO(rclcpp::get_logger("message"), "Marker id: %d", edge_markers.markers[i].id);
                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge Marker vals : %f, %f", edge_markers.markers[i].points[0].x, edge_markers.markers[i].points[0].y);
                RCLCPP_INFO(rclcpp::get_logger("message"), "Edge Marker vals : %f, %f", edge_markers.markers[i].points[1].x, edge_markers.markers[i].points[1].y);
            }

            // publish node and edge updates for graph
            pub_nodes_->publish(node_markers);
            pub_edges_->publish(edge_markers);

            // update position to be at loop closure
            int edge_ind = edge_markers.markers.size() - 1;
            pos_update.x = edge_markers.markers[edge_ind].points[1].x;
            pos_update.y = edge_markers.markers[edge_ind].points[1].y;
            pos_update.update = true;

            x_pos = edge_markers.markers[edge_ind].points[1].x;
            y_pos = edge_markers.markers[edge_ind].points[1].y;

            loop_nums++;

            RCLCPP_INFO(rclcpp::get_logger("message"), "Publishing markers!");
        }

    }

    pub_odom_update_->publish(pos_update);

  }

  // subscription to odom orientation
  // topic: /odom_orientation   type: img_transform::msg::Odom
  // publish line as markers for edges
  // topic: /edges   type: visualization_msgs::msg::MarkerArray
  // publish red spheres as markers for nodes
  // topic: /nodes   type: visualization_msgs::msg::MarkerArray
  void odom_callback(
    const img_transform::msg::Odom::ConstSharedPtr& msg
  ){
    theta_pos = msg->theta;
  }

  // subscription to transform between two frames
  // topic: /image_transform   type: img_transform::msg::Transform
  void image_transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Transform callback");
    r_i_updated.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2);
    r_i_updated.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2);
    r_i_updated.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2);
    t_i_updated(0) = msg->row_1.at(3);
    t_i_updated(1) = msg->row_2.at(3);
    t_i_updated(2) = msg->row_3.at(3);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 3, 3> information_matrix;
    information_matrix.row(0) << I11, I12, I13;
    information_matrix.row(1) << I21, I22, I23;
    information_matrix.row(2) << I31, I32, I33;


    // fill in edge information for SE-Sync
    img_transform::Edge edge;

    id_1 = msg->id;
    id_2 = msg->id_2;

    loop_pairs.push_back({id_1, id_2});

    edge.id_a = id_1 + 1;
    edge.id_b = id_2;
    RCLCPP_INFO(rclcpp::get_logger("message"), "ID 1: %d", id_1);
    RCLCPP_INFO(rclcpp::get_logger("message"), "ID 2: %d", id_2);

    edge.rot = r_i_updated;
    edge.trans = t_i_updated;
    // edge.transform = trans;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();


    // fill in markers
    visualization_msgs::msg::Marker node_i;
    node_i.header.frame_id = "world";
    node_i.header.stamp = header.stamp;
    node_i.id = id_1;
    node_i.ns = "nodes_im";
    node_i.action = visualization_msgs::msg::Marker::ADD;
    node_i.pose.orientation.w = 1.0;
    node_i.pose.position.x = edge_markers.markers[id_2].points[0].x;
    node_i.pose.position.y = edge_markers.markers[id_2].points[0].y;
    node_i.type = visualization_msgs::msg::Marker::SPHERE;
    node_i.scale.x = 0.03;
    node_i.scale.y = 0.03;
    node_i.scale.z = 0.03;
    node_i.color.a = 1.0;
    node_i.color.r = 0.9;
    node_i.color.g = 0.3;
    node_i.color.b = 0.3;
    node_markers.markers.push_back(node_i);

    visualization_msgs::msg::Marker edge_i;
    edge_i.header.frame_id = "world";
    edge_i.header.stamp = header.stamp;
    edge_i.id = id_1;
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
    p.x = edge_markers.markers[id_1].points[1].x;
    p.y = edge_markers.markers[id_1].points[1].y;
    edge_i.points.push_back(p);
    p.x = edge_markers.markers[id_2].points[0].x;
    p.y = edge_markers.markers[id_2].points[0].y;
    edge_i.points.push_back(p);
    edge_markers.markers.push_back(edge_i);

    // pub_nodes_->publish(node_markers);
    pub_edges_->publish(edge_markers);

    registration = true;

  }


  // subscription to transforms from wheel odom
  // topic: /wheel_transform   type: img_transform::msg::Odom
  // publish line as markers for edges
  // topic: /edges   type: visualization_msgs::msg::MarkerArray
  // publish red spheres as markers for nodes
  // topic: /nodes   type: visualization_msgs::msg::MarkerArray
  // publish position of node to loop back to for active learning
  // topic: /loop_back   type: geometry_msgs::msg::Pose

  void wheel_transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){

    id_trans = msg->id + 1;

    img_transform::msg::FrameID frame_id;
    frame_id.id = id_trans;

    pub_frame_id_->publish(frame_id);

    edge_id_vals.push_back(id_trans);

    Eigen::Matrix<double, 4, 4> wheel_transformation;
    r_w_updated.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2);
    r_w_updated.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2);
    r_w_updated.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2);
    t_w_updated(0) = msg->row_1.at(3);
    t_w_updated(1) = msg->row_2.at(3);
    t_w_updated(2) = msg->row_3.at(3);

    prev_x = msg->x.at(0);
    prev_y = msg->y.at(0);
    x_pos = msg->x.at(1);
    y_pos = msg->y.at(1);
    theta_pos = msg->theta;

    x_w_pos = msg->x_w;
    y_w_pos = msg->y_w;


    // Identity matrix times penalty weight
    // trust image registration more than wheel odom
    Eigen::Matrix<double, 3, 3> information_matrix;
    information_matrix.row(0) << 0.1*I11, 0.1*I12, 0.1*I13;
    information_matrix.row(1) << 0.1*I21, 0.1*I22, 0.1*I23;
    information_matrix.row(2) << 0.1*I31, 0.1*I32, 0.01*I33;


    // fill in edge information for SE-Sync
    img_transform::Edge edge;
    if (first_wheel_edge){
        edge.id_a = 0;
    }
    else{
        edge.id_a = edge_markers.markers[edge_markers.markers.size()-1].id;
    }
    first_wheel_edge = false;
    edge.id_b = id_trans;
    // edge.transform = trans;
    edge.rot = r_w_updated;
    edge.trans = t_w_updated;
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();

    // fill in marker information
    if (first){
        visualization_msgs::msg::Marker node_w;
        node_w.header.frame_id = "world";
        node_w.header.stamp = header.stamp;
        node_w.id = 0;
        node_w.ns = "nodes";
        node_w.action = visualization_msgs::msg::Marker::ADD;
        node_w.pose.orientation.w = 1.0;
        node_w.pose.position.x = prev_x;
        node_w.pose.position.y = prev_y;
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
    p.x = prev_x;
    p.y = prev_y;
    edge_w.points.push_back(p);
    p.x = x_pos;
    p.y = y_pos;
    edge_w.points.push_back(p);
    edge_markers.markers.push_back(edge_w);

    pub_nodes_->publish(node_markers);
    pub_edges_->publish(edge_markers);

    if (id_trans%20 == 0){
        geometry_msgs::msg::Pose loop_position;
        loop_position.position.x = node_markers.markers[id_trans - 20].pose.position.x;
        loop_position.position.y = node_markers.markers[id_trans - 20].pose.position.y;
        loop_position.orientation.z = 0.0;
        pub_loop_back_->publish(loop_position);
    }
  }


  double dx_cam;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_image_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_wheel_transform_;
  rclcpp::Subscription<img_transform::msg::Odom>::SharedPtr sub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nodes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;
//   rclcpp::Publisher<img_transform::msg::Nodes>::SharedPtr pub_id_vals_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_loop_back_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<img_transform::msg::FrameID>::SharedPtr pub_frame_id_;
  rclcpp::Publisher<img_transform::msg::Odom>::SharedPtr pub_odom_update_;

  img_transform::Vector2D transform;
  img_transform::Vector2D rob_state;
  std::vector<img_transform::Edge> edges;

  int id_trans = 0;
  int id_rob_state = 0;
  int a = 50;
  size_t num_poses = 0;
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

  double prev_x = 0.0;
  double prev_y = 0.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta_pos = 0.0;
  double x_w_pos = 0.0;
  double y_w_pos = 0.0;

  int id_1 = 0;
  int id_2 = 0;

  int loop_nums = 0;
  std::vector<std::vector<int>> loop_pairs;

  std::vector<int> edge_id_vals;

  visualization_msgs::msg::MarkerArray node_markers;
  visualization_msgs::msg::MarkerArray edge_markers;

  bool registration = false;

  bool first = true;
  bool first_wheel_edge = true;

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