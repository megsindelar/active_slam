#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>

#include <chrono>

#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include "matplotlibcpp.h"
#include <random>
#include <chrono>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <math.h>
// #include "img_transform/msg/keypoints.hpp"
#include "img_transform/msg/transform.hpp"

#include "sophus/se3.hpp"
#include "img_transform/transform.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/compressed_image.hpp"
#include "img_transform/msg/transform.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/point.hpp>

#include "DBoW2/DBoW2.h"

#include "img_transform/msg/nodes.hpp"
#include "img_transform/msg/frame_id.hpp"

using namespace std;
using namespace cv;
using namespace DBoW2;

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10


// function to create a vocabulary for bag of words
OrbVocabulary BoW_voc(const std::vector<std::vector<cv::Mat >>&features, int num_images);

// function to test frame against the vocabulary from bag of words
std::vector<int> BOW_test(const std::vector<std::vector<cv::Mat >>&features, OrbVocabulary voc, int num_images, std::vector<int> id_vals, double bow_threshold);

// function to add descriptors to feature vector
void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);

// function to calculate distance between two points
double calc_dist(double x1, double y1, double x2, double y2);


class Registration : public rclcpp::Node
{
    public:
    Registration()
    : Node("registration")
    {
        // custom QOS settings that can be used for image pub/sub
        size_t depth = 1;
        rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;

        // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        custom_camera_qos_profile.depth = depth;

        // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // ensure that every message gets received in order, or best effort, meaning that the transport
        // makes no guarantees about the order or reliability of delivery.
        custom_camera_qos_profile.reliability = reliability_policy;

        // The history policy determines how messages are saved until the message is taken by the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
        // parameter.
        custom_camera_qos_profile.history = history_policy;

        // subscribe to compressed image 
        sub_current_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/current_image/compressed", 10,
            std::bind(&Registration::image_callback, this, std::placeholders::_1)
        );

        // publish transforms for se_sync
        pub_transform_ = this->create_publisher<img_transform::msg::Transform>("image_transform", 10);

        // publish compressed image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        pub_keypoint_img_ = std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(
                this,
                "keypoint_image",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        // publish red spheres as markers for nodes
        pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "nodes_img", 10);

        // publish lines as markers for edges
        pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "edges_img", 10);

        // publish flag to add a node if see a lot of keypoints in a frame 
        pub_feature_transform_ = this->create_publisher<std_msgs::msg::Empty>(
            "feature_transform", 10);

        // publish flag to start moving the turtlebot
        pub_start_moving_ = this->create_publisher<std_msgs::msg::Empty>(
            "start_moving", 10);

        // subscribe to a status to add a frame with a node
        sub_frame_id_ = this->create_subscription<img_transform::msg::FrameID>(
            "/frame_id", 10, std::bind(
                &Registration::frame_id_callback,
                this, std::placeholders::_1));

        // subscribe to robot pose
        sub_rob_pose_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/rob_pose", 10, std::bind(
                &Registration::robot_pose,
                this, std::placeholders::_1));

        // sub_id_vals_ = this->create_subscription<img_transform::msg::Nodes>(
        //     "/id_vals", 10, std::bind(
        //         &Registration::id_vals_callback,
        //         this, std::placeholders::_1));

        // subscribe to odom transformation to get edges
        sub_wheel_transform_ = this->create_subscription<img_transform::msg::Transform>(
            "/wheel_transform", 10, std::bind(
                &Registration::wheel_transform_callback,
                this, std::placeholders::_1));


        // Timer
        declare_parameter("rate", 200);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&Registration::timer_callback, this));

        // service to trigger finding the transform between two images
        registration_srv = this->create_service<std_srvs::srv::Empty>(
        "/registration",
        std::bind(
            &Registration::reconstruct, this, std::placeholders::_1,
            std::placeholders::_2));

    }
    private:
        // function for TEASER 
        inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
            return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
        }

        // service to trigger finding the transform between two images
        // topic: /registration   type: std_srvs::srv::Empty
        void reconstruct(
            std_srvs::srv::Empty::Request::SharedPtr,
            std_srvs::srv::Empty::Response::SharedPtr)
        {
            reconstruct_img = true;
        }


        void timer_callback()
        {
            if (first){
                rotation_current.row(0) << 1.0, 0.0, 0.0;
                rotation_current.row(1) << 0.0, 1.0, 0.0;
                rotation_current.row(2) << 0.0, 0.0, 1.0;
                rotation_mat.row(0) << 1.0, 0.0, 0.0;
                rotation_mat.row(1) << 0.0, 1.0, 0.0;
                rotation_mat.row(2) << 0.0, 0.0, 1.0;
                translation_current(0) = 0.0;
                translation_current(1) = 0.0;
                translation_current(1) = 0.0;
                translation(0) = 0.0;
                translation(1) = 0.0;
                translation(2) = 0.0;
                Sophus::SE3d Transformation_prev(rotation_current, translation_current);
            }

            Eigen::Matrix<double, 4, 4> Transformation_current = Transformation_prev.matrix();
            Eigen::Matrix<double, 4, 4> Tran_pub = Eigen::Matrix4d::Identity();


            Mat descriptors;
            Ptr<FeatureDetector> detector = ORB::create();
            Ptr<DescriptorExtractor> descriptor = ORB::create();
            std::vector<KeyPoint> keypoints, keypoints_2;

            Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

            detector->detect ( registration_img, keypoints);

            descriptor->compute ( registration_img, keypoints, descriptors );

            if (keypoints.size() > 150){
                std_msgs::msg::Empty empty;
                pub_feature_transform_->publish(empty);
            }

            if (reconstruct_img){
            reconstruct_img = false;

            if (keypoints.size() > 3){

                descriptors_vec.push_back(descriptors);
                keypoints_vec.push_back(keypoints);

                features.push_back(std::vector<cv::Mat >());
                changeStructure(descriptors, features.back());
            }
            else{
                lost_f_ids++;
            }

            num_images = features.size();

            if (num_images > 3 && keypoints.size() > 3){

                OrbVocabulary voc = BoW_voc(features, num_images);

                std::vector<int> id_vals;
                int id_f = features.size() - 1;

                RCLCPP_INFO(rclcpp::get_logger("message"), "id_f %d", id_f);
                RCLCPP_INFO(rclcpp::get_logger("message"), "edge markers size %d", all_edge_markers.markers.size());
                RCLCPP_INFO(rclcpp::get_logger("message"), "features size %d", features.size());
                id_vals.push_back(id_f);
                double rad_circ = 0.25;

                for (int i = 0; i < features.size() - 1; i++){
                    double d_center = sqrt(pow(all_edge_markers.markers[i].points[1].x - all_edge_markers.markers[id_f].points[1].x, 2) + pow(all_edge_markers.markers[i].points[1].y - all_edge_markers.markers[id_f].points[1].y, 2));
                    RCLCPP_INFO(rclcpp::get_logger("message"), "i %d", i);
                    RCLCPP_INFO(rclcpp::get_logger("message"), "mark diff x, y %f, %f", (all_edge_markers.markers[i].points[1].x - all_edge_markers.markers[id_f].points[1].x), (all_edge_markers.markers[i].points[1].y - all_edge_markers.markers[id_f].points[1].y));
                    if (d_center < rad_circ && (id_f - i) > 9){
                        id_vals.push_back(i);
                    }
                }

                RCLCPP_INFO(rclcpp::get_logger("message"), "id_vals size %d", id_vals.size());

                std::vector<int> loop_candidates = BOW_test(features, voc, num_images, id_vals, bow_threshold);

                bool loop_verified = false;
                bool check_dist = false;
                bool check_matches = false;

                vector<DMatch> matches;

                int img_id = -1;

                RCLCPP_INFO(rclcpp::get_logger("message"), "loop candidates size %d", loop_candidates.size());

                std::vector<int> loop_options;
                std::vector<double> dist_lst;
                double dist = 0.0;

                int id_1 = 0;
                int id_2 = 0;

                for (int i = 0; i < loop_candidates.size(); i++){

                    img_id = loop_candidates[i];
                    RCLCPP_INFO(rclcpp::get_logger("message"), "img_id %d", img_id);

                    if (img_id == -1){
                        RCLCPP_INFO(rclcpp::get_logger("message"), "No BOW Matches!");
                        registration = false;
                        break;
                    }
                    else{
                        std_msgs::msg::Header header;
                        header.stamp = get_clock()->now();
                        cam_info_.header = header;

                        cv::Mat descriptors_2 = descriptors_vec[img_id];
                        keypoints_2 = keypoints_vec[img_id];

                        RCLCPP_INFO(rclcpp::get_logger("message"), "Keypoints size %d, %d", keypoints.size(), keypoints_2.size());

                        if (keypoints.size() > 3 && keypoints_2.size() > 3){

                            RCLCPP_INFO(rclcpp::get_logger("message"), "Matching");

                            try{
                                matcher->match ( descriptors, descriptors_2, matches );
                            }
                            catch (const std::exception & e) {
                                RCLCPP_ERROR_STREAM(
                                std::make_shared<Registration>()->get_logger(),
                                "Matching ERROR: Shutting down node! " << e.what());
                                rclcpp::shutdown();
                            }
                            double min_dist=10000, max_dist=0;

                            min_dist = 10.0;

                            // sort and then take a certain num of matches every time (use std sort)

                            RCLCPP_INFO(rclcpp::get_logger("message"), "Matches size %d", matches.size());
                            if (matches.size() > 11){
                                matched = true;
                                std::sort(matches.begin(), matches.end(), [](cv::DMatch& a, cv::DMatch& b) {
                                    return a.distance < b.distance;
                                });


                                RCLCPP_INFO(rclcpp::get_logger("message"), "Testing matches");

                                check_matches = true;
                                for (int i = 0; i < 5; i++){
                                    RCLCPP_INFO(rclcpp::get_logger("message"), "Matches distance: %f", matches[i].distance);
                                    if (matches[i].distance > 12){
                                        check_matches = false;
                                    }
                                }

                                // int id_1 = all_edge_markers.markers.size() - 1;
                                id_1 = features.size() + lost_f_ids - 1;
                                id_2 = img_id;
                                RCLCPP_INFO(rclcpp::get_logger("message"), "id_1 and id_2: %d, %d", id_1, id_2);

                                // if (features.size() > 30){
                                //     distance_threshold = 0.2;
                                // }

                                dist = calc_dist(all_edge_markers.markers[id_1].points[1].x, all_edge_markers.markers[id_1].points[1].y, all_edge_markers.markers[id_2].points[0].x, all_edge_markers.markers[id_2].points[0].y);
                                if (dist < distance_threshold){
                                    check_dist = true;
                                }
                                else{
                                    check_dist = false;
                                }
                            }


                            RCLCPP_INFO(rclcpp::get_logger("message"), "prev id_1 and id_2: %d, %d", prev_loop_pair[0], prev_loop_pair[1]);
                            if (check_matches && check_dist && (prev_loop_pair[0] != (id_1-1) && prev_loop_pair[1] != id_2)){
                                RCLCPP_INFO(rclcpp::get_logger("message"), "Potential loop!");
                                loop_verified = true;
                                registration = true;
                                loop_options.push_back(img_id);
                                dist_lst.push_back(dist);
                            }
                            matches.clear(); 
                        }
                    }
                }

                double min_dist = 100.0;
                for (int i = 0; i < loop_options.size(); i++){
                    min_dist = min(min_dist, dist_lst[i]);
                    if (min_dist == dist_lst[i]){
                        RCLCPP_INFO(rclcpp::get_logger("message"), "Min dist: %f", min_dist);
                        img_id = loop_options[i];
                    }
                }

                if (registration)
                {
                    prev_loop_pair[0] = id_1;
                    prev_loop_pair[1] = img_id;
                    registration = false;


                    if (loop_verified){

                        std::vector< DMatch > good_matches;

                        for (int i = 0; i < 12; i++)
                        {
                            good_matches.push_back(matches[i]);
                        }


                        float f_1 = 295.009696;
                        float f_2 = 296.22399483;
                        float p_1 = 195.515395;
                        float p_2 = 131.035055713;
                        std::vector<float> key1_xw;
                        std::vector<float> key1_yw;
                        std::vector<float> key1_zw;
                        std::vector<float> key2_xw;
                        std::vector<float> key2_yw;
                        std::vector<float> key2_zw;

                        // first get transform point by multiplying by camera matrix
                        int counter_matches = 0;
                        for (size_t i=0; i<good_matches.size(); i++){
                            float k1_xw = (z/f_1)*(keypoints.at(good_matches[i].queryIdx).pt.x - p_1*z);
                            float k1_yw = (z/f_2)*(keypoints.at(good_matches[i].queryIdx).pt.y - p_2*z);
                            float k2_xw = (z/f_1)*(keypoints_2.at(good_matches[i].trainIdx).pt.x - p_1*z);
                            float k2_yw = (z/f_2)*(keypoints_2.at(good_matches[i].trainIdx).pt.y - p_2*z);

                            key1_xw.push_back(k1_xw);
                            key1_yw.push_back(k1_yw);
                            key1_zw.push_back(z);
                            key2_xw.push_back(k2_xw);
                            key2_yw.push_back(k2_yw);
                            key2_zw.push_back(z);

                            counter_matches++;
                        }

                        // Convert the point cloud to Eigen
                        size_t N = good_matches.size();


                        Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
                        for (size_t i = 0; i < N; ++i) {
                            // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                            src.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
                            std::cout << "z1: " << key1_zw.at(i) << std::endl;
                        }

                        // Homogeneous coordinates
                        Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
                        src_h.resize(4, src.cols());
                        src_h.topRows(3) = src;
                        src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

                        // Apply an arbitrary SE(3) transformation
                        Eigen::Matrix4d T;

                        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
                        for (size_t i = 0; i < N; ++i) {
                            // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                            tgt.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
                            std::cout << "z2: " << key2_zw.at(i) << std::endl;
                        }

                        // Run TEASER++ registration
                        // Prepare solver parameters
                        teaser::RobustRegistrationSolver::Params params;
                        params.noise_bound = NOISE_BOUND;
                        params.cbar2 = 1;
                        params.estimate_scaling = false;
                        params.rotation_max_iterations = 100;
                        params.rotation_gnc_factor = 1.4;
                        params.rotation_estimation_algorithm =
                            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
                        params.rotation_cost_threshold = 0.005;

                        // Solve with TEASER++
                        teaser::RobustRegistrationSolver solver(params);
                        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                        solver.solve(src, tgt);
                        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                        auto solution = solver.getSolution();

                        // Compare results
                        std::cout << "=====================================" << std::endl;
                        std::cout << "          TEASER++ Results           " << std::endl;
                        std::cout << "=====================================" << std::endl;
                        std::cout << "Expected rotation: " << std::endl;
                        std::cout << T.topLeftCorner(3, 3) << std::endl;
                        std::cout << "Estimated rotation: " << std::endl;
                        std::cout << solution.rotation << std::endl;
                        std::cout << "Error (deg): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
                                    << std::endl;
                        std::cout << std::endl;
                        std::cout << "Expected translation: " << std::endl;
                        std::cout << T.topRightCorner(3, 1) << std::endl;
                        std::cout << "Estimated translation: " << std::endl;
                        std::cout << solution.translation << std::endl;
                        std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
                        std::cout << std::endl;
                        std::cout << "Number of correspondences: " << N << std::endl;
                        std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
                        std::cout << "Time taken (s): "
                                    << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                                        1000000.0
                                    << std::endl;


                        for (int i = 0; i < 3; i++){
                            rotation_mat.row(i) << solution.rotation(i,0), solution.rotation(i,1), solution.rotation(i,2);
                            translation(i) = solution.translation(i);
                        }

                        Sophus::SE3d Transformation(rotation_mat, translation);
                        Tran_pub = Transformation.matrix();

                        Sophus::SE3d Trans = Transformation_prev*Transformation;
                        Transformation_current = Trans.matrix();//Transformation_prev.matrix()*Transformation.matrix();
                        for (int i = 0; i < 3; i++){
                            rotation_current.row(i) << Transformation_current(i,0), Transformation_current(i,1), Transformation_current(i,2);
                            RCLCPP_INFO(rclcpp::get_logger("message"), "Rotation %f, %f, %f", Transformation_current(i,0), Transformation_current(i,1), Transformation_current(i,2));
                            translation_current(i) = Transformation_current(i,3);
                            RCLCPP_INFO(rclcpp::get_logger("message"), "Translation %f", Transformation_current(i,3));
                        }
                        Transformation_prev = Trans;
                        Eigen::Matrix<double, 4, 1>  vec_mark;
                        vec_mark[0] = rob_x;
                        vec_mark[1] = rob_y;
                        vec_mark[2] = 0.0;
                        vec_mark[3] = 1.0;
                        Eigen::Matrix<double, 4, 1> vec_new_mark;
                        vec_new_mark = Trans.inverse()*vec_mark;

                        visualization_msgs::msg::Marker node;
                        node.header.frame_id = "world";
                        node.header.stamp = this->get_clock()->now();
                        node.action = visualization_msgs::msg::Marker::ADD;
                        node.pose.orientation.w = 1.0;
                        node.type = visualization_msgs::msg::Marker::SPHERE;
                        node.scale.x = 0.05;
                        node.scale.y = 0.05;
                        node.scale.z = 0.05;
                        node.color.a = 1.0;
                        node.color.r = 0.3;
                        node.color.g = 0.9;
                        node.color.b = 0.3;

                        visualization_msgs::msg::Marker edge_m;
                        edge_m.header.frame_id = "world";
                        edge_m.header.stamp = this->get_clock()->now();
                        edge_m.action = visualization_msgs::msg::Marker::ADD;
                        edge_m.pose.orientation.w = 1.0;
                        edge_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
                        edge_m.scale.x = 0.03;
                        edge_m.scale.y = 0.03;
                        edge_m.scale.z = 0.03;
                        edge_m.color.a = 1.0;
                        edge_m.color.r = 0.3;
                        edge_m.color.g = 0.9;
                        edge_m.color.b = 0.3;


                        node.header.stamp = this->get_clock()->now();
                        edge_m.header.stamp = this->get_clock()->now();

                        node.id = id;
                        node.pose.position.x = vec_new_mark[0];
                        node.pose.position.y = vec_new_mark[1];
                        node_markers.markers.push_back(node);
                        // pub_nodes_->publish(node_markers);

                        edge_m.id = id;
                        geometry_msgs::msg::Point p;
                        p.x = rob_x;
                        p.y = rob_y;
                        edge_m.points.push_back(p);
                        p.x = vec_new_mark[0];
                        p.y = vec_new_mark[1];
                        edge_m.points.push_back(p);
                        edge_markers.markers.push_back(edge_m);
                        // pub_edges_->publish(edge_markers);
                        id++;


                        // publish transform for se_sync
                        img_transform::msg::Transform T_01;
                        // img_transform::msg::Transform rob_state;
                        T_01.id = features.size() + lost_f_ids - 2;
                        T_01.id_2 = img_id;
                        // rob_state.id = id;
                        for (int i = 0; i < 4; i++)
                        {
                            T_01.row_1.push_back((Tran_pub.matrix())(0,i));
                            T_01.row_2.push_back((Tran_pub.matrix())(1,i));
                            T_01.row_3.push_back((Tran_pub.matrix())(2,i));
                            T_01.row_4.push_back((Tran_pub.matrix())(3,i));

                        }

                        pub_transform_->publish(T_01);
                    }
                }
            }
        }
        }

        // subscribe to robot pose
        // topic: /rob_pose   type: geometry_msgs::msg::Point
        void robot_pose(const geometry_msgs::msg::Point::ConstSharedPtr& msg){
            rob_x = msg->x;
            rob_y = msg->y;
        }

        // subscribe to a status to add a frame with a node
        // topic: /frame_id   type: img_transform::msg::FrameID
        void frame_id_callback(
            const img_transform::msg::FrameID::ConstSharedPtr& msg
        ){
            // num_images = msg->id;
            take_image = true;
        }

        // subscribe to compressed image 
        // topic: /current_image/compressed   type: sensor_msgs::msg::CompressedImage
        void image_callback(
            const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg
        ){
            registration_img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;

            if (take_image){
                
                RCLCPP_INFO(rclcpp::get_logger("message"), "Taking image");
                reconstruct_img = true;
                take_image = false;
                if (!moving){
                    std_msgs::msg::Empty empty;
                    pub_start_moving_->publish(empty);
                    moving = true;
                }
            }

        }

        // void id_vals_callback(const img_transform::msg::Nodes::ConstSharedPtr& msg){
        //     id_vals = msg->edge_ids;
        // }


        // subscribe to odom transformation to get edges
        // topic: /wheel_transform   type: img_transform::msg::Transform
         void wheel_transform_callback(
            const img_transform::msg::Transform::ConstSharedPtr& msg
        ){
            int id_trans = msg->id + 1;

            double prev_x = msg->x.at(0);
            double prev_y = msg->y.at(0);
            double x_pos = msg->x.at(1);
            double y_pos = msg->y.at(1);

            if (first){
                all_edge_markers = visualization_msgs::msg::MarkerArray();

                visualization_msgs::msg::Marker edge;
                geometry_msgs::msg::Point p;
                p.x = prev_x;
                p.y = prev_y;
                edge.points.push_back(p);
                p.x = prev_x;
                p.y = prev_y;

                edge.id = id_trans;
                edge.points.push_back(p);
                all_edge_markers.markers.push_back(edge);
                first = false;
            }

            visualization_msgs::msg::Marker edge;
            geometry_msgs::msg::Point p;
            p.x = prev_x;
            p.y = prev_y;
            edge.points.push_back(p);
            p.x = x_pos;
            p.y = y_pos;

            edge.id = id_trans;
            edge.points.push_back(p);
            all_edge_markers.markers.push_back(edge);

            int ind_ed = all_edge_markers.markers.size() - 1;

            RCLCPP_INFO(rclcpp::get_logger("message"), "Edge_0: %f, %f", all_edge_markers.markers[ind_ed].points[0].x, all_edge_markers.markers[ind_ed].points[0].y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "Edge_1: %f, %f", all_edge_markers.markers[ind_ed].points[1].x, all_edge_markers.markers[ind_ed].points[1].y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "all_edge_markers size: %d", all_edge_markers.markers.size());

        }



        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_current_img_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        sensor_msgs::msg::CameraInfo cam_info_;
        std::shared_ptr<image_transport::CameraPublisher> pub_keypoint_img_;
        rclcpp::Publisher<img_transform::msg::Transform>::SharedPtr pub_transform_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr registration_srv;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nodes_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_feature_transform_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_start_moving_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_rob_pose_;
        // rclcpp::Subscription<img_transform::msg::Nodes>::SharedPtr sub_id_vals_;
        rclcpp::Subscription<img_transform::msg::FrameID>::SharedPtr sub_frame_id_;
        rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_wheel_transform_;

        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        Mat img1;
        Mat registration_img;
        std::string child_frame = "current";
        std::string parent_frame = "previous";
        bool first = true;
        tf2::Quaternion q;
        int count = 0;
        bool matched = true;
        float z = 0.095;

        Sophus::SE3d Transformation_prev;
        Sophus::SE3d Trans;
        Sophus::SE3d Transformation;
        Eigen::Matrix<double, 3, 3> rotation_mat;
        Eigen::Matrix<double, 3, 3> rotation_current;
        Eigen::VectorXd translation = Eigen::VectorXd(3);
        Eigen::VectorXd translation_current = Eigen::VectorXd(3);
        int id = 1;

        visualization_msgs::msg::MarkerArray node_markers;
        visualization_msgs::msg::MarkerArray edge_markers;
        visualization_msgs::msg::MarkerArray all_edge_markers;

        std::vector<int> prev_loop_pair {-1, -1};

        bool first_img = true;
        bool reconstruct_img = false;
        bool registration = false;

        double distance_threshold = 0.15;

        bool moving = false;

        double rob_x = 0.0;
        double rob_y = 0.0;

        int lost_f_ids = 0;

        double bow_threshold = 0.14;

        std::vector<std::vector<cv::Mat>> features;
        std::vector<std::vector<KeyPoint>> keypoints_vec;
        std::vector<cv::Mat> descriptors_vec;

        long int num_images = 1;

        bool take_image = true;

};


// function to create a vocabulary for bag of words
OrbVocabulary BoW_voc(const std::vector<std::vector<cv::Mat >>&features, int num_images){
    int k = 9;
    int L = 3;
    WeightingType weight = TF_IDF;
    ScoringType scoring = L1_NORM;

    // create vocabulary
    OrbVocabulary voc(k, L, weight, scoring);

    voc.create(features);

    BowVector v1, v2;
    for (int i = 0; i < num_images; i++){
        voc.transform(features[i], v1);
        for(int j = 0; j < num_images; j++)
        {
            voc.transform(features[j], v2);
            double score = voc.score(v1, v2);
        }
    }

    return voc;
}



// function to test frame against the vocabulary from bag of words
std::vector<int> BOW_test(const std::vector<std::vector<cv::Mat >>&features, OrbVocabulary voc, int num_images, std::vector<int> id_vals, double bow_threshold){
    // create a database
    OrbDatabase db(voc, false, 0);

    // check the nodes in a circle area around the robot
    for(int i = 0; i < id_vals.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("message"), "Id vals: %d", id_vals[i]);
        db.add(features[id_vals[i]]);
    }

    QueryResults ret;

    std::vector<int> loop_candidates;

    int curr_img = id_vals[0];
    RCLCPP_INFO(rclcpp::get_logger("message"), "Searching Image: %d", curr_img);
    db.query(features[curr_img], ret, 4);
    for (int j = 1; j < id_vals.size(); j++){
        RCLCPP_INFO(rclcpp::get_logger("message"), "Query Results: %f", ret[j].Score);
        RCLCPP_INFO(rclcpp::get_logger("message"), "Image Match ID: %d", ret[j].Id);
        RCLCPP_INFO(rclcpp::get_logger("message"), "j: %d", j);

        if (abs(ret[j].Id) < id_vals[0]){
            // arbitrary threshold (really depends on how many different images there are to compare against)
            if (ret[j] > bow_threshold){
                // another condition to determine if the detected loop is real
                if (abs(curr_img - id_vals[ret[j].Id]) > 9 && ret[j].Score > 0.0 && ret[j].Score < 1.0 ){
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Current Image: %d", curr_img);
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Image Match ID: %d", ret[j].Id);
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Image Match ID diff: %d", curr_img - ret[j].Id);

                    loop_candidates.push_back(id_vals[ret[j].Id]);
                }
            }
        }

        }
    // }
    return loop_candidates;

}

// function to calculate distance between two points
double calc_dist(double x1, double y1, double x2, double y2){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Poses x,y: (%f, %f), (%f, %f)", x1, y1, x2, y2);
    double dist = sqrt((pow((x2-x1),2) + pow((y2-y1), 2)));

    RCLCPP_INFO(rclcpp::get_logger("message"), "Distance: %f", dist);
    return dist;
}

// function to add descriptors to feature vector
void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Registration>());
    rclcpp::shutdown();
    return 0;
}