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
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

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
#include <cmath>

#include "sophus/se3.hpp"
#include "img_transform/transform.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/compressed_image.hpp"

#include "img_transform/msg/transform.hpp"
#include "img_transform/msg/nodes.hpp"
#include "img_transform/msg/frame_id.hpp"
#include "img_transform/msg/waypoint.hpp"
#include "img_transform/msg/odom.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/point.hpp>

#include "DBoW2/DBoW2.h"

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

        // publish flag to add a node if see a lot of keypoints in a frame 
        pub_feature_transform_ = this->create_publisher<std_msgs::msg::Empty>(
            "feature_transform", 10);

        // publish flag to start moving the turtlebot
        pub_start_moving_ = this->create_publisher<std_msgs::msg::Empty>(
            "start_moving", 10);

        pub_loop_closure_ = this->create_publisher<std_msgs::msg::Empty>(
            "loop_closure", 10);

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

        // subscribe to visual search complete status
        sub_search_done_ = this->create_subscription<std_msgs::msg::Empty>(
            "/search_done", 10, std::bind(
                &Registration::search_done_callback,
                this, std::placeholders::_1));

        // subscribe to nodes marker list for poster visual reconstruction
        sub_poster_data_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/nodes", 10, std::bind(
                &Registration::poster_data_callback,
                this, std::placeholders::_1));

        // subscribe to odom transformation to get edges
        sub_wheel_transform_ = this->create_subscription<img_transform::msg::Transform>(
            "/wheel_transform", 10, std::bind(
                &Registration::wheel_transform_callback,
                this, std::placeholders::_1));

        // subscribe to waypoints for visual search
        sub_search_waypoint_ = this->create_subscription<img_transform::msg::Waypoint>(
            "/search_waypoint", 10, std::bind(
                &Registration::search_waypoint_callback,
                this, std::placeholders::_1));

        // subscribe to waypoints for non-visual search movements
        sub_waypoint_status_ = this->create_subscription<std_msgs::msg::Empty>(
            "/waypoint_complete", 10, std::bind(
                &Registration::waypoint_status,
                this, std::placeholders::_1));

        // subscribe to status to trigger looping sequence
        sub_loop_back_ = this->create_subscription<img_transform::msg::Waypoint>(
            "/loop_back", 10, std::bind(
                &Registration::loop_back_callback,
                this, std::placeholders::_1));


        /// \brief subscriber to odom orientation
        sub_odom_ = this->create_subscription<img_transform::msg::Odom>(
            "/odom_orientation", 10, std::bind(
                &Registration::odom_callback,
                this, std::placeholders::_1));

        // subscribe to loop completed status
        sub_finish_loop_ = this->create_subscription<std_msgs::msg::Empty>(
            "/finish_loop", 10, std::bind(
                &Registration::finish_loop_callback,
                this, std::placeholders::_1));

        // subscribe a status to trigger poster reconstruction before updating from SESync
        sub_before_update_ = this->create_subscription<std_msgs::msg::Empty>(
            "/poster_before_update", 10, std::bind(
                &Registration::poster_before_callback,
                this, std::placeholders::_1));

        // Timer
        declare_parameter("rate", 200);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&Registration::timer_callback, this));

        // service to trigger finding the transform between two images
        registration_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/registration",
        std::bind(
            &Registration::registration_srv, this, std::placeholders::_1,
            std::placeholders::_2));

        // service to trigger poster reconstruction
        reconstruction_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/reconstruct_poster",
        std::bind(
            &Registration::reconstruct_srv, this, std::placeholders::_1,
            std::placeholders::_2));

    }
    private:
        // function for TEASER 
        inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
            return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
        }

        // service to trigger finding the transform between two images
        // topic: /registration   type: std_srvs::srv::Empty
        void registration_srv(
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

            if (update_done){
                std::string file = "cv_images_after" + to_string(loop_count) + ".ext";
                reconstruct_poster_(file);
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

            if (search && take_search_image){
                search_descriptions.push_back(descriptors);
                search_keypoints.push_back(keypoints);
                take_search_image = false;
            }

            if (keypoints.size() > 150){
                std_msgs::msg::Empty empty;
                pub_feature_transform_->publish(empty);
            }

            if (reconstruct_img){
            reconstruct_img = false;


            if (keypoints.size() > 3 && !search){

                descriptors_vec.push_back(descriptors);
                keypoints_vec.push_back(keypoints);

                features.push_back(std::vector<cv::Mat >());
                changeStructure(descriptors, features.back());
            }
            else{
                lost_f_ids++;
            }

            num_images = features.size();

            vector<DMatch> matches;


            if (search_done){
                // MSE test

                mse_vals.clear();

                for (int i = 0; i < search_images.size(); i++){
                    Mat MSE_img = search_images[i];

                    // rotate image to be same orientation as node
                    Mat rotated_img;
                    double angle = loop_img_orientation - image_angles[i];

                    int X = MSE_img.cols/2.;
                    int Y = MSE_img.rows/2.;
                    Point2f pt(X,Y);
                    Mat rotatedMat = getRotationMatrix2D(pt, angle, 1.0); 
                    warpAffine(MSE_img, rotated_img, rotatedMat, Size(MSE_img.cols, MSE_img.rows));

                    Mat resized_img;
                    cv::resize(rotated_img, resized_img, loop_image.size(), 0, 0, INTER_LINEAR);

                    // compute MSE
                    double sum = 0.0;
                    for (int r = 0; r < loop_image.rows; r++){
                        for (int c = 0; c < loop_image.cols; c++){
                            Vec3b intensity_loop = loop_image.at<Vec3b>(r, c);
                            double blue_loop = (double)intensity_loop.val[0];
                            double green_loop = (double)intensity_loop.val[1];
                            double red_loop = (double)intensity_loop.val[2];
                            Vec3b intensity_mse = resized_img.at<Vec3b>(r, c);
                            double blue_mse = (double)intensity_mse.val[0];
                            double green_mse = (double)intensity_mse.val[1];
                            double red_mse = (double)intensity_mse.val[2];
                            double diff = (double)((blue_loop - blue_mse) + (green_loop - green_mse) + (red_loop - red_mse));
                            sum += pow(diff, 2);
                        }
                    }

                    mse_vals.push_back(sum /(loop_image.rows*loop_image.cols));
                }

                // find min MSE val
                double min_mse_val = 100000000000000000000000.0;
                int min_mse_ind = 0;
                for (int i = 0; i < mse_vals.size(); i++){
                    min_mse_val = std::min(min_mse_val, mse_vals[i]);
                    if (min_mse_val == mse_vals[i]){
                        min_mse_ind = i;
                    }
                }

                if (min_mse_ind != -1){
                    Mat descriptors_search = search_descriptions[min_mse_ind];
                    keypoints_search = search_keypoints[min_mse_ind];

                    try{
                        matcher->match ( descriptors_loop, descriptors_search, matches );
                    }
                    catch (const std::exception & e) {
                        RCLCPP_ERROR_STREAM(
                        std::make_shared<Registration>()->get_logger(),
                        "Matching ERROR: Shutting down node! " << e.what());
                        rclcpp::shutdown();
                    }

                    if (matches.size() > 11){
                        // replace loop image with new mse image
                        changeStructure(descriptors_search, features[loop_id]);
                        // sort matches for best matches at the top
                        std::sort(matches.begin(), matches.end(), [](cv::DMatch& a, cv::DMatch& b) {
                            return a.distance < b.distance;
                        });

                        registration = true;
                        search_done = false;
                        loop_verified = true;
                        take_curr_loc_img = true;
                    }
                }
            }


            ////////////////////////////////////////////////////////////////////////
            // this is code from BOW, cv matcher, and odom distance tests
            ////////////////////////////////////////////////////////////////////////

            // if (num_images > 3 && keypoints.size() > 3){

                // OrbVocabulary voc = BoW_voc(features, num_images);

                // std::vector<int> id_vals;
                // int id_f = features.size() - 1;

                // id_vals.push_back(id_f);
                // double rad_circ = 0.25;

                // for (int i = 0; i < features.size() - 1; i++){
                //     double d_center = sqrt(pow(all_edge_markers.markers[i].points[1].x - all_edge_markers.markers[id_f].points[1].x, 2) + pow(all_edge_markers.markers[i].points[1].y - all_edge_markers.markers[id_f].points[1].y, 2));
                //     if (d_center < rad_circ && (id_f - i) > 9){
                //         id_vals.push_back(i);
                //     }
                // }

                // std::vector<int> loop_candidates = BOW_test(features, voc, num_images, id_vals, bow_threshold);

                // bool loop_verified = false;
                // bool check_dist = false;
                // bool check_matches = false;

                // vector<DMatch> matches;

                // int img_id = -1;

                // std::vector<int> loop_options;
                // std::vector<double> dist_lst;
                // double dist = 0.0;

                // int id_1 = 0;
                // int id_2 = 0;

                // for (int i = 0; i < loop_candidates.size(); i++){

                //     img_id = loop_candidates[i];

                //     if (img_id == -1){
                //         registration = false;
                //         break;
                //     }
                //     else{
                //         std_msgs::msg::Header header;
                //         header.stamp = get_clock()->now();
                //         cam_info_.header = header;

                //         cv::Mat descriptors_2 = descriptors_vec[img_id];
                //         keypoints_2 = keypoints_vec[img_id];

                //         if (keypoints.size() > 3 && keypoints_2.size() > 3){

                //             try{
                //                 matcher->match ( descriptors, descriptors_2, matches );
                //             }
                //             catch (const std::exception & e) {
                //                 RCLCPP_ERROR_STREAM(
                //                 std::make_shared<Registration>()->get_logger(),
                //                 "Matching ERROR: Shutting down node! " << e.what());
                //                 rclcpp::shutdown();
                //             }
                //             double min_dist=10000, max_dist=0;

                //             min_dist = 10.0;

                //             // sort and then take a certain num of matches every time (use std sort)

                //             if (matches.size() > 11){
                //                 matched = true;
                //                 std::sort(matches.begin(), matches.end(), [](cv::DMatch& a, cv::DMatch& b) {
                //                     return a.distance < b.distance;
                //                 });


                //                 // check_matches = true;
                //                 // for (int i = 0; i < 5; i++){
                //                 //     if (matches[i].distance > match_threshold){
                //                 //         check_matches = false;
                //                 //     }
                //                 // }

                //                 // int id_1 = all_edge_markers.markers.size() - 1;
                //                 id_1 = features.size() + lost_f_ids - 1;
                //                 id_2 = img_id;

                //                 // if (features.size() > 30){
                //                 //     distance_threshold = 0.2;
                //                 // }

                //                 dist = calc_dist(all_edge_markers.markers[id_1].points[1].x, all_edge_markers.markers[id_1].points[1].y, all_edge_markers.markers[id_2].points[0].x, all_edge_markers.markers[id_2].points[0].y);
                //                 if (dist < distance_threshold){
                //                     check_dist = true;
                //                 }
                //                 else{
                //                     check_dist = false;
                //                 }
                //             }


                //             if (check_matches && check_dist && (prev_loop_pair[0] != (id_1-1) && prev_loop_pair[1] != id_2)){
                //                 loop_verified = true;
                //                 registration = true;
                //                 loop_options.push_back(img_id);
                //                 dist_lst.push_back(dist);
                //             }
                //             matches.clear(); 
                //         }
                //     }
                // }

                // double min_dist = 100.0;
                // for (int i = 0; i < loop_options.size(); i++){
                //     min_dist = min(min_dist, dist_lst[i]);
                //     if (min_dist == dist_lst[i]){
                //         img_id = loop_options[i];
                //     }
                // }

                if (registration)
                {
                    // prev_loop_pair[0] = id_1;
                    // prev_loop_pair[1] = img_id;
                    registration = false;

                    if (loop_verified){
                        loop_verified = false;

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
                            float k1_xw = (z/f_1)*(keypoints_loop.at(good_matches[i].queryIdx).pt.x - p_1*z);
                            float k1_yw = (z/f_2)*(keypoints_loop.at(good_matches[i].queryIdx).pt.y - p_2*z);
                            float k2_xw = (z/f_1)*(keypoints_search.at(good_matches[i].trainIdx).pt.x - p_1*z);
                            float k2_yw = (z/f_2)*(keypoints_search.at(good_matches[i].trainIdx).pt.y - p_2*z);

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
                            translation_current(i) = Transformation_current(i,3);
                        }

                        Transformation_prev = Trans;
                        Eigen::Matrix<double, 4, 1>  vec_mark;
                        vec_mark[0] = rob_x;
                        vec_mark[1] = rob_y;
                        vec_mark[2] = 0.0;
                        vec_mark[3] = 1.0;
                        Eigen::Matrix<double, 4, 1> vec_new_mark;
                        vec_new_mark = Trans.inverse()*vec_mark;

                        id++;

                        // publish transform for se_sync
                        img_transform::msg::Transform T_01;
                        T_01.id = features.size() + lost_f_ids - 2;
                        T_01.id_2 = loop_id;
                        for (int i = 0; i < 4; i++)
                        {
                            T_01.row_1.push_back((Tran_pub.matrix())(0,i));
                            T_01.row_2.push_back((Tran_pub.matrix())(1,i));
                            T_01.row_3.push_back((Tran_pub.matrix())(2,i));
                            T_01.row_4.push_back((Tran_pub.matrix())(3,i));

                        }

                        pub_transform_->publish(T_01);
                        std_msgs::msg::Empty empty;
                        pub_loop_closure_->publish(empty);

                        loop_count++;
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

        // subscriber to odom orientation
        // topic: /odom_orientation   type: img_transform::msg::Odom
        void odom_callback(
            const img_transform::msg::Odom::ConstSharedPtr& msg
        ){
            theta_pos = msg->theta;
        }

        // subscribe to waypoints for visual search
        // topic: /search_waypoint   type: img_transform::msg::Waypoint
        void search_waypoint_callback(img_transform::msg::Waypoint::SharedPtr msg){
            search = msg->search;
        }

        // subscribe to status to trigger looping sequence
        // topic: /loop_back   type: img_transform::msg::Waypoint
        void loop_back_callback(img_transform::msg::Waypoint::SharedPtr msg){
            loop_id = msg->id;
            loop_image = edge_images[loop_id];
            loop_img_orientation = all_edge_markers.markers[loop_id].pose.orientation.z;
            descriptors_loop = descriptors_vec[loop_id];
            keypoints_loop = keypoints_vec[loop_id];
            search_descriptions.clear();
            search_keypoints.clear();
        }

        // subscribe to waypoints for non-visual search movements
        // topic: /waypoint_complete   type: std_msgs::msg::Empty
        void waypoint_status(std_msgs::msg::Empty::SharedPtr msg){
            next_waypoint = true;
        }

        // subscribe to visual search complete status
        // topic: /search_done   type: std_msgs::msg::Empty
        void search_done_callback(std_msgs::msg::Empty::SharedPtr msg){
            search_done = true;
            search = false;
            search_images.clear();
            reconstruct_img = true;
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

            if (take_image && search){
                search_images.push_back(registration_img);
                image_angles.push_back(theta_pos);
                reconstruct_img = false;
                take_image = false;
                take_search_image = true;
            }

            if (take_curr_loc_img){
                edge_images.push_back(registration_img);
                reconstruct_img = true;
                take_curr_loc_img = false;
                search = false;
            }

            if (take_image && !search){
                edge_images.push_back(registration_img);
                reconstruct_img = true;
                take_image = false;
                if (!moving){
                    std_msgs::msg::Empty empty;
                    pub_start_moving_->publish(empty);
                    moving = true;
                }
            }

        }

        // service to trigger poster reconstruction
        // topic: /reconstruct_poster   type: std_srvs::srv::Empty
        void reconstruct_srv(
            std_srvs::srv::Empty::Request::SharedPtr,
            std_srvs::srv::Empty::Response::SharedPtr)
        {
            // Declare what you need
            cv::FileStorage file("cv_images_test.ext", cv::FileStorage::WRITE);

            RCLCPP_INFO(rclcpp::get_logger("message"), "Writing to file!");

            int num = all_edge_markers.markers.size() - 1;

            file << "num" << num;

            for (int i = 0; i < all_edge_markers.markers.size()-1; i++){
                // Write to file!
                image1 = edge_images[i];
                file << "image" + to_string(i) << image1;

                file << "transform_x" + to_string(i) << -marker_transforms[i].x;
                file << "transform_y" + to_string(i) << -marker_transforms[i].y;
                file << "transform_theta" + to_string(i) << marker_transforms[i].theta;

                image2 = edge_images[i+1];
                file << "image" + to_string(i+1) << image2;
            }

            //Close the file and release all the memory buffers
            file.release();
            RCLCPP_INFO(rclcpp::get_logger("message"), "Done!");
        }

        // subscribe to loop completed status
        // topic: /finish_loop   type: std_msgs::msg::Empty
        void finish_loop_callback(std_msgs::msg::Empty::SharedPtr msg){
            update_nodes = true;
        }

        // subscribe a status to trigger poster reconstruction before updating from SESync
        // topic: /poster_before_update   type: std_msgs::msg::Empty
        void poster_before_callback(std_msgs::msg::Empty::SharedPtr msg){
            std::string file = "cv_images_before_" + to_string(loop_count) + ".ext";
            reconstruct_poster_(file);
        }

        // function to reconstruct the poster
        void reconstruct_poster_(std::string filename){
            // Declare what you need
            cv::FileStorage file(filename, cv::FileStorage::WRITE);

            RCLCPP_INFO(rclcpp::get_logger("message"), "Writing to file!");

            int num = all_node_markers.markers.size() - 2;

            file << "num" << num;

            for (int i = 0; i < all_node_markers.markers.size()-2; i++){
                // Write to file!
                image1 = edge_images[i];
                file << "image" + to_string(i) << image1;

                file << "transform_x" + to_string(i) << -marker_transforms[i].x;
                file << "transform_y" + to_string(i) << -marker_transforms[i].y;
                file << "transform_theta" + to_string(i) << marker_transforms[i].theta;

                image2 = edge_images[i+1];
                file << "image" + to_string(i+1) << image2;
            }

            //Close the file and release all the memory buffers
            file.release();
            RCLCPP_INFO(rclcpp::get_logger("message"), "Done!");
        }

        // subscribe to nodes marker list for poster visual reconstruction
        // topic: /nodes   type: visualization_msgs::msg::MarkerArray
        void poster_data_callback(
            const visualization_msgs::msg::MarkerArray::ConstSharedPtr& msg
        )
        {
            if (first_node){
                all_node_markers = visualization_msgs::msg::MarkerArray();
            }
            visualization_msgs::msg::Marker node;
            int ind = all_node_markers.markers.size();
            node.pose.position.x = msg->markers[ind].pose.position.x;
            node.pose.position.y = msg->markers[ind].pose.position.y;
            node.pose.orientation.z = msg->markers[ind].pose.orientation.z;
            node.id = msg->markers[ind].id;
            all_node_markers.markers.push_back(node);

            if (!search && !first_node && !update_nodes){
                int ind_ed = all_node_markers.markers.size() - 1;
                // convert rad to degree
                double rotation_angle = (all_node_markers.markers[ind_ed].pose.orientation.z - all_node_markers.markers[ind_ed-1].pose.orientation.z)*180.0/M_PI;
                double translation_x = all_node_markers.markers[ind_ed].pose.position.x - all_node_markers.markers[ind_ed-1].pose.position.x;
                double translation_y = all_node_markers.markers[ind_ed].pose.position.y - all_node_markers.markers[ind_ed-1].pose.position.y;

                img_transform::Vector2D transformation_edges;
                transformation_edges.x = translation_x;
                transformation_edges.y = translation_y;
                transformation_edges.theta = rotation_angle;

                marker_transforms.push_back(transformation_edges);
            }

            if (update_nodes){

                marker_transforms.clear();

                for (int i = 0; i < all_node_markers.markers.size() - 1; i++){

                    all_node_markers.markers[i+1].pose.position.x = msg->markers[i+1].pose.position.x;
                    all_node_markers.markers[i+1].pose.position.y = msg->markers[i+1].pose.position.y;
                    all_node_markers.markers[i+1].pose.orientation.z = msg->markers[i+1].pose.orientation.z;

                    // convert rad to degree
                    double rotation_angle = (all_node_markers.markers[i+1].pose.orientation.z - all_node_markers.markers[i].pose.orientation.z)*180.0/M_PI;
                    double translation_x = all_node_markers.markers[i+1].pose.position.x - all_node_markers.markers[i].pose.position.x;
                    double translation_y = all_node_markers.markers[i+1].pose.position.y - all_node_markers.markers[i].pose.position.y;

                    img_transform::Vector2D transformation_edges;
                    transformation_edges.x = translation_x;
                    transformation_edges.y = translation_y;
                    transformation_edges.theta = rotation_angle;

                    marker_transforms.push_back(transformation_edges);
                }
                update_done = true;
            }

            first_node = false;
            update_nodes = false;
        }

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
            double theta = msg->theta;

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
                edge.pose.orientation.z = 0.0;
                all_edge_markers.markers.push_back(edge);
                first = false;
                first_img = true;
            }


            if (!search){
                visualization_msgs::msg::Marker edge;
                geometry_msgs::msg::Point p;
                p.x = prev_x;
                p.y = prev_y;
                edge.points.push_back(p);
                p.x = x_pos;
                p.y = y_pos;

                edge.id = id_trans;
                edge.points.push_back(p);
                edge.pose.orientation.z = theta;
                all_edge_markers.markers.push_back(edge);

                int ind_ed = all_edge_markers.markers.size() - 1;
            }

        }



        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_current_img_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        sensor_msgs::msg::CameraInfo cam_info_;
        sensor_msgs::msg::CameraInfo cam_info_2_;
        std::shared_ptr<image_transport::CameraPublisher> pub_keypoint_img_;
        rclcpp::Publisher<img_transform::msg::Transform>::SharedPtr pub_transform_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr registration_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reconstruction_srv_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_poster_data_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_feature_transform_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_start_moving_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_loop_closure_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_rob_pose_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_search_done_;
        rclcpp::Subscription<img_transform::msg::FrameID>::SharedPtr sub_frame_id_;
        rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_wheel_transform_;
        rclcpp::Subscription<img_transform::msg::Waypoint>::SharedPtr sub_search_waypoint_;
        rclcpp::Subscription<img_transform::msg::Waypoint>::SharedPtr sub_loop_back_;
        rclcpp::Subscription<img_transform::msg::Odom>::SharedPtr sub_odom_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_waypoint_status_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_finish_loop_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_before_update_;

        cv::Ptr<cv::ORB> orb = cv::ORB::create();

        std::vector<img_transform::Vector2D> marker_transforms;

        Mat descriptors_loop;
        std::vector<KeyPoint> keypoints_loop;
        Mat img1;
        Mat registration_img;
        Mat loop_image;
        bool loop_verified = false;
        double loop_img_orientation = 0.0;
        bool take_search_image = false;
        std::vector<double> image_angles;
        std::vector<double> mse_vals;
        std::vector<Mat> edge_images;
        std::vector<Mat> search_images;
        std::vector<Mat> search_descriptions;
        std::vector<std::vector<KeyPoint>> search_keypoints;
        std::vector<KeyPoint> keypoints_search;


        std::string child_frame = "current";
        std::string parent_frame = "previous";
        bool first = true;
        tf2::Quaternion q;
        int count = 0;
        bool matched = true;
        float z = 0.095;

        bool update_done = false;
        bool update_nodes = false;


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
        visualization_msgs::msg::MarkerArray all_node_markers;

        int stitched_width = 0;
        int stitched_height = 0;
        cv::Mat image1;
        cv::Mat image2;

        int loop_count = 0;

        std::vector<int> prev_loop_pair {-1, -1};

        bool first_img = false;
        bool reconstruct_img = false;
        bool registration = false;
        bool first_node = true;

        double match_threshold = 11;
        double distance_threshold = 0.1;
        double bow_threshold = 0.14;

        double theta_pos = 0.0;

        bool moving = false;
        bool take_curr_loc_img = false;

        bool search = false;
        bool search_done = false;
        bool next_waypoint = false;
        int loop_id = 0;

        double rob_x = 0.0;
        double rob_y = 0.0;

        int lost_f_ids = 0;

        std::vector<std::vector<cv::Mat>> features;
        std::vector<std::vector<KeyPoint>> keypoints_vec;
        std::vector<cv::Mat> descriptors_vec;

        long int num_images = 1;

        bool take_image = true;

        double img_x_prev = 0.0;
        double img_y_prev = 0.0;

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
        db.add(features[id_vals[i]]);
    }

    QueryResults ret;

    std::vector<int> loop_candidates;

    int curr_img = id_vals[0];
    db.query(features[curr_img], ret, 4);
    for (int j = 1; j < id_vals.size(); j++){
        if (abs(ret[j].Id) < id_vals[0]){
            // arbitrary threshold (really depends on how many different images there are to compare against)
            if (ret[j] > bow_threshold){
                // another condition to determine if the detected loop is real
                if (abs(curr_img - id_vals[ret[j].Id]) > 9 && ret[j].Score > 0.0 && ret[j].Score < 1.0 ){
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
    double dist = sqrt((pow((x2-x1),2) + pow((y2-y1), 2)));

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