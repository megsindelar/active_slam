#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include <opencv2/calib3d.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <math.h>
#include "img_transform/msg/keypoints.hpp"

#include "sophus/se3.hpp"
#include "img_transform/transform.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

#define RED_PIN 17
#define BLUE_PIN 18
#define GREEN_PIN 27

class ImgTransform : public rclcpp::Node
{
    public:
    ImgTransform()
    : Node("img_transform")
    {
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

        // subscribe to raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_subscriber.cpp
        sub_current_img_ = std::make_shared<image_transport::CameraSubscriber>(
            image_transport::create_camera_subscription(
                this,
                "current_image",
                std::bind(&ImgTransform::image_callback, this, std::placeholders::_1, std::placeholders::_2),
                "compressed",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        // rclcpp::QoS {10}.get_rmw_qos_profile()

        //a transform broadcaster to publish position and orientation of the turtlebot
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        // publish raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        pub_keypoint_img_ = std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(
                this,
                "keypoint_image",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        // raspi_cam_sub_.subscribe(this, "current_image/compressed", custom_camera_qos_profile);
        // raspi_cam_info_sub_.subscribe(this, "camera_info", custom_camera_qos_profile);

        // // sync.reset(new Sync(MySyncPolicy(1), raspi_cam_sub_, raspi_cam_info_sub_));

        // sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>(
        // raspi_cam_sub_, raspi_cam_info_sub_, 10);
        // sync->registerCallback(
        //     std::bind(
        //         &ImgTransform::synchronized_img_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Timer
        declare_parameter("rate", 200);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&ImgTransform::timer_callback, this));

    }
    private:
        inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
            return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
        }
        void timer_callback()
        {
            if (first){
                prev_pos(0) = 0.0;
                prev_pos(1) = 0.0;
                prev_pos(2) = z;
                prev_pos(3) = 1.0;
                rotation_mat_prev.row(0) << 1.0, 0.0, 0.0;
                rotation_mat_prev.row(1) << 0.0, 1.0, 0.0;
                rotation_mat_prev.row(2) << 0.0, 0.0, 1.0;
            }
            // RCLCPP_INFO(rclcpp::get_logger("message"), "NEW TRANSFORM!!!!");
            if (!current_img.empty() && !prev_img.empty()) // && done != 1)
            // if (sizeof(keypoints_1) > 6)
            {
                // RCLCPP_INFO(rclcpp::get_logger("right_rad"), "sending back response: [%d]", right_rad);
                // RCLCPP_INFO(rclcpp::get_logger("message"), "NEW TRANSFORM!!!!");

                auto begin_total = std::chrono::high_resolution_clock::now();
                Mat descriptors_1, descriptors_2;
                Ptr<FeatureDetector> detector = ORB::create();
                Ptr<DescriptorExtractor> descriptor = ORB::create();
                std::vector<KeyPoint> keypoints_1, keypoints_2;

                Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

                detector->detect ( prev_img,keypoints_1 );
                detector->detect ( current_img,keypoints_2 );

                // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 1");
                descriptor->compute ( prev_img, keypoints_1, descriptors_1 );
                descriptor->compute ( current_img, keypoints_2, descriptors_2 );

                Mat outimg1;
                drawKeypoints( prev_img, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
                Mat outimg2;
                drawKeypoints( current_img, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
                Mat outimg3;
                drawKeypoints( prev_img, keypoints_2, outimg3, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


                std_msgs::msg::Header header;
                header.stamp = get_clock()->now();
                cam_info_.header = header;
                pub_keypoint_img_->publish(*(cv_bridge::CvImage(header, "bgr8", outimg2).toImageMsg()), cam_info_);

                vector<DMatch> matches;
                try{
                    matcher->match ( descriptors_1, descriptors_2, matches );
                }
                catch (const std::exception & e) {
                    RCLCPP_ERROR_STREAM(
                    std::make_shared<ImgTransform>()->get_logger(),
                    "Matching ERROR: Shutting down node! " << e.what());
                    rclcpp::shutdown();
                }
                double min_dist=10000, max_dist=0;
                // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 2");

                min_dist = 10.0;

                // sort and then take a certain num of matches every time (use std sort)

                std::vector< DMatch > good_matches;
                RCLCPP_INFO(rclcpp::get_logger("message"), "Matches size: %ld", matches.size());
                // std::sort(matches.begin(), matches.end(), img_transform::compare);


                Eigen::VectorXd current_pos(4);
                if (matches.size() < 12)
                {
                    matched = false;
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    RCLCPP_INFO(rclcpp::get_logger("message"), "Less than 12 matches");
                    current_pos = Transformation_prev.inverse().matrix()*prev_pos;
                    rotation_mat = rotation_mat_prev;
                }
                else{
                    matched = true;
                    std::sort(matches.begin(), matches.end(), [](cv::DMatch& a, cv::DMatch& b) {
                        return a.distance < b.distance;
                    });

                    for (int i = 0; i < 12; i++)
                    {
                        // RCLCPP_INFO(rclcpp::get_logger("message"), "Match distances: %f", matches[i].distance);
                        good_matches.push_back(matches[i]);
                    }

                    // for ( int i = 0; i < descriptors_1.rows; i++ )
                    // {
                    //     if ( matches[i].distance <= min ( 2*min_dist, 30.0 ) )s
                    //     {
                    //         good_matches.push_back ( matches[i] );
                    //     }
                    // }

                    // std::cout << "num of matches: " << good_matches.size() << std::endl;


                    float f_1 = 295.009696;
                    float f_2 = 296.22399483;
                    float p_1 = 195.515395;
                    float p_2 = 131.035055713;
                    std::vector<float> key1_xw;
                    std::vector<float> key1_yw;
                    std::vector<float> key1_zw;
                    // std::vector<float> key1_xp;
                    // std::vector<float> key1_yp;
                    std::vector<float> key2_xw;
                    std::vector<float> key2_yw;
                    std::vector<float> key2_zw;
                    // std::vector<float> key2_xp;
                    // std::vector<float> key2_yp;

                    // first get transform point by multiplying by camera matrix
                    int counter_matches = 0;
                    for (size_t i=0; i<good_matches.size(); i++){
                        float k1_xw = (z/f_1)*(keypoints_1.at(good_matches[i].queryIdx).pt.x - p_1*z);
                        float k1_yw = (z/f_2)*(keypoints_1.at(good_matches[i].queryIdx).pt.y - p_2*z);
                        float k2_xw = (z/f_1)*(keypoints_2.at(good_matches[i].trainIdx).pt.x - p_1*z);
                        float k2_yw = (z/f_2)*(keypoints_2.at(good_matches[i].trainIdx).pt.y - p_2*z);

                        key1_xw.push_back(k1_xw);
                        // std::cout << "x1: " << k1_xw << std::endl;
                        key1_yw.push_back(k1_yw);
                        key1_zw.push_back(z);
                        // key1_xp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.x);
                        // key1_yp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.y);
                        key2_xw.push_back(k2_xw);
                        key2_yw.push_back(k2_yw);
                        key2_zw.push_back(z);
                        // key2_xp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.x);
                        // key2_yp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.y);

                        counter_matches++;
                    }
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 3");

                    // Convert the point cloud to Eigen
                    size_t N = good_matches.size();


                    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
                    for (size_t i = 0; i < N; ++i) {
                        // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                        src.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
                        std::cout << "z1: " << key1_zw.at(i) << std::endl;
                    }
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 10");

                    // Homogeneous coordinates
                    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
                    src_h.resize(4, src.cols());
                    src_h.topRows(3) = src;
                    src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

                    // Apply an arbitrary SE(3) transformation
                    Eigen::Matrix4d T;
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 11");

                    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
                    for (size_t i = 0; i < N; ++i) {
                        // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                        tgt.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
                        std::cout << "z2: " << key2_zw.at(i) << std::endl;
                    }
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 12");

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
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 13");

                    // Solve with TEASER++
                    teaser::RobustRegistrationSolver solver(params);
                    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                    solver.solve(src, tgt);
                    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                    auto solution = solver.getSolution();
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 4");

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

                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 6");

                    // draw transformation on first image to visualize
                    std::vector<float> trans_x;
                    Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts(3, N);
                    Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts2(3, N);
                    for(int i = 0; i < N; i++){
                        // std::cout << "i " << i << std::endl;
                        keypnts.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
                        keypnts2.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
                    }
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 7");


                    // np.array([[0,      -omg[2],  omg[1]],
                    //      [omg[2],       0, -omg[0]],
                    //      [-omg[1], omg[0],       0]])

                    // Eigen::Matrix<double, 3, 3> transformation;
                    // Eigen::Matrix<double, 3, 3> trans_mat;
                    Eigen::VectorXd translation(3);
                    for (int i = 0; i < 3; i++){
                        rotation_mat.row(i) << solution.rotation(i,0), solution.rotation(i,1), solution.rotation(i,2);
                        translation(i) = solution.translation(i);
                    }

                    // trans_mat.row(0) << 0, -solution.translation(2), solution.translation(1);
                    // trans_mat.row(1) << solution.translation(2), 0, -solution.translation(0);
                    // trans_mat.row(2) << -solution.translation(1), solution.translation(0), 0;
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "Transformation Matrix 3: %f %f %f", solution.rotation(2,0), solution.rotation(2,1), solution.rotation(2,2));

                    // Eigen::Matrix<double, 4, 4> inv_trans;
                    // Eigen::Matrix<double, 3, 3> inv_rot;
                    // Eigen::VectorXd inv_rot_p(3);
                    // inv_rot = rotation_mat.transpose();
                    // inv_rot_p = -inv_rot*translation;
                    // // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 1");
                    // for (int i = 0; i < 3; i++)
                    // {
                    //     inv_trans.row(i) << inv_rot(i,0), inv_rot(i,1), inv_rot(i,2), inv_rot_p(i);
                    //     RCLCPP_INFO(rclcpp::get_logger("message"), "Inverse Transform Matrix: %f %f %f %f", inv_rot(i,0), inv_rot(i,1), inv_rot(i,2), inv_rot_p(i));
                    // }
                    // inv_trans.row(3) << 0.0, 0.0, 0.0, 1.0;
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "Inverse Transform Matrix: %f %f %f %f", 0.0, 0.0, 0.0, 1.0);
                    Sophus::SE3d Transformation(rotation_mat, translation);

                    // TODO: get rid of
                    done = 1;

                    // current_pos = transformation.inverse()*prev_pos;
                    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 3");
                    current_pos = Transformation.inverse().matrix()*prev_pos;
                    Eigen::Matrix<double, 4, 4> Inv_Trans = Transformation.inverse().matrix();

                    // for (int i = 0; i < 3; i++){
                    //     rotation_mat.row(i) << Inv_Trans(i,0), Inv_Trans(i,1), Inv_Trans(i,2);
                    //     translation(i) = Inv_Trans(i,3);
                    // }

                    int error = (current_pos(0) - prev_pos(0)) + (current_pos(1) - prev_pos(1));

                    // fix for if TEASER finds an unreasonable transformation
                    if (error > 0.1)
                    {
                        current_pos = Transformation_prev.inverse().matrix()*prev_pos;
                    }
                    else
                    {
                        Transformation_prev = Transformation;
                    }
                    rotation_mat_prev = rotation_mat;
                }

                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = parent_frame;
                t.child_frame_id = child_frame + std::to_string(num_transforms);
                RCLCPP_INFO(rclcpp::get_logger("message"), "Prev pos: %f %f %f %f", prev_pos(0), prev_pos(1), prev_pos(2), prev_pos(3));
                RCLCPP_INFO(rclcpp::get_logger("message"), "Current pos: %f %f", current_pos(0), current_pos(1));
                RCLCPP_INFO(rclcpp::get_logger("message"), "Current minus prev pos: %f %f", prev_pos(0) - current_pos(0), prev_pos(1) - current_pos(1));

                // RCLCPP_INFO(rclcpp::get_logger("message"), "Prev pos: %f %f", prev_pos(0), prev_pos(1));

                t.transform.translation.x = current_pos(0) - prev_pos(0);
                t.transform.translation.y = current_pos(1) - prev_pos(1);
                // t.transform.translation.x = solution.translation(0);
                // t.transform.translation.y = solution.translation(1);
                total_trans_x += current_pos(0) - prev_pos(0);
                total_trans_y += current_pos(1) - prev_pos(1);
                // RCLCPP_INFO(rclcpp::get_logger("message"), "Translation: %f %f", t.transform.translation.x, t.transform.translation.y);

                // RCLCPP_INFO(rclcpp::get_logger("message"), "Total Translation: %f %f", total_trans_x, total_trans_y);

                // RCLCPP_INFO(rclcpp::get_logger("message"), "Current Pos: %f %f", current_pos(0), current_pos(1));

                Eigen::Quaterniond q(rotation_mat);
                geometry_msgs::msg::Quaternion msg_quaternion = tf2::toMsg(q);

                // t.transform.rotation.x = q.x;
                // t.transform.rotation.y = q.y;
                // t.transform.rotation.z = q.z;
                // t.transform.rotation.w = q.w;


                t.transform.rotation.x = msg_quaternion.x;
                t.transform.rotation.y = msg_quaternion.y;
                t.transform.rotation.z = msg_quaternion.z;
                t.transform.rotation.w = msg_quaternion.w;

                num_transforms ++;
                parent_frame = t.child_frame_id;

                tf_static_broadcaster_->sendTransform(t);

                prev_pos(0) = current_pos(0);
                prev_pos(1) = current_pos(1);
                prev_pos(2) = current_pos(2);
                prev_pos(3) = current_pos(3);
                first = false;
            }
        }

        void image_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
        ){
            // imshow("subscribed_image",cv_bridge::toCvCopy(*msg, msg->encoding)->image);
            // if (matched == true)
            // {
            //     prev_img.release();
            //     prev_img = current_img.clone();
            // }
            prev_img.release();
            prev_img = current_img.clone();
            current_img.release();
            current_img = cv_bridge::toCvCopy(*msg, msg->encoding)->image;
            done = 0;
        }

        // void synchronized_img_callback(
        //     const sensor_msgs::msg::Image::ConstSharedPtr& msg,
        //     const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
        // ){
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "Testing");
        //     if (matched == true)
        //     {
        //         prev_img.release();
        //         prev_img = current_img.clone();
        //     }
        //     current_img.release();
        //     current_img = cv_bridge::toCvCopy(*msg, msg->encoding)->image;
        //     done = 0;
        // }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::CameraSubscriber> sub_current_img_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        sensor_msgs::msg::CameraInfo cam_info_;
        std::shared_ptr<image_transport::CameraPublisher> pub_keypoint_img_;

        // message_filters::Subscriber<sensor_msgs::msg::Image> raspi_cam_sub_;
        // message_filters::Subscriber<sensor_msgs::msg::CameraInfo> raspi_cam_info_sub_;
        // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image , 
        // sensor_msgs::msg::CameraInfo> MySyncPolicy;
        // // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        // std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> sync;

        Mat current_img;
        Mat prev_img;
        int done = 0;
        std::string child_frame = "current";
        std::string parent_frame = "previous";
        Eigen::VectorXd prev_pos = Eigen::VectorXd(4);
        int num_transforms = 0;
        bool first = true;
        tf2::Quaternion q;
        // std::vector<KeyPoint> keypoints_1, keypoints_2;
        int count = 0;
        double total_trans_x = 0;
        double total_trans_y = 0;
        bool matched = true;
        float z = 0.095;
        Sophus::SE3d Transformation_prev;
        Eigen::Matrix<double, 3, 3> rotation_mat_prev;
        Eigen::Matrix<double, 3, 3> rotation_mat;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgTransform>());
    rclcpp::shutdown();
    return 0;
}