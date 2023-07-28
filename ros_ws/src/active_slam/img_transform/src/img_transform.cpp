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
#include "img_transform/msg/keypoints.hpp"
#include "img_transform/msg/transform.hpp"

#include "sophus/se3.hpp"
#include "img_transform/transform.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/compressed_image.hpp"

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
        //std::make_shared<image_transport::Subscriber>(
        // sub_current_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        //     "/current_image/compressed", 10, std::bind(
        //         &ImgTransform::image_callback,
        //         this, std::placeholders::_1)
        //     // image_transport::create_subscription(
        //         // this,
        //         // "current_image",
        //         // std::bind(&ImgTransform::image_callback, this, std::placeholders::_1, std::placeholders::_2),
        //         // "compressed",
        //         // rclcpp::QoS {10}.get_rmw_qos_profile()
        // );

        sub_current_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/current_image/compressed", 10,
            std::bind(&ImgTransform::image_callback, this, std::placeholders::_1)
        );

        // publish robot path to see where it has been
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // publish transforms for cpl_slam
        pub_transform_ = this->create_publisher<img_transform::msg::Transform>("transform", 10);

        // publish robot state for cpl_slam
        pub_robot_state_ = this->create_publisher<img_transform::msg::Transform>("robot_state", 10);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // rclcpp::QoS {10}.get_rmw_qos_profile()

        // publish raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        pub_keypoint_img_ = std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(
                this,
                "keypoint_image",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

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
                rotation_current.row(0) << 1.0, 0.0, 0.0;
                rotation_current.row(1) << 0.0, 1.0, 0.0;
                rotation_current.row(2) << 0.0, 0.0, 1.0;
                translation_current(0) = 0.0;
                translation_current(1) = 0.0;
                translation_current(1) = 0.0;
                Sophus::SE3d Transformation_prev(rotation_current, translation_current);
            }

            Eigen::Matrix<double, 4, 4> Transformation_current = Transformation_prev.matrix();
            if (!current_img.empty() && !prev_img.empty()) // && done != 1)
            {
                auto begin_total = std::chrono::high_resolution_clock::now();
                Mat descriptors_1, descriptors_2;
                Ptr<FeatureDetector> detector = ORB::create();
                Ptr<DescriptorExtractor> descriptor = ORB::create();
                std::vector<KeyPoint> keypoints_1, keypoints_2;

                Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

                detector->detect ( prev_img,keypoints_1 );
                detector->detect ( current_img,keypoints_2 );

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

                min_dist = 10.0;

                // sort and then take a certain num of matches every time (use std sort)

                std::vector< DMatch > good_matches;
                // RCLCPP_INFO(rclcpp::get_logger("message"), "Matches size: %ld", matches.size());
                // std::sort(matches.begin(), matches.end(), img_transform::compare);

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
                    rotation_current = rotation_mat;
                    translation_current = translation;
                    Sophus::SE3d Transformation(rotation_mat, translation);
                    Sophus::SE3d Trans = Transformation_prev*Transformation;
                    Transformation_current = Trans.matrix();
                }
                else{
                    matched = true;
                    std::sort(matches.begin(), matches.end(), [](cv::DMatch& a, cv::DMatch& b) {
                        return a.distance < b.distance;
                    });

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
                        float k1_xw = (z/f_1)*(keypoints_1.at(good_matches[i].queryIdx).pt.x - p_1*z);
                        float k1_yw = (z/f_2)*(keypoints_1.at(good_matches[i].queryIdx).pt.y - p_2*z);
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

                    // TODO: get rid of
                    done = 1;

                    Sophus::SE3d Trans = Transformation_prev*Transformation;
                    Transformation_current = Trans.matrix();//Transformation_prev.matrix()*Transformation.matrix();
                    for (int i = 0; i < 3; i++){
                        rotation_current.row(i) << Transformation_current(i,0), Transformation_current(i,1), Transformation_current(i,2);
                        translation_current(i) = Transformation_current(i,3);
                    }
                    Transformation_prev = Trans;
                }


                // publish path to see how the robot has moved
                geometry_msgs::msg::PoseStamped pos;
                path.header.stamp = this->get_clock()->now();
                path.header.frame_id = "world";
                pos.header.stamp = this->get_clock()->now();
                pos.header.frame_id = "world";
                pos.pose.position.x = translation_current(0);
                pos.pose.position.y = translation_current(1);

                Eigen::Quaterniond q(rotation_current);
                geometry_msgs::msg::Quaternion msg_quaternion_path = tf2::toMsg(q);
                pos.pose.orientation.x = msg_quaternion_path.x;
                pos.pose.orientation.y = msg_quaternion_path.y;
                pos.pose.orientation.z = msg_quaternion_path.z;
                pos.pose.orientation.w = msg_quaternion_path.w;
                path.poses.push_back(pos);
                pub_path_->publish(path);


                // publish frame to visualize orientation
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "previous";
                t.child_frame_id = "current";

                t.transform.translation.x = translation_current(0);
                t.transform.translation.y = translation_current(1);
                t.transform.rotation.x = msg_quaternion_path.x;
                t.transform.rotation.y = msg_quaternion_path.y;
                t.transform.rotation.z = msg_quaternion_path.z;
                t.transform.rotation.w = msg_quaternion_path.w;

                tf_broadcaster_->sendTransform(t);


                // publish transform for cpl_slam
                img_transform::msg::Transform T_01;
                img_transform::msg::Transform rob_state;
                T_01.id = id;
                rob_state.id = id;
                for (int i = 0; i < 4; i++)
                {
                    T_01.row_1.push_back((Transformation.matrix())(0,i));
                    T_01.row_2.push_back((Transformation.matrix())(1,i));
                    T_01.row_3.push_back((Transformation.matrix())(2,i));
                    T_01.row_4.push_back((Transformation.matrix())(3,i));
                    rob_state.row_1.push_back((Transformation_current.matrix())(0,i));
                    rob_state.row_2.push_back((Transformation_current.matrix())(1,i));
                    rob_state.row_3.push_back((Transformation_current.matrix())(2,i));
                    rob_state.row_4.push_back((Transformation_current.matrix())(3,i));
                }

                img_transform::Vertex vertex;
                Eigen::Matrix<double, 4, 4> tr;
                img_transform::Vector2D trans;
                trans.x = 1.0;
                trans.y = 1.0;
                trans.theta = 1.0;
                tr.row(0) << Transformation_current.row(0), Transformation_current.row(0), Transformation_current.row(0), Transformation_current.row(0);
                tr.row(1) << Transformation_current.row(1), Transformation_current.row(1), Transformation_current.row(1), Transformation_current.row(1);
                tr.row(2) << Transformation_current.row(2), Transformation_current.row(2), Transformation_current.row(2), Transformation_current.row(2);
                tr.row(3) << Transformation_current.row(3), Transformation_current.row(3), Transformation_current.row(3), Transformation_current.row(3);
                // trans = img_transform::trans_to_vec(tr);

                cv::DMatch i;
                cv::DMatch j;
                bool y = img_transform::compare(i,j);

                pub_transform_->publish(T_01);

                pub_robot_state_->publish(rob_state);

                num_transforms ++;

                first = false;
                id ++;
            }
        }

        void image_callback(
            const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg
        ){
            auto end_total = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end_total - begin_total);
            // every 10 Hz
            // RCLCPP_INFO(rclcpp::get_logger("message"), "Elapsed time: %ld", elapsed.count());
            // if ((elapsed*100) > std::chrono::duration<long int>(1))
            // {
            prev_img.release();
            prev_img = current_img.clone();
            current_img.release();
            // current_img = cv_bridge::toCvCopy(*msg, msg->encoding)->image;
            current_img = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
            done = 0;
            begin_total = std::chrono::high_resolution_clock::now();
            // }
        }



        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_current_img_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        sensor_msgs::msg::CameraInfo cam_info_;
        std::shared_ptr<image_transport::CameraPublisher> pub_keypoint_img_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::Publisher<img_transform::msg::Transform>::SharedPtr pub_transform_;
        rclcpp::Publisher<img_transform::msg::Transform>::SharedPtr pub_robot_state_;


        Mat current_img;
        Mat prev_img;
        int done = 0;
        std::string child_frame = "current";
        std::string parent_frame = "previous";
        int num_transforms = 0;
        bool first = true;
        tf2::Quaternion q;
        int count = 0;
        double total_trans_x = 0;
        double total_trans_y = 0;
        bool matched = true;
        float z = 0.095;
        Sophus::SE3d Transformation_prev;
        Sophus::SE3d Trans;
        Sophus::SE3d Transformation;
        Eigen::Matrix<double, 3, 3> rotation_mat;
        Eigen::Matrix<double, 3, 3> rotation_current;
        Eigen::VectorXd translation = Eigen::VectorXd(3);
        Eigen::VectorXd translation_current = Eigen::VectorXd(3);
        nav_msgs::msg::Path path;
        std::chrono::system_clock::time_point begin_total = std::chrono::high_resolution_clock::now();
        int id = 1;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgTransform>());
    rclcpp::shutdown();
    return 0;
}