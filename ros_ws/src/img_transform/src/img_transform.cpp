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

#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include "matplotlibcpp.h"
#include <random>
#include <chrono>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <math.h>

using namespace std;
using namespace cv;

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

class ImgTransform : public rclcpp::Node
{
    public:
    ImgTransform()
    : Node("img_transform")
    {
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

        //a transform broadcaster to publish position and orientation of the turtlebot
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

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
            if (!current_img.empty() && !prev_img.empty() && done != 1)
            {
                // RCLCPP_INFO(rclcpp::get_logger("right_rad"), "sending back response: [%d]", right_rad);
                RCLCPP_INFO(rclcpp::get_logger("message"), "NEW TRANSFORM!!!!");

                auto begin_total = std::chrono::high_resolution_clock::now();
                std::vector<KeyPoint> keypoints_1, keypoints_2;
                Mat descriptors_1, descriptors_2;
                Ptr<FeatureDetector> detector = ORB::create();
                Ptr<DescriptorExtractor> descriptor = ORB::create();
                
                Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

                detector->detect ( prev_img,keypoints_1 );
                detector->detect ( current_img,keypoints_2 );
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 1");
                descriptor->compute ( prev_img, keypoints_1, descriptors_1 );
                descriptor->compute ( current_img, keypoints_2, descriptors_2 );

                Mat outimg1;
                drawKeypoints( prev_img, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
                Mat outimg2;
                drawKeypoints( current_img, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
                Mat outimg3;
                drawKeypoints( prev_img, keypoints_2, outimg3, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

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
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 2");

                min_dist = 10.0;

                std::vector< DMatch > good_matches;
                for ( int i = 0; i < descriptors_1.rows; i++ )
                {
                    if ( matches[i].distance <= min ( 2*min_dist, 30.0 ) )
                    {
                        good_matches.push_back ( matches[i] );
                    }
                }

                std::cout << "num of matches: " << good_matches.size() << std::endl;


                float z = 1.0;
                float f_1 = 295.009696;
                float f_2 = 296.22399483;
                float p_1 = 195.515395;
                float p_2 = 131.035055713;
                std::vector<float> key1_xw;
                std::vector<float> key1_yw;
                std::vector<float> key1_zw;
                std::vector<float> key1_xp;
                std::vector<float> key1_yp;
                std::vector<float> key2_xw;
                std::vector<float> key2_yw;
                std::vector<float> key2_zw;
                std::vector<float> key2_xp;
                std::vector<float> key2_yp;
                // first get transform point by multiplying by camera matrix
                for (int i=0; i<good_matches.size(); i++){
                    float k1_xw = (1/f_1)*(keypoints_1.at(good_matches[i].queryIdx).pt.x - p_1*z);
                    float k1_yw = (1/f_2)*(keypoints_1.at(good_matches[i].queryIdx).pt.y - p_2*z);
                    float k2_xw = (1/f_1)*(keypoints_2.at(good_matches[i].trainIdx).pt.x - p_1*z);
                    float k2_yw = (1/f_2)*(keypoints_2.at(good_matches[i].trainIdx).pt.y - p_2*z);
                    key1_xw.push_back(k1_xw);
                    // std::cout << "x1: " << k1_xw << std::endl;
                    key1_yw.push_back(k1_yw);
                    key1_zw.push_back(z);
                    key1_xp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.x);
                    key1_yp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.y);
                    key2_xw.push_back(k2_xw);
                    key2_yw.push_back(k2_yw);
                    key2_zw.push_back(z);
                    key2_xp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.x);
                    key2_yp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.y);
                }
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 3");

                // Convert the point cloud to Eigen
                int N = good_matches.size();


                Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
                for (size_t i = 0; i < N; ++i) {
                    // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                    src.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
                    std::cout << "z1: " << key1_zw.at(i) << std::endl;
                }
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 10");

                // Homogeneous coordinates
                Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
                src_h.resize(4, src.cols());
                src_h.topRows(3) = src;
                src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

                // Apply an arbitrary SE(3) transformation
                Eigen::Matrix4d T;
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 11");

                Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
                for (size_t i = 0; i < N; ++i) {
                    // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
                    tgt.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
                    std::cout << "z2: " << key2_zw.at(i) << std::endl;
                }
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 12");

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
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 13");

                // Solve with TEASER++
                teaser::RobustRegistrationSolver solver(params);
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                solver.solve(src, tgt);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                auto solution = solver.getSolution();
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 4");

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

                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 6");


                // draw transformation on first image to visualize
                std::vector<float> trans_x;
                Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts(3, N);
                Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts2(3, N);
                for(int i = 0; i < N; i++){
                    // std::cout << "i " << i << std::endl;
                    keypnts.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
                    keypnts2.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
                }
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 7");
                Eigen::Matrix<double, 3, 3> transformation;
                for (int i = 0; i < 2; i++){
                    transformation.row(i) << solution.rotation(i,0), solution.rotation(i,1), solution.translation(i);
                }
                transformation.row(2) << solution.rotation(2,0), solution.rotation(2,1), solution.rotation(2,2);
                std::cout << transformation << std::endl;
                RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 5");

                // std::vector<KeyPoint> key_trans_2 = keypoints_2;
                // std::vector<KeyPoint> key_trans_1 = keypoints_1;
                // for(int i = 0; i < N; i++){
                //     Eigen::Matrix<double, 3, 1> new_2 = transformation.inverse()*keypnts.col(i);
                //     Eigen::Matrix<double, 3, 1> new_1 = transformation*keypnts2.col(i);
                //     key_trans_2.at(i).pt.x = new_2(0);
                //     key_trans_2.at(i).pt.y = new_2(1);
                //     key_trans_1.at(i).pt.x = new_1(0);
                //     key_trans_1.at(i).pt.y = new_1(1);
                // }

                // Mat outimg_test_2;
                // drawKeypoints ( current_img, key_trans_2, outimg_test_2);
                // Mat outimg_test_1;
                // drawKeypoints ( current_img, key_trans_2, outimg_test_1);

                // imshow ( "image match", img_match );
                // imshow ("transformed keypoints (keypoint 1 to image 2)", outimg_test_2);
                // imshow ("transformed keypoints (keypoint 2 to image 1)", outimg_test_1);
                // imshow("keypoint 1 detection image 1", outimg1);
                // imshow("keypoint 2 detection image 2", outimg2);
                // imshow("keypoint 2 on detection image 1", outimg3);

                // TODO: get rid of
                done = 1;

                for (int i = 0; i < 3; i++){
                        RCLCPP_INFO(rclcpp::get_logger("message"), "Transformation: [%f] [%f] [%f]", transformation(i,0),transformation(i,1), transformation(i,2));
                }

                auto end_total = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - begin_total);
                printf("Time to complete program: %.3f seconds\n", elapsed.count()*1e-9);


                Eigen::Matrix<double, 3, Eigen::Dynamic> prev_pos(3, 1);
                prev_pos(0) = prev_position.at(0);
                prev_pos(1) = prev_position.at(1);
                prev_pos(2) = prev_position.at(2);


                Eigen::Matrix<double, 3, Eigen::Dynamic> current_pos(3, 1);
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = parent_frame;
                t.child_frame_id = child_frame + std::to_string(num_transforms);

                current_pos = transformation.inverse()*prev_pos;

                t.transform.translation.x = current_pos(0);
                t.transform.translation.y = current_pos(1);

                // double r21 = transformation(1,0);
                // double r11 = transformation(0,0);

                // double roll = atan2(r21,r11);
                // // double pitch = atan2(-transformation(2,0),sqrt(transformation(2,1)**2 + transformation(2,2)**2));
                // double yaw = atan2(transformation(2,1),transformation(2,2));
                // q.setRPY(roll, 0, yaw);
                // geometry_msgs::msg::Quaternion msg_quaternion = tf2::toMsg(q);
                // t.transform.rotation.x = msg_quaternion.x;
                // t.transform.rotation.y = msg_quaternion.y;
                // t.transform.rotation.z = msg_quaternion.z;
                // t.transform.rotation.w = msg_quaternion.w;

                num_transforms += 1;
                parent_frame = t.child_frame_id;

                tf_static_broadcaster_->sendTransform(t);

                prev_position.at(0) = current_pos(0);
                prev_position.at(1) = current_pos(1);
                prev_position.at(2) = current_pos(2);
            }
        }

        void image_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
        ){
            // imshow("subscribed_image",cv_bridge::toCvCopy(*msg, msg->encoding)->image);
            prev_img.release();
            prev_img = current_img.clone();
            current_img.release();
            current_img = cv_bridge::toCvCopy(*msg, msg->encoding)->image;
            done = 0;
            // if (count < 10){
            //     done = 0;
            //     count++;
            // }
            // waitKey(1);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::CameraSubscriber> sub_current_img_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        sensor_msgs::msg::CameraInfo cam_info_;
        Mat current_img;
        Mat prev_img;
        int done = 0;
        std::string child_frame = "current";
        std::string parent_frame = "previous";
        int num_transforms = 0;
        vector<int> prev_position {0,0,1};
        tf2::Quaternion q;
        // int count = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgTransform>());
    rclcpp::shutdown();
    return 0;
}