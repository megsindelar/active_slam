#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdio.h>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <time.h>
#include <Eigen/Core>
#include "turtlebot_control/msg/wheel_commands.hpp"
#include "rgb_lights.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using namespace cv;

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10

class TurtlebotControl : public rclcpp::Node
{
    public:
    TurtlebotControl()
    : Node("turtlebot_control")
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

        // sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "/cmd_vel", 10, std::bind(&TurtlebotControl::velocity_callback, this, std::placeholders::_1));

        // publish raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        pub_current_img_ = std::make_shared<image_transport::CameraPublisher>(
            image_transport::create_camera_publisher(
                this,
                "current_image",
                custom_camera_qos_profile
            )
        );

        // pub_wheel_vel_ = this->create_publisher<turtlebot_control::msg::WheelCommands>("/wheel_cmd", 10);

        // custom_camera_qos_profile

        // rclcpp::QoS {10}.get_rmw_qos_profile()

        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

        // pub_keypoints_ = this->create_publisher<turtlebot_control::msg::Keypoints>("/keypoints", 10);

        // pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer
        declare_parameter("rate", 50);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&TurtlebotControl::timer_callback, this));

        // open raspberry pi camera
        cap.open(deviceID, apiID);

        // Turn on lights
        turtlebot_control::writeGPIO(redPin, "1");
        sleep(1);
        turtlebot_control::writeGPIO(greenPin, "1");
        sleep(1);
        turtlebot_control::writeGPIO(bluePin, "1");

        // std::ofstream file_export("/sys/class/gpio/export");
        // if (file_export.is_open())
        // {
        //     file_export << "17";
        //     file_export.close();
        // }
        // else{
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO export file");
        // }
        // sleep(1);

        // std::ofstream file_dir("/sys/class/gpio/gpio17/direction");
        // if (file_dir.is_open())
        // {
        //     file_dir << "out";
        //     file_dir.close();
        // }
        // else{
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO direction file");
        // }
        // sleep(1);

        // std::ofstream file_val("/sys/class/gpio/gpio17/value");
        // if (file_val.is_open())
        // {
        //     file_val << "0";
        //     file_val.close();
        // }
        // else{
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO value file");
        // }
        // sleep(1);

        // std::ofstream file_unexport("/sys/class/gpio/unexport");
        // if (file_unexport.is_open())
        // {
        //     file_unexport << "17";
        //     file_unexport.close();
        // }
        // else{
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO export file");
        // }
    }
    private:
        void timer_callback()
        {
            std_msgs::msg::Header header;
            header.stamp = get_clock()->now();
            cam_info_.header = header;

            // take image
            // Mat current_frame;
            // VideoCapture cap;

            // check if camera is open
            if (!cap.isOpened()) {
                cerr << "ERROR! Unable to open camera\n";
                // return -1;
            }
            else {
                cap.read(current_frame);
                // check if got a frame from camera
                if (current_frame.empty()) {
                    cerr << "ERROR! blank frame grabbed\n";
                    // break;
                }


                std::vector<KeyPoint> keypoints1, keypoints2;

                // if (!prev_frame.empty()){
                    // Ptr<FeatureDetector> detector = ORB::create();
                    // detector->detect (prev_frame ,keypoints1);
                    // detector->detect (current_frame ,keypoints2);

                    // // publish keypoints
                    // turtlebot_control::msg::Keypoints keypoints;
                    // for (long unsigned int i = 0; i < sizeof(keypoints1); i++){
                    //     RCLCPP_INFO(rclcpp::get_logger("message"), "TEST 10 %ld", sizeof(keypoints1));
                    //     keypoints.x_1.push_back(keypoints1.at(i).pt.x);
                    //     keypoints.y_1.push_back(keypoints1.at(i).pt.y);
                    //     keypoints.size_1.push_back(keypoints1.at(i).size);
                    //     keypoints.angle_1.push_back(keypoints1.at(i).angle);
                    //     keypoints.response_1.push_back(keypoints1.at(i).response);
                    //     keypoints.octave_1.push_back(keypoints1.at(i).octave);
                    //     keypoints.class_id_1.push_back(keypoints1.at(i).class_id);
                    //     keypoints.x_2.push_back(keypoints2.at(i).pt.x);
                    //     keypoints.y_2.push_back(keypoints2.at(i).pt.y);
                    //     keypoints.size_2.push_back(keypoints2.at(i).size);
                    //     keypoints.angle_2.push_back(keypoints2.at(i).angle);
                    //     keypoints.response_2.push_back(keypoints2.at(i).response);
                    //     keypoints.octave_2.push_back(keypoints2.at(i).octave);
                    //     keypoints.class_id_2.push_back(keypoints2.at(i).class_id);
                    // }
                    // pub_keypoints_->publish(keypoints);
                // }

                // for(int i=0; i<num_frames; i++){
                //     cap >> current_frame;
                // }

                // std::vector<int>params;
                // params.push_back(cv::IMWRITE_PNG_COMPRESSION);
                // params.push_back(9);

                // bool image_saved = false;
                // try{
                //     image_saved = imwrite("src/turtlebot_control/images/test_pic.png", frame, params);
                // }
                // catch (const cv::Exception& ex)
                // {
                //     fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
                // }

                // if (image_saved)
                // {
                //     printf("Image saved!");
                // }
                // else
                // {
                //     printf("Image not saved :(");
                // }


                // std::string pkg_path = ament_index_cpp::get_package_share_directory("turtlebot_control");
                // std::cout << pkg_path << std::endl;
                // std::string image_path = pkg_path + "/images/test_pic.png";
                // std::cout << image_path << std::endl;
                // cv::Mat current_frame = cv::imread(image_path);


                //change image from rasp pi to cv mat current_frame 

                // find a way to check if raspberry pi cam is on


                // resize()
                // sensor_msgs::msg::CompressedImage img_msg;
                // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, "bgr8", current_frame).toImageMsg();
                // img_bridge.toCompressedImageMsg(img_msg)
                RCLCPP_INFO(rclcpp::get_logger("message"), "Image size %d %d", current_frame.rows, current_frame.cols);

                // publish(*(cv_bridge::CvImage(header, color_encoding_, raw_left).toImageMsg()), camera_info_);
                pub_current_img_->publish(*(cv_bridge::CvImage(header, "bgr8", current_frame).toImageMsg()), cam_info_);
                // RCLCPP_INFO(rclcpp::get_logger("message"), "Published");




                // num_frames++;
                // std::cout << "published" << std::endl;

                // auto end = std::chrono::high_resolution_clock::now();
                // auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - begin);

                // if (num_frames == 10){
                //     RCLCPP_INFO(rclcpp::get_logger("message"), "Frequency: %ld\n", num_frames/elapsed.count());
                //     begin = std::chrono::high_resolution_clock::now();
                //     num_frames = 0;
                // }
                prev_frame = current_frame;

                // auto msg = geometry_msgs::msg::Twist();
                // msg.angular.z = 1.0;
                // msg.linear.x = 1.0 * 1.0;
                // msg.linear.y = 0.0;
                // pub_cmd_vel_->publish(msg);
            }
        }

        // void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        // {
        //     // double theta = msg->angular.z;
        //     // double x = msg->linear.x;
        //     // double y = msg->linear.y;
        //     // turtlelib::Twist2D twist = {theta, x, y};
        //     // turtlelib::DiffDrive robot = {track_width, wheel_radius};
        //     // // turtlelib::DiffDrive rob_new = robot.Inverse_Kin(q);
        //     // turtlelib::Phi wheel_vel = robot.Inverse_Kin(twist);
        //     // double temp_phi_r = wheel_vel.r / motor_cmd_per_rad_sec;
        //     // if (temp_phi_r > motor_cmd_max) {
        //     // temp_phi_r = motor_cmd_max;
        //     // } else if (temp_phi_r < -motor_cmd_max) {
        //     // temp_phi_r = -motor_cmd_max;
        //     // }
        //     // double temp_phi_l = wheel_vel.l / motor_cmd_per_rad_sec;
        //     // if (temp_phi_l > motor_cmd_max) {
        //     // temp_phi_l = motor_cmd_max;
        //     // } else if (temp_phi_l < -motor_cmd_max) {
        //     // temp_phi_l = -motor_cmd_max;
        //     // }
        //     auto mesg = turtlebot_control::msg::WheelCommands();
        //     mesg.left_velocity = 1.0;
        //     mesg.right_velocity = 1.0;
        //     pub_wheel_vel_->publish(mesg);
        // }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::CameraPublisher> pub_current_img_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
        // rclcpp::Publisher<turtlebot_control::msg::Keypoints>::SharedPtr pub_keypoints_;
        // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
        // rclcpp::Publisher<turtlebot_control::msg::WheelCommands>::SharedPtr pub_wheel_vel_;
        // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        sensor_msgs::msg::CameraInfo cam_info_;
        int num_frames = 0;
        std::chrono::system_clock::time_point begin = std::chrono::high_resolution_clock::now();
        Mat prev_frame;
        Mat current_frame;
        VideoCapture cap;
        int deviceID = -1;
        int apiID = CAP_V4L;

        // GPIO pins
        std::string redPin = "17";
        std::string greenPin = "18";
        std::string bluePin = "27";
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotControl>());
    rclcpp::shutdown();
    return 0;
}