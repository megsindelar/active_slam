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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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
        // optional custom QOS settings
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
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

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

    }
    private:
        void timer_callback()
        {
            std_msgs::msg::Header header;
            header.stamp = get_clock()->now();
            cam_info_.header = header;


            // check if camera is open
            if (!cap.isOpened()) {
                cerr << "ERROR! Unable to open camera\n";
            }
            else {
                cap.read(current_frame);
                // check if got a frame from camera
                if (current_frame.empty()) {
                    cerr << "ERROR! blank frame grabbed\n";
                    // break;
                }


                std::vector<KeyPoint> keypoints1, keypoints2;

                
                pub_current_img_->publish(*(cv_bridge::CvImage(header, "bgr8", current_frame).toImageMsg()), cam_info_);

                prev_frame = current_frame;
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::CameraPublisher> pub_current_img_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
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