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
#include "turtlebot_control/msg/keypoints.hpp"


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
        // publish raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        pub_current_img_ = std::make_shared<image_transport::Publisher>(
            image_transport::create_publisher(
                this,
                "current_image",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

        pub_keypoints_ = this->create_publisher<turtlebot_control::msg::Keypoints>("/keypoints", 10);

        // Timer
        declare_parameter("rate", 50);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&TurtlebotControl::timer_callback, this));
    }
    private:
        void timer_callback()
        {
            std_msgs::msg::Header header;
            header.stamp = get_clock()->now();
            cam_info_.header = header;

            // take image
            Mat current_frame;
            VideoCapture cap;

            int deviceID = -1;
            int apiID = CAP_V4L;
            // open raspberry pi camera
            cap.open(deviceID, apiID);
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

                pub_current_img_->publish(*cv_bridge::CvImage(header, "bgr8", current_frame).toImageMsg());
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
            }
            
            sleep(5);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::Publisher> pub_current_img_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
        rclcpp::Publisher<turtlebot_control::msg::Keypoints>::SharedPtr pub_keypoints_;
        sensor_msgs::msg::CameraInfo cam_info_;
        int num_frames = 0;
        std::chrono::system_clock::time_point begin = std::chrono::high_resolution_clock::now();
        Mat prev_frame;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotControl>());
    rclcpp::shutdown();
    return 0;
}