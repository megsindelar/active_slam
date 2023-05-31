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

class VideoFreq : public rclcpp::Node
{
    public:
    VideoFreq()
    : Node("video_frequency")
    {
        // subscribe to raw image
        // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_subscriber.cpp
        sub_current_img_ = std::make_shared<image_transport::CameraSubscriber>(
            image_transport::create_camera_subscription(
                this,
                "current_image",
                std::bind(&VideoFreq::image_callback, this, std::placeholders::_1, std::placeholders::_2),
                "compressed",
                rclcpp::QoS {10}.get_rmw_qos_profile()
            )
        );

        // Timer
        declare_parameter("rate", 100);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&VideoFreq::timer_callback, this));
    }
    private:
        inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
            return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
        }
        void timer_callback()
        {
            auto end_total = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end_total - begin_total);
            // RCLCPP_INFO(rclcpp::get_logger("message"), "Elapsed time: %ld\n", elapsed.count());
            if (elapsed.count() == 5){
                RCLCPP_INFO(rclcpp::get_logger("message"), "Frequency: %ld\n", count/elapsed.count());
                count = 0;
                begin_total = std::chrono::high_resolution_clock::now();
            }
        }

        void image_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
        ){
            // imshow("subscribed_image",cv_bridge::toCvCopy(*msg, msg->encoding)->image);
            Mat current_img = cv_bridge::toCvCopy(*msg, msg->encoding)->image;
            count++;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<image_transport::CameraSubscriber> sub_current_img_;
        sensor_msgs::msg::CameraInfo cam_info_;
        int count;
        std::chrono::system_clock::time_point begin_total = std::chrono::high_resolution_clock::now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoFreq>());
    rclcpp::shutdown();
    return 0;
}