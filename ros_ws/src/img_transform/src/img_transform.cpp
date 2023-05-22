#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/highgui.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


class ImgTransform : public rclcpp::Node
{
    public:
    ImgTransform()
    : Node("img_transform")
    {
        // // publish raw image
        // // referenced from Nick Morales: https://github.com/ngmor/unitree_camera/blob/main/unitree_camera/src/img_publisher.cpp
        // pub_current_img_ = std::make_shared<image_transport::CameraPublisher>(
        //     image_transport::create_camera_publisher(
        //         this,
        //         "current_image",
        //         rclcpp::QoS {10}.get_rmw_qos_profile()
        //     )
        // );


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

        // Timer
        declare_parameter("rate", 200);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&ImgTransform::timer_callback, this));
    }
    private:
        void timer_callback()
        {
            // std_msgs::msg::Header header;
            // header.stamp = get_clock()->now();
            // cam_info_.header = header;


            // std::string pkg_path = ament_index_cpp::get_package_share_directory("img_transform");
            // std::cout << pkg_path << std::endl;
            // std::string image_path = pkg_path + "/images/image.jpg";
            // std::cout << image_path << std::endl;
            // cv::Mat current_frame = cv::imread(image_path);


            // //change image from rasp pi to cv mat current_frame 

            // // find a way to check if raspberry pi cam is on
            // pub_current_img_->publish(*(cv_bridge::CvImage(header, "bgr8", current_frame).toImageMsg()), cam_info_);
        }

        // void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr&)
        void image_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr&
        ){
            cv::imshow("subscribed_image",cv_bridge::toCvCopy(*msg, msg->encoding)->image);
            cv::waitKey(1);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        // std::shared_ptr<image_transport::CameraPublisher> pub_current_img_;
        std::shared_ptr<image_transport::CameraSubscriber> sub_current_img_;
        sensor_msgs::msg::CameraInfo cam_info_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgTransform>());
    rclcpp::shutdown();
    return 0;
}