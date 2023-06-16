#include <iostream>
#include <string>

#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "options.hpp"

using namespace std;
using namespace cv;

class Cam2Img : public rclcpp::Node
{
    public:
    Cam2Img()
    : Node("cam2img")
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

        // Create the image publisher with our custom QoS profile.
        pub = this->create_publisher<std_msgs::msg::String>("image", 10);

        // pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

        // pub_keypoints_ = this->create_publisher<turtlebot_control::msg::Keypoints>("/keypoints", 10);

        // Timer
        declare_parameter("rate", 50);
        int rate_ms = 1000 / (get_parameter("rate").get_parameter_value().get<int>());
        timer_ = create_wall_timer(
          std::chrono::milliseconds(rate_ms),
          std::bind(&Cam2Img::timer_callback, this));
    }
    private:
        void timer_callback()
        {
            // Get the frame from the video capture.
            cap >> frame;
            // Check if the frame was grabbed correctly
            if (!frame.empty()) {
                // Convert to a ROS image
                if (!is_flipped) {
                    convert_frame_to_message(frame, i, msg);
                } else {
                    // Flip the frame if needed
                    cv::flip(frame, flipped_frame, 1);
                    convert_frame_to_message(flipped_frame, i, msg);
                }
                if (show_camera) {
                    // NOTE(esteve): Use C version of cvShowImage to avoid this on Windows:
                    // http://stackoverflow.com/questions/20854682/opencv-multiple-unwanted-window-with-garbage-name
                    cv::Mat cvframe = frame;
                    // Show the image in a window called "cam2image".
                    // cv::ShowImage("cam2image", &cvframe);
                    // Draw the image to the screen and wait 1 millisecond.
                    cv::waitKey(1);
                }
                // Publish the image message and increment the frame_id.
                std::cout << "Publishing image #" << i << std::endl;
                pub->publish(msg);
                ++i;
            }
        }

        std::string
        mat_type2encoding(int mat_type)
        {
            switch (mat_type) {
                case CV_8UC1:
                return "mono8";
                case CV_8UC3:
                return "bgr8";
                case CV_16SC1:
                return "mono16";
                case CV_8UC4:
                return "rgba8";
                default:
                throw std::runtime_error("Unsupported encoding type");
            }
        }

        void convert_frame_to_message(
        const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
        {
            // copy cv information into ros message
            msg->height = frame.rows;
            msg->width = frame.cols;
            msg->encoding = mat_type2encoding(frame.type());
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            size_t size = frame.step * frame.rows;
            msg->data.resize(size);
            memcpy(&msg->data[0], frame.data, size);
            msg->header.frame_id = std::to_string(frame_id);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<sensor_msgs::msg::Image> pub;
        // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
        // rclcpp::Publisher<turtlebot_control::msg::Keypoints>::SharedPtr pub_keypoints_;
        // sensor_msgs::msg::CameraInfo cam_info_;


        // Set the parameters of the quality of service profile. Initialize as the default profile
        // and set the QoS parameters specified on the command line.
        rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;

        // rclcpp::QoSInitialization qos_init;

        cv::VideoCapture cap;
        // Initialize OpenCV image matrices.
        cv::Mat frame;
        cv::Mat flipped_frame;

        // is_flipped will cause the incoming camera image message to flip about the y-axis.
        bool is_flipped = false;
        size_t i = 1;
        bool show_camera = false;

        // Initialize a shared pointer to an Image message.
        sensor_msgs::msg::Image::Ptr msg = std::make_shared<sensor_msgs::msg::Image>();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    msg->is_bigendian = false;
    // Initialize default demo parameters
    size_t depth = 10;
    rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    size_t width = 320;
    size_t height = 240;

    qos_init.history_policy_arg = RMW_LOG_SEVERITY_DEBUG;

    // custom_qos_settings.depth = 10;
    // custom.qos_settings.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    // Configure demo parameters with command line options.
    bool success = parse_command_options(
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &width, &height);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Parsing successful? %d", success);

    // // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
    // auto node = rclcpp::node::Node::make_shared("cam2image");

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

    // // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // // callback.
    // auto callback =
    //     [&is_flipped](const std_msgs::msg::Bool::SharedPtr msg) -> void
    //     {
    //     is_flipped = msg->data;
    //     printf("Set flip mode to: %s\n", is_flipped ? "on" : "off");
    //     };

    // // Set the QoS profile for the subscription to the flip message.
    // rmw_qos_profile_t custom_flip_qos_profile = rmw_qos_profile_sensor_data;
    // custom_flip_qos_profile.depth = 10;

    // auto sub = node->create_subscription<std_msgs::msg::Bool>(
    //     "flip_image", callback, custom_flip_qos_profile);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(30);

    // Initialize OpenCV video capture stream.
    int deviceID = -1;
    int apiID = CAP_V4L;
    // open raspberry pi camera
    cap.open(deviceID, apiID);

    // Set the width and height based on command line arguments.
    cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap.isOpened()) {
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Could not open video stream\n");
        // fprintf(stderr, "Could not open video stream\n");
        return 1;
    }

    // Initialize a shared pointer to an Image message.
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;

    size_t i = 1;
    rclcpp::spin(std::make_shared<Cam2Img>());
    rclcpp::shutdown();
    return 0;
}