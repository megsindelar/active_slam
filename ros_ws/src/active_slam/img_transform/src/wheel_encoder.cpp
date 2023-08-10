/// \file turtle_control
/// \brief Used to control the turtlebot movements through joint states
///
/// PARAMETERS:
///     wheel_radius (double):  wheel radius of turtlebot
///     track_width (double): track width of turtlebot
///     motor_cmd_max (double): maximum ticks of motor
///     motor_cmd_per_rad_sec (double): convert motor ticks to radians
///     encoder_ticks_per_rad (double): convert encoder ticks to radians
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot_msgs/WheelCommands): command velocity to control the turtlebot
///     /joint_states (sensor_msgs/JointState): the joint states of the turtlebot
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs/Twist): description of the topic
///     /sensor_data (nuturtlebot_msgs/SensorData): description of the topic

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "img_transform/msg/transform.hpp"

#include <Eigen/Core>
#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>
#include <Eigen/SPQRSupport>

#include <geometry_msgs/msg/point.hpp>

#include "sophus/se3.hpp"
#include <iostream>

/// \brief a node to control the turtlebot movements
class WheelEncoder : public rclcpp::Node
{
public:
  WheelEncoder()
  : Node("wheel_encoder")
  {
    /// \brief the radius of the wheel of turtlebot
    /// \param wheel_radius - wheel radius of turtlebot
    this->declare_parameter("wheel_radius", -1.0);
    wheel_radius = this->get_parameter("wheel_radius").get_parameter_value().get<double>();
    if (wheel_radius == -1.0) {throw std::logic_error("All parameters are not set! WR");}

    /// \brief the track width between the wheels of turtlebot
    /// \param track_width - track width of turtlebot
    this->declare_parameter("track_width", -1.0);
    track_width = this->get_parameter("track_width").get_parameter_value().get<double>();
    if (track_width == -1.0) {throw std::logic_error("All parameters are not set! TW");}

    /// \brief parameter that is the max allowable ticks that the motor can go
    /// \param motor_cmd_max - maximum ticks of motor
    this->declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max = this->get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    if (motor_cmd_max == -1.0) {throw std::logic_error("All parameters are not set! MCM");}

    /// \brief parameter that is used to convert motor ticks to radians
    /// \param motor_cmd_per_rad_sec - convert motor ticks to radians
    this->declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec =
      this->get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    if (motor_cmd_per_rad_sec == -1.0) {throw std::logic_error("All parameters are not set! MCRS");}

    /// \brief parameter that is used to convert encoder ticks to radians
    /// \param encoder_ticks_per_rad - convert encoder ticks to radians
    this->declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad =
      this->get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    if (encoder_ticks_per_rad == -1.0) {throw std::logic_error("All parameters are not set! ETPR");}

    // /// \brief a subscriber to get the sensor data of the turtlebot
    // /// topic: /sensor_data (nuturtlebot_msgs/SensorData)
    // sub_sensor_data_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
    //   "/sensor_data", 10, std::bind(&WheelEncoder::sensor_callback, this, std::placeholders::_1));

    reconstruction_srv = this->create_service<std_srvs::srv::Empty>(
      "/reconstruction",
      std::bind(
        &WheelEncoder::reconstruct, this, std::placeholders::_1,
        std::placeholders::_2));

    /// \brief a subscriber to get the joint states of the turtlebot
    /// topic: /joint_states (sensor_msgs/JointState)
    sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(
        &WheelEncoder::joint_states_callback,
        this, std::placeholders::_1));

    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nodes_w", 10);

    pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "edges_w", 10);

    pub_transform_ = this->create_publisher<img_transform::msg::Transform>("wheel_transform", 10);

    pub_rob_pose_ = this->create_publisher<geometry_msgs::msg::Point>("rob_pose", 10);

    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 10);
    int rate_ms = 100 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&WheelEncoder::timer_callback, this));

    robot = turtlelib::DiffDrive {track_width, wheel_radius};
  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    // get wheel encoder data from sensor callback (in radians)
    // convert to meters (circumference of wheel)
    // use body length to calculate how cam moves (assuming it's in middle of body)
    // // -> for both turning and straight line (straight line is simple, but turning
    // //     is slightly more complex)

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 436");

    if (first_flag){

    if (ff_1){
        ff_1 = false;
        rot_0.row(0) << cos(0.0), -sin(0.0), 0.0;
        rot_0.row(1) << sin(0.0), cos(0.0), 0.0;
        rot_0.row(2) << 0.0, 0.0, 1.0;
        trans_0(0) = 0.0;
        trans_0(1) = 0.0;
        trans_0(2) = 0.0;

        for (int i = 0; i < 1000000; i++){
            std::cout << "waiting" << std::endl;
        }
    }

    double x = x_arr[m]; // robot.x_get() - x_sub;
    double y = y_arr[m]; // robot.y_get() - y_sub;
    double theta = theta_arr[m]; // robot.theta_get() - theta_sub;

    RCLCPP_INFO(rclcpp::get_logger("message"), "Test 436");

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 1");
    Sophus::SE3d T0(rot_0, trans_0);


    Eigen::Matrix<double, 3, 3> rot_1;
    Eigen::Matrix<double, 3, 1> trans_1;
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2");
    // RCLCPP_INFO(rclcpp::get_logger("message"), "x TESSSSSTTTTTTTTTTTTTTTTT %f: ", x);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "y %f: ", y);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "theta %f: ", theta);
    rot_1.row(0) << cos(theta), -sin(theta), 0.0;
    rot_1.row(1) << sin(theta), cos(theta), 0.0;
    rot_1.row(2) << 0.0, 0.0, 1.0;
    trans_1(0) = x;
    trans_1(1) = y;
    trans_1(2) = 0.0;
    // RCLCPP_INFO(rclcpp::get_logger("message"), "x, y, theta: %f, %f, %f", x, y, theta);
    RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2.5");

    Sophus::SE3d T1(rot_1, trans_1);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T1 rot mat 0: %f, %f", T1.rotationMatrix()(0,0), T1.rotationMatrix()(0,1));
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T1 rot mat 1: %f, %f", T1.rotationMatrix()(1,0), T1.rotationMatrix()(1,1));
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T1 x y: %f, %f", T1.translation()(0), T1.translation()(1));
    double dx_ = 0.1;
    double dy_ = 0.1;
    double dtheta_ = 0.1;
    // Eigen::Vector<double,6> v01(dx_, dy_, 0.0, 0.0, 0.0, dtheta_);
    // Sophus::SE3d T01_n = Sophus::SE3d::exp(v01);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2.7");

    // RCLCPP_INFO(rclcpp::get_logger("message"), "T0 x, y: %f, %f", T0.translation()(0), T0.translation()(1));

    // RCLCPP_INFO(rclcpp::get_logger("message"), "T1 x, y: %f, %f", T1.translation()(0), T1.translation()(1));

    // RCLCPP_INFO(rclcpp::get_logger("message"), "x TESSSSSTTTTTTTTTTTTTTTTT %f: ", T1.matrix()(0,3));

    Sophus::SE3d T_01 = T0.inverse()*T1;

    // RCLCPP_INFO(rclcpp::get_logger("message"), "x TESSSSSTTTTTTTTTTTTTTTTT %f: ", T1.matrix()(0,3));
    // Sophus::SE3d T_01 = T_01_g*T01_n;

    // RCLCPP_INFO(rclcpp::get_logger("message"), T_01.matrix());
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T_01 row 1: %f, %f, %f, %f", T_01.matrix()(0,0), T_01.matrix()(0,1), T_01.matrix()(0,2), T_01.matrix()(0,3));
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T_01 row 2: %f, %f, %f, %f", T_01.matrix()(1,0), T_01.matrix()(1,1), T_01.matrix()(1,2), T_01.matrix()(1,3));
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T_01 row 3: %f, %f, %f, %f", T_01.matrix()(2,0), T_01.matrix()(2,1), T_01.matrix()(2,2), T_01.matrix()(2,3));
    // RCLCPP_INFO(rclcpp::get_logger("message"), "T_01 row 4: %f, %f, %f, %f", T_01.matrix()(3,0), T_01.matrix()(3,1), T_01.matrix()(3,2), T_01.matrix()(3,3));
    

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 2");

    double x_trans = robot.x_get() - x_prev; //T_01.matrix()(0,3);
    double y_trans = robot.y_get() - y_prev; //T_01.matrix()(1,3);
    Eigen::Vector3d euler = T_01.rotationMatrix().eulerAngles(2,1,0);
    double theta_rot = euler(0);

    double theta_r = robot.theta_get() - theta_prev;

    // RCLCPP_INFO(rclcpp::get_logger("message"), "diff x, y, theta: %f, %f, %f", x_trans, y_trans, theta_r);

    // RCLCPP_INFO(rclcpp::get_logger("message"), "x: %f", x_trans);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "y: %f", y_trans);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "theta: %f", theta_r);

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 3");
 
    // need a subscriber of when recognize somewhere I've been before (aka bag of words that's from img_transform)
    if (reconstruct_graph == false){ // && first == false && (abs(x_trans) > 0.1 || abs(y_trans) > 0.1 || abs(theta_r) > 3.8)){
        // Sophus::SE3 T0(rot_mat, trans_mat);
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_0 row 1: %f, %f, %f, %f", T0.matrix()(0,0), T0.matrix()(0,1), T0.matrix()(0,2), T0.matrix()(0,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_0 row 2: %f, %f, %f, %f", T0.matrix()(1,0), T0.matrix()(1,1), T0.matrix()(1,2), T0.matrix()(1,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_0 row 3: %f, %f, %f, %f", T0.matrix()(2,0), T0.matrix()(2,1), T0.matrix()(2,2), T0.matrix()(2,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_0 row 4: %f, %f, %f, %f", T0.matrix()(3,0), T0.matrix()(3,1), T0.matrix()(3,2), T0.matrix()(3,3));
        
        
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_1 row 1: %f, %f, %f, %f", T1.matrix()(0,0), T1.matrix()(0,1), T1.matrix()(0,2), T1.matrix()(0,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_1 row 2: %f, %f, %f, %f", T1.matrix()(1,0), T1.matrix()(1,1), T1.matrix()(1,2), T1.matrix()(1,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_1 row 3: %f, %f, %f, %f", T1.matrix()(2,0), T1.matrix()(2,1), T1.matrix()(2,2), T1.matrix()(2,3));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T_1 row 4: %f, %f, %f, %f", T1.matrix()(3,0), T1.matrix()(3,1), T1.matrix()(3,2), T1.matrix()(3,3));
        
        
        RCLCPP_INFO(rclcpp::get_logger("message"), "diff x, y, theta: %f, %f, %f", x_trans, y_trans, theta_r);

        RCLCPP_INFO(rclcpp::get_logger("message"), "Test 4");

        geometry_msgs::msg::Point point_pub;
        point_pub.x = robot.x_get();
        point_pub.y = robot.y_get();

        pub_rob_pose_->publish(point_pub);

        // publish transform for SESync
        // Eigen::Matrix2d rot = Eigen::Rotation2D<double>(theta_rot).toRotationMatrix();
        // Eigen::Matrix<double, 3,3> rotate;
        // rotate.row(0) << rot(0,0), rot(0,1), 0.0;
        // rotate.row(1) << rot(1,0), rot(1,1), 0.0;
        // rotate.row(2) << 0.0, 0.0, 1.0;
        // Eigen::Matrix<double, 3, 1> trans;
        // trans(0) = robot.x_get();
        // trans(1) = robot.y_get();
        // trans(2) = 0.0;

        // Sophus::SE3 T1(rotate, trans);

        // Sophus::SE3 T_01_s = T0.inverse()*T1;

        x_prev = robot.x_get();
        y_prev = robot.y_get();
        theta_prev = robot.theta_get();

        img_transform::msg::Transform T01;
        for (int i = 0; i < 2; i++){
            T01.row_1.push_back(T_01.rotationMatrix()(0,i));
            T01.row_2.push_back(T_01.rotationMatrix()(1,i));
            T01.row_3.push_back(0.0);
            T01.row_4.push_back(0.0);
        }

        RCLCPP_INFO(rclcpp::get_logger("message"), "Test 5");
        T01.row_1.push_back(0.0);
        T01.row_2.push_back(0.0);
        T01.row_3.push_back(1.0);
        T01.row_4.push_back(0.0);

        T01.row_1.push_back(T_01.translation()(0));
        T01.row_2.push_back(T_01.translation()(1));
        T01.row_3.push_back(0.0);
        T01.row_4.push_back(1.0);

        T01.x.push_back(T0.matrix()(0,3));
        T01.x.push_back(T1.matrix()(0,3));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "x TESSSSSTTTTTTTTTTTTTTTTT %f: ", T1.matrix()(0,3));
        
        T01.y.push_back(T0.matrix()(1,3));
        T01.y.push_back(T1.matrix()(1,3));
        T01.id = id;

        id++;

        pub_transform_->publish(T01);

        rot_0 = T1.rotationMatrix();
        trans_0 = T1.translation();
        // RCLCPP_INFO(rclcpp::get_logger("message"), "t rot mat 0: %f, %f", rot_0(0,0), rot_0(0,1));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "t rot mat 1: %f, %f", rot_0(1,0), rot_0(1,1));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "t x y: %f, %f", trans_0(0), trans_0(1));
    }
    m++;
    if (m > 3){
        first_flag = false;
    }
    }
  }


  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    turtlelib::Phi delta_p = {
        static_cast<double>(msg->position.at(0)) - robot.phi_r_get(),
        static_cast<double>(msg->position.at(1)) - robot.phi_l_get(),
        };
    // RCLCPP_INFO(rclcpp::get_logger("message"), "DPhi r: %f", delta_p.r);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "DPhi l: %f", delta_p.l);
    if (first){
        // auto message = sensor_msgs::msg::JointState();
        // message.name = {"wheel_left_joint", "wheel_right_joint"};
        // message.position = {0.0, 0.0};
        // message.header.stamp = this->get_clock()->now();
        // message.velocity = {0.0, 0.0};
        // pub_joint_states_->publish(message);
        robot.Forward_Kin(delta_p);
        x_sub = robot.x_get();
        y_sub = robot.y_get();
        theta_sub = robot.theta_get();

        rot_0.row(0) << cos(0.0), -sin(0.0), 0.0;
        rot_0.row(1) << sin(0.0), cos(0.0), 0.0;
        rot_0.row(2) << 0.0, 0.0, 1.0;
        trans_0(0) = 0.0;
        trans_0(1) = 0.0;
        trans_0(2) = 0.0;
        first_flag = true;
        // RCLCPP_INFO(rclcpp::get_logger("message"), "robot x, y, theta: %f, %f, %f", robot.x_get(), robot.y_get(), robot.theta_get());
    }
    else{
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Inside");
        turtlelib::Twist2D twist = robot.calc_body_twist(delta_p);

        auto mesg = nav_msgs::msg::Odometry();
        mesg.header.stamp = header.stamp;
        mesg.header.frame_id = "odom";
        mesg.child_frame_id = "body";
        mesg.twist.twist.angular.z = twist.getW();
        mesg.twist.twist.linear.x = twist.getX();
        mesg.twist.twist.linear.y = twist.getY();

        robot.Forward_Kin(delta_p);
        mesg.pose.pose.position.x = robot.x_get();
        mesg.pose.pose.position.y = robot.y_get();
        // RCLCPP_INFO(rclcpp::get_logger("message"), "robot x, y, theta: %f, %f, %f", robot.x_get(), robot.y_get(), robot.theta_get());
        mesg.pose.pose.position.z = 0.0;
        tf2::Quaternion tf2_quatern;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = robot.theta_get();
        tf2_quatern.setRPY(roll, pitch, yaw);

        mesg.pose.pose.orientation.x = tf2_quatern.x();
        mesg.pose.pose.orientation.y = tf2_quatern.y();
        mesg.pose.pose.orientation.z = tf2_quatern.z();
        mesg.pose.pose.orientation.w = tf2_quatern.w();

        // RCLCPP_INFO(rclcpp::get_logger("message"), "rob x: %f", robot.x_get());

        // pub_odometry_->publish(mesg);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "body";

        t.transform.translation.x = robot.x_get();
        t.transform.translation.y = robot.y_get();
        t.transform.rotation.x = tf2_quatern.x();
        t.transform.rotation.y = tf2_quatern.y();
        t.transform.rotation.z = tf2_quatern.z();
        t.transform.rotation.w = tf2_quatern.w();
    }
    first = false;
    // if (first){
    //     x_prev = robot.x_get();
    //     y_prev = robot.y_get();
    //     first = false;
    // }


    // tf_broadcaster_->sendTransform(t);

  }

  void reconstruct(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
    {
        reconstruct_graph = true;
    }


//   void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
//   {
//     RCLCPP_INFO(rclcpp::get_logger("message"), "Received encoder feedback");
//     left_ang = static_cast<double>(msg->left_encoder) / encoder_ticks_per_rad;
//     right_ang = static_cast<double>(msg->right_encoder) / encoder_ticks_per_rad;
//     // auto message = sensor_msgs::msg::JointState();
//     // message.name = {"wheel_right_joint", "wheel_left_joint"};
//     double dt = (msg->stamp.sec + (1e-9) * msg->stamp.nanosec) - time_prev;
//     // message.position = {right_ang, left_ang};
//     // message.header.stamp = msg->stamp;
//     temp_r = right_ang / dt;
//     temp_l = left_ang / dt;
//     header.stamp = msg->stamp;
//     // message.velocity = {temp_r, temp_l};
//     // pub_joint_states_->publish(message);
//     // time_prev = (msg->stamp.sec + (1e-9) * msg->stamp.nanosec);

//     turtlelib::Phi delta_p = {
//       right_ang - robot.phi_r_get(),
//       left_ang - robot.phi_l_get(),
//     };

//     turtlelib::Twist2D twist = robot.calc_body_twist(delta_p);

//     auto mesg = nav_msgs::msg::Odometry();
//     mesg.header.stamp = header.stamp;
//     mesg.header.frame_id = "odom";
//     mesg.child_frame_id = "body";
//     mesg.twist.twist.angular.z = twist.getW();
//     mesg.twist.twist.linear.x = twist.getX();
//     mesg.twist.twist.linear.y = twist.getY();

//     robot.Forward_Kin(delta_p);
//     mesg.pose.pose.position.x = robot.x_get();
//     mesg.pose.pose.position.y = robot.y_get();
//     mesg.pose.pose.position.z = 0.0;
//     tf2::Quaternion tf2_quatern;
//     double roll = 0.0;
//     double pitch = 0.0;
//     double yaw = robot.theta_get();
//     tf2_quatern.setRPY(roll, pitch, yaw);

//     mesg.pose.pose.orientation.x = tf2_quatern.x();
//     mesg.pose.pose.orientation.y = tf2_quatern.y();
//     mesg.pose.pose.orientation.z = tf2_quatern.z();
//     mesg.pose.pose.orientation.w = tf2_quatern.w();

//     // pub_odometry_->publish(mesg);

//     geometry_msgs::msg::TransformStamped t;
//     t.header.stamp = header.stamp;
//     t.header.frame_id = "odom";
//     t.child_frame_id = "body";

//     t.transform.translation.x = robot.x_get();
//     t.transform.translation.y = robot.y_get();
//     t.transform.rotation.x = tf2_quatern.x();
//     t.transform.rotation.y = tf2_quatern.y();
//     t.transform.rotation.z = tf2_quatern.z();
//     t.transform.rotation.w = tf2_quatern.w();

//     // tf_broadcaster_->sendTransform(t);

//     visualization_msgs::msg::Marker node;
//     node.header.frame_id = "world";
//     node.header.stamp = this->get_clock()->now();
//     node.action = visualization_msgs::msg::Marker::ADD;
//     node.pose.orientation.w = 1.0;
//     node.type = visualization_msgs::msg::Marker::SPHERE;
//     node.scale.x = 0.1;
//     node.scale.y = 0.1;
//     node.scale.z = 0.1;
//     node.color.a = 1.0;
//     node.color.r = 0.9;
//     node.color.g = 0.3;
//     node.color.b = 0.3;

//     visualization_msgs::msg::Marker edge_m;
//     edge_m.header.frame_id = "world";
//     edge_m.header.stamp = this->get_clock()->now();
//     edge_m.action = visualization_msgs::msg::Marker::ADD;
//     edge_m.pose.orientation.w = 1.0;
//     edge_m.type = visualization_msgs::msg::Marker::LINE_STRIP;
//     edge_m.scale.x = 0.1;
//     edge_m.scale.y = 0.1;
//     edge_m.scale.z = 0.1;
//     edge_m.color.a = 1.0;
//     edge_m.color.r = 0.9;
//     edge_m.color.g = 0.3;
//     edge_m.color.b = 0.3;


//     node.header.stamp = this->get_clock()->now();
//     edge_m.header.stamp = this->get_clock()->now();

//     node.id = id;
//     node.pose.position.x = robot.x_get();
//     node.pose.position.y = robot.y_get();
//     node_markers.markers.push_back(node);
//     pub_nodes_->publish(node_markers);

//     edge_m.id = id;
//     geometry_msgs::msg::Point p;
//     p.x = x_prev;
//     p.y = y_prev;
//     edge_m.points.push_back(p);
//     p.x = robot.x_get();
//     p.y = robot.y_get();
//     edge_m.points.push_back(p);
//     edge_markers.markers.push_back(edge_m);
//     pub_edges_->publish(edge_markers);

//     id++;


//     x_prev = robot.x_get();
//     y_prev = robot.y_get();
//   }

  /// initialize all publishers, subscribers, and services
  double wheel_radius, motor_cmd_max, track_width, motor_cmd_per_rad_sec, collision_radius, encoder_ticks_per_rad;
//   rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_data_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_rob_pose_;
  double time_prev = 0.0;
  turtlelib::DiffDrive robot{track_width, wheel_radius};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_nodes_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_edges_;
  rclcpp::Publisher<img_transform::msg::Transform>::SharedPtr pub_transform_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reconstruction_srv;

  double right_ang = 0.0;
  double left_ang = 0.0;
  double temp_l = 0.0;
  double temp_r = 0.0;

  std_msgs::msg::Header header;

  visualization_msgs::msg::MarkerArray node_markers;
  visualization_msgs::msg::MarkerArray edge_markers;

  int id = 1;
  double x_prev = 0.0;
  double y_prev = 0.0;
  double theta_prev = 0.0;

  double x_sub = 0.0;
  double y_sub = 0.0;
  double theta_sub = 0.0;

  bool first = true;
  bool reconstruct_graph = false;
  int count = 0;

  Eigen::Matrix<double, 3, 3> rot_0;
  Eigen::Matrix<double, 3, 1> trans_0;

  bool first_flag = true;
  bool ff_1 = true;
//   std::vector<double> x_arr {0.0, 0.401742, 0.441624, 0.847666, 0.90923, 0.533757, 0.494193, 0.047714, 0.0};
//   std::vector<double> y_arr {0.0, -0.526768, -0.578827, -0.27026, -0.223236, 0.177863, 0.220178, -0.038281, 0.0};
//   std::vector<double> theta_arr {0.0, 0.001898, 1.506935, 1.573059, 3.114796, 3.241666, 4.751765, 2.029354, 0.0};
//   std::vector<double> x_arr {0.0, 1.0, 1.0, 0.0};
//   std::vector<double> y_arr {1.0, 1.0, 0.0, 0.0};
//   std::vector<double> theta_arr {M_PI/2.0, M_PI, (3.0*M_PI)/2.0, 2.0*M_PI};
  std::vector<double> x_arr {-0.301859, -0.605766, -0.304241, -0.161178};
  std::vector<double> y_arr {0.080700, -0.206141, -0.351543, -0.046565};
  std::vector<double> theta_arr {-0.002847, 1.615771, 3.203731, 4.801784};
  int m = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WheelEncoder>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      std::make_shared<WheelEncoder>()->get_logger(),
      "ERROR: " << e.what());
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}