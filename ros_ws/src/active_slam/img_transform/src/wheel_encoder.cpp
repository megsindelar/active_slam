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
#include "geometry_msgs/msg/pose.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "img_transform/msg/transform.hpp"
#include "img_transform/msg/odom.hpp"

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
    if (wheel_radius == -1.0) {throw std::logic_error("All parameters are not set!");}

    /// \brief the track width between the wheels of turtlebot
    /// \param track_width - track width of turtlebot
    this->declare_parameter("track_width", -1.0);
    track_width = this->get_parameter("track_width").get_parameter_value().get<double>();
    if (track_width == -1.0) {throw std::logic_error("All parameters are not set!");}

    /// \brief parameter that is the max allowable ticks that the motor can go
    /// \param motor_cmd_max - maximum ticks of motor
    this->declare_parameter("motor_cmd_max", -1.0);
    motor_cmd_max = this->get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    if (motor_cmd_max == -1.0) {throw std::logic_error("All parameters are not set!");}

    /// \brief parameter that is used to convert motor ticks to radians
    /// \param motor_cmd_per_rad_sec - convert motor ticks to radians
    this->declare_parameter("motor_cmd_per_rad_sec", -1.0);
    motor_cmd_per_rad_sec =
      this->get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    if (motor_cmd_per_rad_sec == -1.0) {throw std::logic_error("All parameters are not set!");}

    /// \brief parameter that is used to convert encoder ticks to radians
    /// \param encoder_ticks_per_rad - convert encoder ticks to radians
    this->declare_parameter("encoder_ticks_per_rad", -1.0);
    encoder_ticks_per_rad =
      this->get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    if (encoder_ticks_per_rad == -1.0) {throw std::logic_error("All parameters are not set!");}

    /// \brief parameter that is the difference between wheel base and cam in x-direction
    /// \param dx_cam - convert encoder ticks to radians
    this->declare_parameter("dx_cam", -1.0);
    dx_cam =
      this->get_parameter("dx_cam").get_parameter_value().get<double>();
    if (dx_cam == -1.0) {throw std::logic_error("All parameters are not set!");}

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

    pub_odom_ = this->create_publisher<img_transform::msg::Odom>(
      "odom_orientation", 10);

    pub_nodes_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "nodes_w", 10);

    pub_edges_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "edges_w", 10);

    pub_transform_ = this->create_publisher<img_transform::msg::Transform>("wheel_transform", 10);

    pub_rob_pose_ = this->create_publisher<geometry_msgs::msg::Point>("rob_pose", 10);

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    sub_odom_update_ = this->create_subscription<img_transform::msg::Odom>(
      "/odom_update", 10, std::bind(
        &WheelEncoder::odom_update_callback,
        this, std::placeholders::_1));

    sub_feature_transform_ = this->create_subscription<std_msgs::msg::Empty>(
      "/feature_transform", 10, std::bind(
        &WheelEncoder::feature_transform_callback,
        this, std::placeholders::_1));

    sub_next_waypoint_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/next_waypoint", 10, std::bind(
        &WheelEncoder::next_waypoint_callback,
        this, std::placeholders::_1));


    pub_waypoint_complete_ = this->create_publisher<std_msgs::msg::Empty>("waypoint_complete", 10);

    wheel_cmd_srv = this->create_service<std_srvs::srv::Empty>(
        "/next_waypoint_srv",
        std::bind(
            &WheelEncoder::next_waypoint_srv, this, std::placeholders::_1,
            std::placeholders::_2));

    stop_moving_srv = this->create_service<std_srvs::srv::Empty>(
        "/stop_moving",
        std::bind(
            &WheelEncoder::stop_turtlebot, this, std::placeholders::_1,
            std::placeholders::_2));

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

  void next_waypoint_callback(geometry_msgs::msg::Pose::SharedPtr msg)
  {
    waypoint.position.x = msg->position.x;
    waypoint.position.y = msg->position.y;
    waypoint.orientation.z = msg->orientation.z;

    RCLCPP_INFO(rclcpp::get_logger("message"), "Next waypoint wheel_enc!");

    next_waypoint = true;
    move = true;
  }

  void next_waypoint_srv(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        next_waypoint = true;
        move = true;
    }

  void stop_turtlebot(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = 0.0;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        pub_cmd_vel_->publish(msg);
        move = false;
    }

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

    // RCLCPP_INFO(rclcpp::get_logger("message"), "x, y, and theta sub: %f, %f, %f", x_sub, y_sub, theta_sub);
    double x = robot.x_get(); // - x_sub; //x_arr[m];
    double y = robot.y_get(); // - y_sub; //y_arr[m];
    double theta = robot.theta_get(); // - theta_sub; //theta_arr[m];

    // RCLCPP_INFO(rclcpp::get_logger("message"), "x: %f", x);
    // RCLCPP_INFO(rclcpp::get_logger("message"), "y: %f", y);


    img_transform::msg::Odom rotate;
    rotate.theta = theta;
    pub_odom_->publish(rotate);

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 436");

    // RCLCPP_INFO(rclcpp::get_logger("message"), "Test 134");

    Sophus::SE3d T0_w(rot_0, trans_0);

    Eigen::Vector<double,6> v01(dx_cam, 0.0, 0.0, 0.0, 0.0, 0.0);
    Sophus::SE3d T0_exp = Sophus::SE3d::exp(v01);

    Sophus::SE3d T0 = T0_w*T0_exp;

    if (update_pos){
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Robot x: %f", robot.x_get());
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Robot y: %f", robot.y_get());
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!22");
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Pos update wheel_enc node: %f", x);
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Pos update wheel_enc node: %f", y);
        T0.translation()(0) = x_update;
        T0.translation()(1) = y_update;
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Pos update trans_0 x: %f", T0.translation()(0));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Pos update trans_0 y: %f", T0.translation()(1));
        trans_0(0) = x;
        trans_0(1) = y;
        // RCLCPP_INFO(rclcpp::get_logger("message"), "T0_w trans %f", T0_w.translation()(0));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "T0_w trans %f", T0_w.translation()(1));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "trans_0 %f", trans_0(0));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "trans_0 %f", trans_0(1));
    }


    Eigen::Matrix<double, 3, 3> rot_1;
    Eigen::Matrix<double, 3, 1> trans_1;

    rot_1.row(0) << cos(theta), -sin(theta), 0.0;
    rot_1.row(1) << sin(theta), cos(theta), 0.0;
    rot_1.row(2) << 0.0, 0.0, 1.0;
    trans_1(0) = x;
    trans_1(1) = y;
    trans_1(2) = 0.0;


    Sophus::SE3d T1_w(rot_1, trans_1);

    Sophus::SE3d T1_exp = Sophus::SE3d::exp(v01);
    Sophus::SE3d T1 = T1_w*T1_exp;

    double dx_ = 0.1;
    double dy_ = 0.1;
    double dtheta_ = 0.1;


    Sophus::SE3d T_01 = T0.inverse()*T1;



    double x_trans = x - x_prev; //T_01.matrix()(0,3);
    double y_trans = y - y_prev; //T_01.matrix()(1,3);
    Eigen::Vector3d euler = T_01.rotationMatrix().eulerAngles(2,1,0);
    double theta_rot = euler(0);

    double theta_r = theta - theta_prev;



    //////////////////////////////////////////////////////////////////////

    // using PID control to follow waypoints

    // TODO: make a list or pattern sequence for target poses and angles 
    // Make it so waypoint always has a x, y, and theta
    // IDEA!!!!!!!!!!!!!!!!!!!!!!!
    // wherever the waypoint is, make a temp waypoint for the wheel base
    // which is start at the x,y of target, then look at direction of desired theta
    // and go backwards in a straight line of direction of desired theta by 0.1 (aka dx_cam)
    // to get the wheel desired target, then use stuff below to get wheels there, 
    // then just turn the robot until reach desired cam theta and pose!

    double v_lin_max = 0.055;
    double v_lin_min = 0.01;
    double v_ang_min = 0.05;
    double v_ang_max = 0.12;
    double k_dist = 1.0;
    double k_ang = 0.4;
    double v_lin = 0.0;
    double v_ang = 0.0;

    if (next_waypoint){
        RCLCPP_INFO(rclcpp::get_logger("message"), "theta: %f", theta);
        // theta_des = waypoints[w_ind][2];
        theta_des = waypoint.orientation.z;
        next_waypoint = false;
        double theta_temp = turtlelib::normalize_angle(theta_des);
        RCLCPP_INFO(rclcpp::get_logger("message"), "theta temp: %f", theta_temp);

        if (theta_temp >= 0.0 && theta_temp <= (M_PI/2.0)){
            temp_waypoint_x = waypoint.position.x - dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y - dx_cam*sin(theta_temp);
            RCLCPP_INFO(rclcpp::get_logger("message"), "temp waypoints 1: %f, %f", temp_waypoint_x, temp_waypoint_y);
        }

        if (theta_temp > (M_PI/2.0) && theta_temp <= M_PI){
            theta_temp = M_PI - theta_temp;
            temp_waypoint_x = waypoint.position.x + dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y - dx_cam*sin(theta_temp);
            RCLCPP_INFO(rclcpp::get_logger("message"), "temp waypoints 2: %f, %f", temp_waypoint_x, temp_waypoint_y);
        }

        if (theta_temp >= -M_PI && theta_temp < -(M_PI/2.0)){
            theta_temp = M_PI + theta_temp;
            temp_waypoint_x = waypoint.position.x + dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y + dx_cam*sin(theta_temp);
            RCLCPP_INFO(rclcpp::get_logger("message"), "temp waypoints 3: %f, %f", temp_waypoint_x, temp_waypoint_y);
        }

        if (theta_temp >= -(M_PI/2.0) && theta_temp < 0.0){
            theta_temp = -theta_temp;
            temp_waypoint_x = waypoint.position.x - dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y + dx_cam*sin(theta_temp);
            RCLCPP_INFO(rclcpp::get_logger("message"), "temp waypoints 4: %f, %f", temp_waypoint_x, temp_waypoint_y);
        }

        angle1_error = 1.0;
        angle2_error = 1.0;
        prev_dist_err = 100;

        des_ang = turtlelib::normalize_angle(atan2(temp_waypoint_y - T1_w.translation()(1), temp_waypoint_x - T1_w.translation()(0)));
        RCLCPP_INFO(rclcpp::get_logger("message"), "des_ang: %f", des_ang);

        w_ind++;
    }

    if (move){
        double err_threshold = 0.01;
        double distance_error = sqrt(pow(temp_waypoint_x - T1_w.translation()(0), 2) + pow(temp_waypoint_y - T1_w.translation()(1), 2));

        // RCLCPP_INFO(rclcpp::get_logger("message"), "T1 x, y: %f, %f", T1_w.translation()(0), T1_w.translation()(1));
        // RCLCPP_INFO(rclcpp::get_logger("message"), "temp_waypoint: %f, %f", temp_waypoint_x, temp_waypoint_y);
        // RCLCPP_INFO(rclcpp::get_logger("message"), "distance error: %f", distance_error);

        if (abs(angle1_error) > err_threshold && abs(distance_error) > err_threshold){
            angle1_error = (turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(des_ang));
            // RCLCPP_INFO(rclcpp::get_logger("message"), "des_ang: %f", des_ang);
            // RCLCPP_INFO(rclcpp::get_logger("message"), "theta: %f", theta);
            // RCLCPP_INFO(rclcpp::get_logger("message"), "normalized des_ang: %f", turtlelib::normalize_angle(des_ang));
            // RCLCPP_INFO(rclcpp::get_logger("message"), "normalized robot.theta_get(): %f", turtlelib::normalize_angle(robot.theta_get()));
            // RCLCPP_INFO(rclcpp::get_logger("message"), "normalized theta: %f", turtlelib::normalize_angle(theta));
            RCLCPP_INFO(rclcpp::get_logger("message"), "angle 1 error: %f", angle1_error);
            v_lin = 0.0;
            v_ang = k_ang*angle1_error;
            if (abs(angle1_error) > M_PI){
                v_ang = -v_ang;
            }
            if (v_ang > 0){
                v_ang = std::min(v_ang, v_ang_max);
                v_ang = std::max(v_ang, v_ang_min);
            }
            else{
                v_ang = std::max(v_ang, -v_ang_max);
                v_ang = std::min(v_ang, -v_ang_min);
            }
            prev_dist_err = 100.0;
            // RCLCPP_INFO(rclcpp::get_logger("message"), "v_ang: %f", v_ang);
        }
        if (abs(distance_error) > err_threshold && abs(angle1_error) < err_threshold){
            v_lin = k_dist*distance_error;
            RCLCPP_INFO(rclcpp::get_logger("message"), "distance error: %f", distance_error);
            RCLCPP_INFO(rclcpp::get_logger("message"), "prev distance error: %f", prev_dist_err);
            v_ang = 0.0;
            v_lin = std::min(v_lin, v_lin_max);
            v_lin = std::max(v_lin, v_lin_min);
            if (prev_dist_err < distance_error && abs(prev_dist_err) < 0.04){

                des_ang = turtlelib::normalize_angle(atan2(temp_waypoint_y - T1_w.translation()(1), temp_waypoint_x - T1_w.translation()(0)));
                angle1_error = (turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(des_ang));
                v_ang = k_ang*angle1_error;
                v_lin = 0.0;
                // prev_dist_err = distance_error;
            }
            prev_dist_err = std::min(prev_dist_err, distance_error);
            RCLCPP_INFO(rclcpp::get_logger("message"), "v_lin: %f", v_lin);
        }
        if (abs(distance_error) < err_threshold && abs(angle2_error) > err_threshold){
            angle2_error = turtlelib::normalize_angle(turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(theta_des));
            // RCLCPP_INFO(rclcpp::get_logger("message"), "normalized theta: %f", turtlelib::normalize_angle(theta));
            RCLCPP_INFO(rclcpp::get_logger("message"), "angle 2 error: %f", angle2_error);
            v_ang = k_ang*angle2_error;
            if (abs(angle2_error) > M_PI){
                v_ang = -v_ang;
            }
            if (v_ang > 0){
                v_ang = std::min(v_ang, v_ang_max);
                v_ang = std::max(v_ang, v_ang_min);
            }
            else{
                v_ang = std::max(v_ang, -v_ang_max);
                v_ang = std::min(v_ang, -v_ang_min);
            }
            v_lin = 0.0;
        }
        if (abs(distance_error) < err_threshold && abs(angle2_error) < err_threshold){
            RCLCPP_INFO(rclcpp::get_logger("message"), "Reached Waypoint!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "T1_w x, y: %f, %f", T1_w.translation()(0), T1_w.translation()(1));
            RCLCPP_INFO(rclcpp::get_logger("message"), "T1 x, y: %f, %f", T1.translation()(0), T1.translation()(1));
            v_ang = 0.0;
            v_lin = 0.0;
            move = false;
            std_msgs::msg::Empty empty;
            pub_waypoint_complete_->publish(empty);
            // next_waypoint = true;
        }

        // publish to cmd_vel topic (same as teleop topic)
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = v_ang;
        msg.linear.x = v_lin;
        msg.linear.y = 0.0;
        pub_cmd_vel_->publish(msg);
    }

    //////////////////////////////////////////////////////////////////////

    
    // need a subscriber of when recognize somewhere I've been before (aka bag of words that's from img_transform)
    if (reconstruct_graph == false && first == false && (abs(x_trans) > 0.1 || abs(y_trans) > 0.1 || abs(theta_r) > 0.785375 || ((abs(x_trans) > 0.05 || abs(y_trans) > 0.05) && feature_transform))){

        feature_transform = false;
        update_pos = false;
        RCLCPP_INFO(rclcpp::get_logger("message"), "Test test 1");

        geometry_msgs::msg::Point point_pub;
        point_pub.x = robot.x_get();
        point_pub.y = robot.y_get();

        pub_rob_pose_->publish(point_pub);

        x_prev = x;
        y_prev = y;
        theta_prev = theta;

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

        T01.x.push_back(T0.translation()(0));
        T01.x.push_back(T1.translation()(0));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T1 trans %f", T0.translation()(0));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T1 trans %f", T1.translation()(0));
        T01.x_w = (T1_w.matrix()(0,3));

        // RCLCPP_INFO(rclcpp::get_logger("message"), "x TESSSSSTTTTTTTTTTTTTTTTT %f: ", T1.matrix()(0,3));
        
        T01.y.push_back(T0.translation()(1));
        T01.y.push_back(T1.translation()(1));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T1 trans %f", T0.translation()(1));
        RCLCPP_INFO(rclcpp::get_logger("message"), "T1 trans %f", T1.translation()(1));
        T01.y_w = (T1_w.matrix()(1,3));

        T01.theta = robot.theta_get();
        T01.id = id;

        id++;

        pub_transform_->publish(T01);

        Eigen::Vector<double,6> v10(-dx_cam, 0.0, 0.0, 0.0, 0.0, 0.0);
        Sophus::SE3d T1_w_exp = Sophus::SE3d::exp(v10);
        T1_w = T1*T1_w_exp;

        rot_0 = T1_w.rotationMatrix();
        trans_0 = T1_w.translation();

    }
    }
  }


  void feature_transform_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    feature_transform = true;
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
        phi_sub = delta_p;
        // robot.Forward_Kin(delta_p);
        // x_sub = robot.x_get();
        // y_sub = robot.y_get();
        // theta_sub = robot.theta_get();

        rot_0.row(0) << cos(0.0), -sin(0.0), 0.0;
        rot_0.row(1) << sin(0.0), cos(0.0), 0.0;
        rot_0.row(2) << 0.0, 0.0, 1.0;
        trans_0(0) = 0.0;
        trans_0(1) = 0.0;
        trans_0(2) = 0.0;
        first_flag = true;


        // auto message = sensor_msgs::msg::JointState();
        // message.name = {"wheel_right_joint", "wheel_left_joint"};
        // // double dt = (msg->stamp.sec + (1e-9) * msg->stamp.nanosec) - time_prev;
        // message.position = {0.0, 0.0};
        // message.header.stamp = msg->header.stamp;
        // // temp_r = right_ang / dt;
        // // temp_l = left_ang / dt;
        // // header.stamp = msg->stamp;
        // // message.velocity = {temp_r, temp_l};
        // pub_joint_states_->publish(message);

        // RCLCPP_INFO(rclcpp::get_logger("message"), "robot x, y, theta: %f, %f, %f", robot.x_get(), robot.y_get(), robot.theta_get());
    }
    else{
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Inside");
        // turtlelib::Twist2D twist = robot.calc_body_twist(delta_p);

        // auto mesg = nav_msgs::msg::Odometry();
        // mesg.header.stamp = header.stamp;
        // mesg.header.frame_id = "odom";
        // mesg.child_frame_id = "body";
        // mesg.twist.twist.angular.z = twist.getW();
        // mesg.twist.twist.linear.x = twist.getX();
        // mesg.twist.twist.linear.y = twist.getY();

        delta_p.r -= phi_sub.r;
        delta_p.l -= phi_sub.l;

        robot.Forward_Kin(delta_p);
        // mesg.pose.pose.position.x = robot.x_get();
        // mesg.pose.pose.position.y = robot.y_get();
        // RCLCPP_INFO(rclcpp::get_logger("message"), "robot x, y, theta: %f, %f, %f", robot.x_get(), robot.y_get(), robot.theta_get());
        // mesg.pose.pose.position.z = 0.0;
        // tf2::Quaternion tf2_quatern;
        // double roll = 0.0;
        // double pitch = 0.0;
        // double yaw = robot.theta_get();
        // tf2_quatern.setRPY(roll, pitch, yaw);

        // mesg.pose.pose.orientation.x = tf2_quatern.x();
        // mesg.pose.pose.orientation.y = tf2_quatern.y();
        // mesg.pose.pose.orientation.z = tf2_quatern.z();
        // mesg.pose.pose.orientation.w = tf2_quatern.w();

        // RCLCPP_INFO(rclcpp::get_logger("message"), "rob x: %f", robot.x_get());

        // pub_odometry_->publish(mesg);

        // geometry_msgs::msg::TransformStamped t;
        // t.header.stamp = header.stamp;
        // t.header.frame_id = "odom";
        // t.child_frame_id = "body";

        // t.transform.translation.x = robot.x_get();
        // t.transform.translation.y = robot.y_get();
        // t.transform.rotation.x = tf2_quatern.x();
        // t.transform.rotation.y = tf2_quatern.y();
        // t.transform.rotation.z = tf2_quatern.z();
        // t.transform.rotation.w = tf2_quatern.w();
    }
    first = false;
    // if (first){
    //     x_prev = robot.x_get();
    //     y_prev = robot.y_get();
    //     first = false;
    // }


    // tf_broadcaster_->sendTransform(t);

  }

  void odom_update_callback(const img_transform::msg::Odom::SharedPtr msg)
  {
    update = msg->update;
    if (update){
        RCLCPP_INFO(rclcpp::get_logger("message"), "UPDATE!!!!!!!!!!!!!11");
        update_pos = true;
        x_update = msg->x;
        y_update = msg->y;
        theta_update = msg->theta;

        Eigen::Matrix<double, 3, 3> r_update;
        Eigen::Matrix<double, 3, 1> t_update;
        r_update.row(0) << cos(theta_update), -sin(theta_update), 0.0;
        r_update.row(1) << sin(theta_update), cos(theta_update), 0.0;
        r_update.row(2) << 0.0, 0.0, 1.0;
        t_update(0) = x_update;
        t_update(1) = y_update;
        t_update(2) = 0.0;

        Sophus::SE3d T0_update(r_update, t_update);

        Eigen::Vector<double,6> v01_update(dx_cam, 0.0, 0.0, 0.0, 0.0, 0.0);

        Sophus::SE3d T0_exp = Sophus::SE3d::exp(v01_update);
        // Sophus::SE3d T1 = T1_w*T1_exp;
        Sophus::SE3d T0_w_update = T0_update*T0_exp.inverse();

        x_prev = T0_w_update.translation()(0);
        y_prev = T0_w_update.translation()(1);


        robot = {track_width, wheel_radius, {robot.phi_r_get(), robot.phi_l_get()}, {robot.theta_get(), T0_w_update.translation()(0) + x_sub, T0_w_update.translation()(1) + y_sub}};
    }
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
  double wheel_radius, motor_cmd_max, track_width, motor_cmd_per_rad_sec, collision_radius, encoder_ticks_per_rad, dx_cam;
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
  rclcpp::Publisher<img_transform::msg::Odom>::SharedPtr pub_odom_;
  rclcpp::Subscription<img_transform::msg::Odom>::SharedPtr sub_odom_update_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_feature_transform_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_next_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_waypoint_complete_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reconstruction_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr wheel_cmd_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_moving_srv;

  double right_ang = 0.0;
  double left_ang = 0.0;
  double temp_l = 0.0;
  double temp_r = 0.0;

  double angle1_error = 1.0;
  double angle2_error = 1.0;

  bool next_waypoint = false;
  bool move = false;

  geometry_msgs::msg::Pose waypoint;

  int count_updates = 0;

  std_msgs::msg::Header header;

  bool feature_transform = false;

  visualization_msgs::msg::MarkerArray node_markers;
  visualization_msgs::msg::MarkerArray edge_markers;

  std::vector<std::vector<double>> waypoints {{0.4, 0.0, M_PI/2.0}, {0.4, 0.4, M_PI}, {0.0, 0.4, 3.0*M_PI/2.0}, {0.0, 0.0, 2.0*M_PI}};
  int w_ind = 0;

  int id = 0;
  double x_prev = 0.0;
  double y_prev = 0.0;
  double theta_prev = 0.0;

  double prev_dist_err = 100.0;

  double theta_des = 0.0;
  double des_ang = 0.0;

  turtlelib::Phi phi_sub = {0.0, 0.0};
  double x_sub = 0.0;
  double y_sub = 0.0;
  double theta_sub = 0.0;

  bool first = true;
  bool reconstruct_graph = false;
  int count = 0;

  Eigen::Matrix<double, 3, 3> rot_0;
  Eigen::Matrix<double, 3, 1> trans_0;

  bool first_flag = false;
  bool ff_1 = true;

  bool update = false;
  bool update_pos = false;
  double x_update = 0.0;
  double y_update = 0.0;
  double theta_update = 0.0;

  double temp_waypoint_x = 0.0;
  double temp_waypoint_y = 0.0;
//   std::vector<double> x_arr {0.0, 0.401742, 0.441624, 0.847666, 0.90923, 0.533757, 0.494193, 0.047714, 0.0};
//   std::vector<double> y_arr {0.0, -0.526768, -0.578827, -0.27026, -0.223236, 0.177863, 0.220178, -0.038281, 0.0};
//   std::vector<double> theta_arr {0.0, 0.001898, 1.506935, 1.573059, 3.114796, 3.241666, 4.751765, 2.029354, 0.0};
//   std::vector<double> x_arr {0.0, 1.0, 1.0, 0.0};
//   std::vector<double> y_arr {1.0, 1.0, 0.0, 0.0};
//   std::vector<double> theta_arr {M_PI/2.0, M_PI, (3.0*M_PI)/2.0, 2.0*M_PI};
//   std::vector<double> x_arr {-0.301859, -0.605766, -0.304241, -0.161178};
//   std::vector<double> y_arr {0.080700, -0.206141, -0.351543, -0.046565};
//   std::vector<double> theta_arr {-0.002847, 1.615771, 3.203731, 4.801784};
//   int m = 0;
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