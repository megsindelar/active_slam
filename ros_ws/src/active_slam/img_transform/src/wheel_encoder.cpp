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
#include "tf2_ros/transform_broadcaster.h"
#include "std_srvs/srv/empty.hpp"
#include "img_transform/msg/transform.hpp"
#include "img_transform/msg/odom.hpp"
#include "img_transform/msg/waypoint.hpp"

#include <Eigen/Core>
#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>
#include <Eigen/SPQRSupport>

#include <geometry_msgs/msg/point.hpp>

#include "math.h"

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

    sub_loop_closure_ = this->create_subscription<std_msgs::msg::Empty>(
      "/loop_closure", 10, std::bind(
        &WheelEncoder::loop_closure_callback,
        this, std::placeholders::_1));

    sub_next_waypoint_ = this->create_subscription<img_transform::msg::Waypoint>(
      "/next_waypoint", 10, std::bind(
        &WheelEncoder::next_waypoint_callback,
        this, std::placeholders::_1));

    sub_search_waypoint_ = this->create_subscription<img_transform::msg::Waypoint>(
      "/search_waypoint", 10, std::bind(
        &WheelEncoder::search_waypoint_callback,
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

  void next_waypoint_callback(img_transform::msg::Waypoint::SharedPtr msg)
  {
    waypoint.position.x = msg->x;
    waypoint.position.y = msg->y;
    waypoint.orientation.z = msg->theta;

    next_waypoint = true;

    loop_back = msg->loop;
    search = msg->search;
    move_waypoint = true;
    first_search = true;
  }

  void search_waypoint_callback(img_transform::msg::Waypoint::SharedPtr msg)
  {
    search_waypoint.position.x = msg->x;
    search_waypoint.position.y = msg->y;
    search_waypoint.orientation.z = msg->theta;

    search = msg->search;

    if (search){
        searching = true;
        move_search = true;
    }
    else{
        searching = false;
        move_search = false;
    }
  }

  void next_waypoint_srv(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        next_waypoint = true;
        move_waypoint = true;
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
        move_waypoint = false;
        move_search = false;
    }

  /// \brief timer callback running at a set frequency
  void timer_callback()
  {

    if (first_flag){

    double x = robot.x_get();
    double y = robot.y_get();
    double theta = robot.theta_get();



    img_transform::msg::Odom rotate;
    rotate.theta = theta;
    pub_odom_->publish(rotate);


    Sophus::SE3d T0_w(rot_0, trans_0);

    Eigen::Vector<double,6> v01(dx_cam, 0.0, 0.0, 0.0, 0.0, 0.0);
    Sophus::SE3d T0_exp = Sophus::SE3d::exp(v01);

    Sophus::SE3d T0 = T0_w*T0_exp;

    if (update_pos){
        T0.translation()(0) = x_update;
        T0.translation()(1) = y_update;
        trans_0(0) = x;
        trans_0(1) = y;
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



    double x_trans = x - x_prev;
    double y_trans = y - y_prev;
    Eigen::Vector3d euler = T_01.rotationMatrix().eulerAngles(2,1,0);
    double theta_rot = euler(0);

    double theta_r = theta - theta_prev;



    //////////////////////////////////////////////////////////////////////

    // using PID control to follow waypoints

    double v_lin_max = 0.055;
    double v_lin_search_max = 0.03;
    double v_lin_min = 0.01;
    double v_ang_min = 0.05;
    double v_ang_max = 0.12;
    double k_dist = 1.0;
    double k_search_dist = 0.3;
    double k_ang = 0.4;
    double v_lin = 0.0;
    double v_ang = 0.0;
    double err_threshold = 0.01;

    if (next_waypoint){
        theta_des = waypoint.orientation.z;
        next_waypoint = false;
        double theta_temp = turtlelib::normalize_angle(theta_des);

        if (theta_temp >= 0.0 && theta_temp <= (M_PI/2.0)){
            temp_waypoint_x = waypoint.position.x - dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y - dx_cam*sin(theta_temp);
        }

        if (theta_temp > (M_PI/2.0) && theta_temp <= M_PI){
            theta_temp = M_PI - theta_temp;
            temp_waypoint_x = waypoint.position.x + dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y - dx_cam*sin(theta_temp);
        }

        if (theta_temp >= -M_PI && theta_temp < -(M_PI/2.0)){
            theta_temp = M_PI + theta_temp;
            temp_waypoint_x = waypoint.position.x + dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y + dx_cam*sin(theta_temp);
        }

        if (theta_temp >= -(M_PI/2.0) && theta_temp < 0.0){
            theta_temp = -theta_temp;
            temp_waypoint_x = waypoint.position.x - dx_cam*cos(theta_temp);
            temp_waypoint_y = waypoint.position.y + dx_cam*sin(theta_temp);
        }

        angle1_error = 1.0;
        angle2_error = 1.0;
        prev_dist_err = 100;

        v_lin_neg = false;

        des_ang = turtlelib::normalize_angle(atan2(temp_waypoint_y - T1_w.translation()(1), temp_waypoint_x - T1_w.translation()(0)));

        w_ind++;
    }

    if (move_waypoint){
        distance_error = sqrt(pow(temp_waypoint_x - T1_w.translation()(0), 2) + pow(temp_waypoint_y - T1_w.translation()(1), 2));

        if (abs(angle1_error) > err_threshold){ //abs(distance_error) > err_threshold
            double theta_norm = turtlelib::normalize_angle(theta);
            double des_norm = turtlelib::normalize_angle(des_ang);
            angle1_error = theta_norm - des_norm;

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
        }
        if (abs(distance_error) > err_threshold && abs(angle1_error) < err_threshold){
            v_lin = k_dist*distance_error;
            v_lin = std::min(v_lin, v_lin_max);
            v_lin = std::max(v_lin, v_lin_min);

            if (v_lin_neg){
                v_lin = -v_lin;
            }

            v_ang = 0.0;
            if (prev_dist_err < distance_error && abs(prev_dist_err) < 0.04){

                des_ang = turtlelib::normalize_angle(atan2(temp_waypoint_y - T1_w.translation()(1), temp_waypoint_x - T1_w.translation()(0)));
                angle1_error = (turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(des_ang));
                v_ang = k_ang*angle1_error;
                v_lin = 0.0;
            }
            prev_dist_err = std::min(prev_dist_err, distance_error);
        }
        if (abs(distance_error) < err_threshold && abs(angle2_error) > err_threshold && abs(angle1_error) < err_threshold){
            angle2_error = turtlelib::normalize_angle(turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(theta_des));
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
        if (abs(distance_error) < err_threshold && abs(angle2_error) < err_threshold && abs(angle1_error) < err_threshold){
            v_ang = 0.0;
            v_lin = 0.0;
            move_waypoint = false;
            std_msgs::msg::Empty empty;
            pub_waypoint_complete_->publish(empty);
            waypoint_completed = true;
        }

        // publish to cmd_vel topic (same as teleop topic)
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = v_ang;
        msg.linear.x = v_lin;
        msg.linear.y = 0.0;
        pub_cmd_vel_->publish(msg);
    }


    if (searching){
        theta_des = search_waypoint.orientation.z;

        searching = false;
        if (first_search){
            x_des_search = search_waypoint.position.x;
            y_des_search = search_waypoint.position.y;
            theta_des_search = search_waypoint.orientation.z;
            angle1_error = 0.0;
            distance_error = 0.0;
        }
        else{
            if (theta_des == theta_des_search && search_waypoint.position.x == x_des_search && search_waypoint.position.y == y_des_search){
                angle1_error = 0.0;
                v_lin_neg = true;
                temp_waypoint_x = search_waypoint.position.x; // + sub_err_x;
                temp_waypoint_y = search_waypoint.position.y; // + sub_err_y;
                sub_err_x = 0.0;
                sub_err_y = 0.0;
                prev_dist_err = 100.0;
                distance_error = sqrt(pow(temp_waypoint_x - T1.translation()(0), 2) + pow(temp_waypoint_y - T1.translation()(1), 2));
            }
            else{
                temp_waypoint_x = search_waypoint.position.x;
                temp_waypoint_y = search_waypoint.position.y;

                double test_ang = atan2(temp_waypoint_y - T1.translation()(1), temp_waypoint_x - T1.translation()(0));

                des_ang = turtlelib::normalize_angle(atan2(temp_waypoint_y - T1.translation()(1), temp_waypoint_x - T1.translation()(0)));

                angle1_error = 1.0;
                prev_dist_err = 100;

                v_lin_neg = false;
                distance_error = sqrt(pow(temp_waypoint_x - T1.translation()(0), 2) + pow(temp_waypoint_y - T1.translation()(1), 2));
            }
        }

        first_search = false;

    }


    if (move_search){
        if (abs(angle1_error) > err_threshold){
            angle1_error = (turtlelib::normalize_angle(theta) - turtlelib::normalize_angle(des_ang));
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
        }

        if (abs(distance_error) > err_threshold && abs(angle1_error) < err_threshold){
            distance_error = sqrt(pow(temp_waypoint_x - T1.translation()(0), 2) + pow(temp_waypoint_y - T1.translation()(1), 2));
            v_lin = k_search_dist*distance_error;
            v_lin = std::min(v_lin, v_lin_search_max);
            v_lin = std::max(v_lin, v_lin_min);

            if (v_lin_neg){
                v_lin = -v_lin;
            }

            v_ang = 0.0;

            if (prev_dist_err < distance_error && abs(prev_dist_err - distance_error) > 0.01){
                distance_error = 0.0;
                sub_err_x += temp_waypoint_x - T1.translation()(0);
                sub_err_y += temp_waypoint_y - T1.translation()(1);
            }

            prev_dist_err = std::min(prev_dist_err, distance_error);
        }
        if (abs(distance_error) < err_threshold && abs(angle1_error) < err_threshold){
            v_ang = 0.0;
            v_lin = 0.0;
            move_search = false;
            std_msgs::msg::Empty empty;
            pub_waypoint_complete_->publish(empty);
            waypoint_completed = true;
        }

        // publish to cmd_vel topic (same as teleop topic)
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = v_ang;
        msg.linear.x = v_lin;
        msg.linear.y = 0.0;
        pub_cmd_vel_->publish(msg);
    }


    //////////////////////////////////////////////////////////////////////

    if (reconstruct_graph == false && first == false && (abs(x_trans) > 0.1 || abs(y_trans) > 0.1 || abs(theta_r) > 0.785375 || ((abs(x_trans) > 0.05 || abs(y_trans) > 0.05) && feature_transform) || (search && waypoint_completed))){

        search = false;
        waypoint_completed = false;
        feature_transform = false;
        update_pos = false;

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
        T01.x_w = (T1_w.matrix()(0,3));

        T01.y.push_back(T0.translation()(1));
        T01.y.push_back(T1.translation()(1));
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

  void loop_closure_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    geometry_msgs::msg::Point point_pub;
    point_pub.x = robot.x_get();
    point_pub.y = robot.y_get();

    pub_rob_pose_->publish(point_pub);
  }


  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    turtlelib::Phi delta_p = {
        static_cast<double>(msg->position.at(0)) - robot.phi_r_get(),
        static_cast<double>(msg->position.at(1)) - robot.phi_l_get(),
        };

    if (first){
        phi_sub = delta_p;

        rot_0.row(0) << cos(0.0), -sin(0.0), 0.0;
        rot_0.row(1) << sin(0.0), cos(0.0), 0.0;
        rot_0.row(2) << 0.0, 0.0, 1.0;
        trans_0(0) = 0.0;
        trans_0(1) = 0.0;
        trans_0(2) = 0.0;
        first_flag = true;
    }
    else{
        delta_p.r -= phi_sub.r;
        delta_p.l -= phi_sub.l;

        robot.Forward_Kin(delta_p);
    }
    first = false;
  }

  void odom_update_callback(const img_transform::msg::Odom::SharedPtr msg)
  {
    update = msg->update;
    if (update){
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
        Eigen::Vector3d euler_update = T0_w_update.rotationMatrix().eulerAngles(2,1,0);
        double theta_new = euler_update(0);

        robot = {track_width, wheel_radius, {robot.phi_r_get(), robot.phi_l_get()}, {theta_new, T0_w_update.translation()(0) + x_sub, T0_w_update.translation()(1) + y_sub}};
    }
  }

  void reconstruct(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
    {
        reconstruct_graph = true;
    }


  /// initialize all publishers, subscribers, and services
  double wheel_radius, motor_cmd_max, track_width, motor_cmd_per_rad_sec, collision_radius, encoder_ticks_per_rad, dx_cam;
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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_loop_closure_;
  rclcpp::Subscription<img_transform::msg::Waypoint>::SharedPtr sub_next_waypoint_;
  rclcpp::Subscription<img_transform::msg::Waypoint>::SharedPtr sub_search_waypoint_;
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

  bool first_search = false;
  double x_des_search = 0.0;
  double y_des_search = 0.0;
  double theta_des_search = 0.0;

  bool next_waypoint = false;
  bool move_waypoint = false;
  bool move_search = false;
  bool loop_back = false;
  bool search = false;
  bool waypoint_completed = false;

  double sub_err_x = 0.0;
  double sub_err_y = 0.0;

  geometry_msgs::msg::Pose waypoint;
  geometry_msgs::msg::Pose search_waypoint;
  bool searching = false;

  int count_updates = 0;

  std_msgs::msg::Header header;

  bool feature_transform = false;

  visualization_msgs::msg::MarkerArray node_markers;
  visualization_msgs::msg::MarkerArray edge_markers;

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

  bool v_lin_neg = false;

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

  double distance_error = 10.0;
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