#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"

class ActiveLearning : public rclcpp::Node
{
public:
  ActiveLearning()
  : Node("activelearning")
  {
    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&ActiveLearning::timer_callback, this));


    pub_waypoint_ = this->create_publisher<geometry_msgs::msg::Pose>("next_waypoint", 10);


    sub_waypoint_status_ = this->create_subscription<std_msgs::msg::Empty>(
      "/waypoint_complete", 10, std::bind(
        &ActiveLearning::waypoint_status,
        this, std::placeholders::_1));

    sub_loop_back_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/loop_back", 10, std::bind(
        &ActiveLearning::loop_back_callback,
        this, std::placeholders::_1));

    sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
      "/start_moving", 10, std::bind(
        &ActiveLearning::start_callback,
        this, std::placeholders::_1));

    stop_moving_srv = this->create_service<std_srvs::srv::Empty>(
        "/stop_moving",
        std::bind(
            &ActiveLearning::stop_turtlebot, this, std::placeholders::_1,
            std::placeholders::_2));

  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    if (first){
        waypoint.position.x = 0.0;
        waypoint.position.y = 0.0;
        waypoint.orientation.z = 0.0;
        waypoint.orientation.w = 1.0;
        first = false;
    }
    if (loop_back){
        loop_back = false;
        pub_waypoint_->publish(waypoint);

    }
    if (next_waypoint){
        RCLCPP_INFO(rclcpp::get_logger("message"), "Next waypoint!");
        next_waypoint = false;

        if (waypoint_count == 1){
            RCLCPP_INFO(rclcpp::get_logger("message"), "Move up!");
            waypoint.position.x += 0.6;
            waypoint.position.y += 0.0;
            waypoint.orientation.z += M_PI/2.0;
        }
        else if (waypoint_count == 2){
            waypoint.position.x += 0.0;
            waypoint.position.y += 0.2;
            waypoint.orientation.z += M_PI/2.0;
        }
        else if (waypoint_count == 3){
            waypoint.position.x -= 0.6;
            waypoint.position.y -= 0.0;
            waypoint.orientation.z += M_PI/2.0;
        }
        else if (waypoint_count == 4){
            waypoint.position.x -= 0.0;
            waypoint.position.y += 0.2;
            waypoint.orientation.z -= M_PI/2.0;
        }
        else if (waypoint_count == 5){
            waypoint.position.x += 0.0;
            waypoint.position.y += 0.0;
            waypoint.orientation.z -= M_PI/2.0;
            waypoint_count = 0;
        }

        pub_waypoint_->publish(waypoint);
        waypoint_count++;
    }
  }

  void waypoint_status(std_msgs::msg::Empty::SharedPtr msg){
    next_waypoint = true;
    loop_back = false;
  }

  void loop_back_callback(geometry_msgs::msg::Pose::SharedPtr msg){
    loop_back = true;
    next_waypoint = false;
    waypoint.position.x = msg->position.x;
    waypoint.position.y = msg->position.y;
    waypoint.orientation.z = msg->orientation.z;
  }

  void stop_turtlebot(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        RCLCPP_INFO(rclcpp::get_logger("message"), "Stop moving!");
        next_waypoint = false;
        loop_back = false;
    }

  void start_callback(std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Start!");
    next_waypoint = true;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_waypoint_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_loop_back_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_waypoint_status_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_moving_srv;

  geometry_msgs::msg::Pose waypoint;

  int waypoint_count = 1;

  bool next_waypoint = false;
  bool loop_back = false;
  bool first = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActiveLearning>());
  rclcpp::shutdown();
  return 0;
}
