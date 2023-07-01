#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelControl : public rclcpp::Node
{
public:
  WheelControl()
  : Node("wheelcontrol")
  {
    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&WheelControl::timer_callback, this));

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  /// \brief timer callback running at a set frequency
  /// publish: /cmd_vel (geometry_msgs/msg/Twist)
  void timer_callback()
  {
    // max linear is 0.26 m/s and max angular is 1.82 m/s
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.0;
    msg.linear.x = 0.1;
    msg.linear.y = 0.0;
    pub_cmd_vel_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("message"), "Published");

  }

  /// initialize all publishers, timers, and services
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelControl>());
  rclcpp::shutdown();
  return 0;
}
