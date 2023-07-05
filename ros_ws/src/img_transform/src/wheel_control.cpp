#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "img_transform/transform.hpp"
#include "std_srvs/srv/empty.hpp"

// #define TURN_180 1.1
#define TURN_90 0.565
#define STRAIGHT_LINE 0.2

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

    stop_srv = this->create_service<std_srvs::srv::Empty>(
      "/stop",
      std::bind(
        &WheelControl::stop_moving, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  /// \brief timer callback running at a set frequency
  /// publish: /cmd_vel (geometry_msgs/msg/Twist)
  void timer_callback()
  {
    // max linear is 0.26 m/s and max angular is 1.82 m/s
    // zig zag pattern
    if (running == true)
    {
        auto begin_line = std::chrono::high_resolution_clock::now();
        auto end_line = std::chrono::high_resolution_clock::now();
        auto elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        while (elapsed_line < std::chrono::seconds(5))
        {
            // straight line
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = 0.0;
            msg.linear.x = STRAIGHT_LINE;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_line = std::chrono::high_resolution_clock::now();
            elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        }
        auto begin_turn = std::chrono::high_resolution_clock::now();
        auto end_turn = std::chrono::high_resolution_clock::now();
        auto elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        while (elapsed_turn < std::chrono::seconds(5))
        {
            // turn 90 deg
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = TURN_90;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_turn = std::chrono::high_resolution_clock::now();
            elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        }
        begin_line = std::chrono::high_resolution_clock::now();
        end_line = std::chrono::high_resolution_clock::now();
        elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        while (elapsed_line < std::chrono::seconds(1))
        {
            // straight line
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = 0.0;
            msg.linear.x = STRAIGHT_LINE;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_line = std::chrono::high_resolution_clock::now();
            elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        }
        begin_turn = std::chrono::high_resolution_clock::now();
        end_turn = std::chrono::high_resolution_clock::now();
        elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        while (elapsed_turn < std::chrono::seconds(5))
        {
            // turn 90 deg
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = TURN_90;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_turn = std::chrono::high_resolution_clock::now();
            elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        }



        begin_line = std::chrono::high_resolution_clock::now();
        end_line = std::chrono::high_resolution_clock::now();
        elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        while (elapsed_line < std::chrono::seconds(5))
        {
            // straight line
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = 0.0;
            msg.linear.x = STRAIGHT_LINE;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_line = std::chrono::high_resolution_clock::now();
            elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        }
        begin_turn = std::chrono::high_resolution_clock::now();
        end_turn = std::chrono::high_resolution_clock::now();
        elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        while (elapsed_turn < std::chrono::seconds(5))
        {
            // turn 90 deg
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = -TURN_90;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_turn = std::chrono::high_resolution_clock::now();
            elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        }
        begin_line = std::chrono::high_resolution_clock::now();
        end_line = std::chrono::high_resolution_clock::now();
        elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        while (elapsed_line < std::chrono::seconds(1))
        {
            // straight line
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = 0.0;
            msg.linear.x = STRAIGHT_LINE;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_line = std::chrono::high_resolution_clock::now();
            elapsed_line = std::chrono::duration_cast<std::chrono::seconds>(end_line - begin_line);
        }
        begin_turn = std::chrono::high_resolution_clock::now();
        end_turn = std::chrono::high_resolution_clock::now();
        elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        while (elapsed_turn < std::chrono::seconds(5))
        {
            // turn 90 deg
            auto msg = geometry_msgs::msg::Twist();
            msg.angular.z = -TURN_90;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            pub_cmd_vel_->publish(msg);
            end_turn = std::chrono::high_resolution_clock::now();
            elapsed_turn = std::chrono::duration_cast<std::chrono::seconds>(end_turn - begin_turn);
        }
        
    }
  }

  void stop_moving(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.0;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    pub_cmd_vel_->publish(msg);
    running = false;
  }

  /// initialize all publishers, timers, and services
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  bool running = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelControl>());
  rclcpp::shutdown();
  return 0;
}
