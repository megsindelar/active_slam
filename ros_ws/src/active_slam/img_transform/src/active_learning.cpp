#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include "math.h"
#include "img_transform/msg/waypoint.hpp"

class ActiveLearning : public rclcpp::Node
{
public:
  ActiveLearning()
  : Node("active_learning")
  {
    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&ActiveLearning::timer_callback, this));

    // publish waypoints for turtlebot cam to move to
    pub_waypoint_ = this->create_publisher<img_transform::msg::Waypoint>("next_waypoint", 10);

    pub_search_waypoint_ = this->create_publisher<img_transform::msg::Waypoint>("search_waypoint", 10);

    pub_search_done_ = this->create_publisher<std_msgs::msg::Empty>("search_done", 10);

    // subscribe to a waypoint complete status that says when turtlebot reaches the waypoint
    sub_waypoint_status_ = this->create_subscription<std_msgs::msg::Empty>(
      "/waypoint_complete", 10, std::bind(
        &ActiveLearning::waypoint_status,
        this, std::placeholders::_1));

    // subscribe to a node to loop back to
    sub_loop_back_ = this->create_subscription<img_transform::msg::Waypoint>(
      "/loop_back", 10, std::bind(
        &ActiveLearning::loop_back_callback,
        this, std::placeholders::_1));

    pub_loop_back_ = this->create_publisher<img_transform::msg::Waypoint>("loop_back", 10);

    // subscribe to a start status to start turtlebot motion once images are published from cam
    sub_start_ = this->create_subscription<std_msgs::msg::Empty>(
      "/start_moving", 10, std::bind(
        &ActiveLearning::start_callback,
        this, std::placeholders::_1));

    sub_finish_loop_ = this->create_subscription<std_msgs::msg::Empty>(
      "/finish_loop", 10, std::bind(
        &ActiveLearning::finish_loop_callback,
        this, std::placeholders::_1));

    sub_test_vals_ = this->create_service<std_srvs::srv::Empty>(
        "/test_vals",
        std::bind(
            &ActiveLearning::test_vals_srv, this, std::placeholders::_1,
            std::placeholders::_2));

    // service to stop turtlebot motion, helpful for testing
    stop_moving_srv = this->create_service<std_srvs::srv::Empty>(
        "/stop_moving",
        std::bind(
            &ActiveLearning::stop_turtlebot, this, std::placeholders::_1,
            std::placeholders::_2));
  }

private:
  /// \brief timer callback running at a set frequency
  // publish waypoints for turtlebot cam to move to
  // topic: /next_waypoint   type: geometry_msgs::msg::Pose
  void timer_callback()
  {
    if (first){
        // initialize waypoints at 0
        waypoint.x = 0.1;
        waypoint.y = 0.0;
        waypoint.theta = 0.0;
        orig_x = waypoint.x;
        orig_y = waypoint.y;
        orig_theta = waypoint.theta;
        first = false;
    }
    if (start_sequence && next_waypoint){
        next_waypoint = false;
        // lawnmowing pattern trajectory
        if (waypoint_count == 1){
            x_pos += 0.4;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = M_PI/2.0;
            dx_next = 0.0;
        }
        else if (waypoint_count == 2){
            y_pos += 0.4;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = M_PI;
        }
        else if (waypoint_count == 3){
            x_pos -= 0.4;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = 3.0*M_PI/2.0;
        }
        else if (waypoint_count == 4){
            y_pos -= 0.4;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = 0.0;
            waypoint_count = -1;
            start_sequence = false;
            img_transform::msg::Waypoint square_loop;
            square_loop.x = orig_x;
            square_loop.y = orig_y;
            square_loop.theta = orig_theta;
            square_loop.loop = true;
            square_loop.search = false;
            pub_loop_back_->publish(square_loop);
        }
        // else if (waypoint_count == 5){
        //     waypoint.x += 0.0;
        //     waypoint.y += 0.0;
        //     waypoint.theta -= M_PI/2.0;
        //     waypoint_count = 0;
        // }
        waypoint.search = false;
        pub_waypoint_->publish(waypoint);
        waypoint_count++;
    }
    if (waypoint.loop && next_waypoint){
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        RCLCPP_INFO(rclcpp::get_logger("message"), "loop back");
        loop_back = true;
        // loop back to previous node
        pub_waypoint_->publish(waypoint);
        waypoint.loop = false;
        x_pos = waypoint.x;
        y_pos = waypoint.y;
        RCLCPP_INFO(rclcpp::get_logger("message"), "x_pos, y_pos %f, %f, %f: ", x_pos, y_pos);
        waypoint_count = 0;
        next_waypoint = false;
    }
    if (waypoint.search && next_waypoint && !search_done){
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "search");
        RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint count %d", waypoint_count);

        next_waypoint = false;

        // lawnmowing pattern trajectory
        if (waypoint_count == 0){
            waypoint.theta = waypoint.theta;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            RCLCPP_INFO(rclcpp::get_logger("message"), "orig_x: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "orig_y: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count < 3){
            waypoint.theta = waypoint.theta;
            waypoint.x += dx*cos(waypoint.theta);
            waypoint.y += dy*sin(waypoint.theta);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count == 3){
            waypoint.theta = orig_theta;
            waypoint.x = orig_x;
            waypoint.y = orig_y;
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x go back: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y go back: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count < 5){
            waypoint.theta = orig_theta - M_PI/6.0;
            waypoint.x += dx*cos(waypoint.theta);
            waypoint.y += dy*sin(waypoint.theta);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count == 5){
            waypoint.theta = orig_theta;
            waypoint.x = orig_x;
            waypoint.y = orig_y;
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x go back: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y go back: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count < 7){
            waypoint.theta = orig_theta + M_PI/6.0;
            waypoint.x += dx*cos(waypoint.theta);
            waypoint.y += dy*sin(waypoint.theta);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        else if (waypoint_count == 7){
            waypoint.theta = orig_theta;
            waypoint.x = orig_x;
            waypoint.y = orig_y;
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x go back: %f,", waypoint.x);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y go back: %f,", waypoint.y);
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
            waypoint.search = true;
        }

        // else if (waypoint_count < 12){
        //     waypoint.theta = orig_theta + M_PI/6.0;
        //     waypoint.x += dx*cos(waypoint.theta);
        //     waypoint.y += dy*sin(waypoint.theta);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x: %f,", waypoint.x);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y: %f,", waypoint.y);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
        //     waypoint.search = true;
        // }

        // else if (waypoint_count == 12){
        //     waypoint.theta = orig_theta;
        //     waypoint.x = orig_x;
        //     waypoint.y = orig_y;
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x go back: %f,", waypoint.x);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y go back: %f,", waypoint.y);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
        //     waypoint.search = true;
        // }

        // else if (waypoint_count < 15){
        //     waypoint.theta = orig_theta + M_PI/3.0;
        //     waypoint.x += dx*cos(waypoint.theta);
        //     waypoint.y += dy*sin(waypoint.theta);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x: %f,", waypoint.x);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y: %f,", waypoint.y);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
        //     waypoint.search = true;
        // }

        // else if (waypoint_count == 15){
        //     waypoint.theta = orig_theta;
        //     waypoint.x = orig_x;
        //     waypoint.y = orig_y;
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint x go back: %f,", waypoint.x);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint y go back: %f,", waypoint.y);
        //     RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint theta: %f,", waypoint.theta);
        //     waypoint.search = true;
        // }

        else{
            RCLCPP_INFO(rclcpp::get_logger("message"), "waypoint search is false:");
            waypoint.search == false;
            search_done = true;
            std_msgs::msg::Empty search_completed;
            pub_search_done_->publish(search_completed);
            loop_back = false;
        }

        pub_search_waypoint_->publish(waypoint);
        waypoint_count++;

    }
    if (finish_loop && next_waypoint){
        waypoint_count = prev_waypoint_count;
        next_waypoint = false;
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        RCLCPP_INFO(rclcpp::get_logger("message"), "finishing loop");
        finish_loop = false;
        prev_waypoint.search = false;
        next_waypoint = true;
        // loop back to previous node
        // pub_waypoint_->publish(prev_waypoint);
        // waypoint.x = prev_waypoint.x;
        // waypoint.y = prev_waypoint.y;
        // waypoint.theta = prev_waypoint.theta;

    }
    if (next_waypoint && waypoint.loop == false && waypoint.search == false && !finish_loop && !start_sequence){
        RCLCPP_INFO(rclcpp::get_logger("message"), "Next waypoint!");
        next_waypoint = false;
        search_done = false;

        // lawnmowing pattern trajectory
        if (waypoint_count == 1){
            x_pos += 0.3 + dx_next;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = M_PI/2.0;
            dx_next = 0.0;
        }
        else if (waypoint_count == 2){
            y_pos += 0.2;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = M_PI;
        }
        else if (waypoint_count == 3){
            x_pos -= 0.3;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = M_PI/2.0;
        }
        else if (waypoint_count == 4){
            y_pos += 0.3;
            waypoint.x = x_pos;
            waypoint.y = y_pos;
            waypoint.theta = 0.0;
            waypoint_count = 0;
            lawnmower_count++;
        }
        if (waypoint_count > 4){
            waypoint_count = 0;
        }
        if (lawnmower_count == 6){
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            RCLCPP_INFO(rclcpp::get_logger("message"), "NEXT LAWNMOWER SECTION!!!");
            dx_next = 0.5;
        }
        // else if (waypoint_count == 5){
        //     waypoint.x += 0.0;
        //     waypoint.y += 0.0;
        //     waypoint.theta -= M_PI/2.0;
        //     waypoint_count = 0;
        // }
        waypoint.search = false;
        pub_waypoint_->publish(waypoint);
        waypoint_count++;
    }
  }

  // subscribe to a waypoint complete status that says when turtlebot reaches the waypoint
  // topic: /waypoint_complete   type: std_msgs::msg::Empty
  void waypoint_status(std_msgs::msg::Empty::SharedPtr msg){
    if (loop_back){
        RCLCPP_INFO(rclcpp::get_logger("message"), "Waypoint search is true!");
        waypoint.search = true;
        loop_back = false;
        search_done = false;
    }
    next_waypoint = true;
  }

  // subscribe to a node to loop back to
  // topic: /loop_back   type: geometry_msgs::msg::Pose
  void loop_back_callback(img_transform::msg::Waypoint::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Prev waypoint %f, %f, %f: ", prev_waypoint.x, prev_waypoint.y, prev_waypoint.theta);
    loop_back = true;
    next_waypoint = true;
    prev_waypoint.x = waypoint.x;
    prev_waypoint.y = waypoint.y;
    prev_waypoint.theta = waypoint.theta;
    prev_waypoint_count = waypoint_count;
    // RCLCPP_INFO(rclcpp::get_logger("message"), "Loop waypoint %f, %f, %f: ", waypoint.x, waypoint.y, waypoint.theta);
    waypoint.x = msg->x;
    waypoint.y = msg->y;
    waypoint.theta = msg->theta;
    orig_x = msg->x;
    orig_y = msg->y;
    orig_theta = msg->theta;
    RCLCPP_INFO(rclcpp::get_logger("message"), "Loop waypoint %f, %f, %f: ", waypoint.x, waypoint.y, waypoint.theta);
    waypoint.loop = msg->loop;
  }

  // service to stop turtlebot motion, helpful for testing
  // topic: /stop_moving   type: std_srvs::srv::Empty
  void stop_turtlebot(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        RCLCPP_INFO(rclcpp::get_logger("message"), "Stop moving!");
        next_waypoint = false;
        loop_back = false;
    }

  void test_vals_srv(
        std_srvs::srv::Empty::Request::SharedPtr,
        std_srvs::srv::Empty::Response::SharedPtr)
    {
        // RCLCPP_INFO(rclcpp::get_logger("message"), "Stop moving!");
        next_waypoint = true;
        // loop_back = false;
    }

  // subscribe to a start status to start turtlebot motion once images are published from cam
  // topic: /start_moving   type: std_srvs::srv::Empty
  void start_callback(std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Start!");
    next_waypoint = true;
    start_sequence = true;
  }

  void finish_loop_callback(std_msgs::msg::Empty::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("message"), "Finish loop!");
    finish_loop = true;
    waypoint.loop = false;
    waypoint.search = false;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<img_transform::msg::Waypoint>::SharedPtr pub_waypoint_;
  rclcpp::Publisher<img_transform::msg::Waypoint>::SharedPtr pub_search_waypoint_;
  rclcpp::Subscription<img_transform::msg::Waypoint>::SharedPtr sub_loop_back_;
  rclcpp::Publisher<img_transform::msg::Waypoint>::SharedPtr pub_loop_back_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_waypoint_status_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_finish_loop_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_search_done_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr sub_test_vals_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_moving_srv;

  img_transform::msg::Waypoint waypoint;
  img_transform::msg::Waypoint prev_waypoint;

  int waypoint_count = 1;
  int lawnmower_count = 0;
  int prev_waypoint_count = 1;

  double x_pos = 0.0;
  double y_pos = 0.0;
  double dx = 0.05;
  double dy = 0.05;
  double dx_next = 0.0;

  bool start_sequence = false;

  double orig_x = 0.0;
  double orig_y = 0.0;
  double orig_theta = 0.0;

  bool next_waypoint = false;
  bool loop_back = false;
  bool search_done = false;
  bool finish_loop = false;
  bool first = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActiveLearning>());
  rclcpp::shutdown();
  return 0;
}