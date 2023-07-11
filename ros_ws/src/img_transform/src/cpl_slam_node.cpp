#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "img_transform/transform.hpp"
#include "std_srvs/srv/empty.hpp"
#include "img_transform/msg/transform.hpp"
#include "img_transform/transform.hpp"
#include <vector>

class CPL_SLAM_NODE : public rclcpp::Node
{
public:
  CPL_SLAM_NODE()
  : Node("cpl_slam_node")
  {
    /// \brief timer callback with a specified frequency rate
    /// \param rate - frequency of timer callback in Hz
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&CPL_SLAM_NODE::timer_callback, this));

    // subscriber to transform between two frames
    sub_transform_ = this->create_subscription<img_transform::msg::Transform>(
            "/transform", 10,
            std::bind(&CPL_SLAM_NODE::transform_callback, this, std::placeholders::_1)
    );

    // subscriber to robot state
    sub_robot_state_ = this->create_subscription<img_transform::msg::Transform>(
            "/robot_state", 10,
            std::bind(&CPL_SLAM_NODE::robot_state_callback, this, std::placeholders::_1)
    );
  }

private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {

  }

  void transform_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    id_trans = msg->id;
    Eigen::Matrix<double, 4, 4> transformation;
    transformation.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    transformation.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    transformation.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    transformation.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);
    transform = img_transform::trans_to_vec(transformation);

    // Identity matrix times penalty weight
    Eigen::Matrix<double, 2, 3> information_matrix;
    information_matrix.row(0) << a*1, 0, 0;
    information_matrix.row(1) << a*1, 0, a*1;

    img_transform::Edge edge;
    edge.id_a = id_trans - 1;
    edge.id_b = id_trans;
    edge.transform = transform;

  }

  void robot_state_callback(
    const img_transform::msg::Transform::ConstSharedPtr& msg
  ){
    id_rob_state = msg->id;
    Eigen::Matrix<double, 4, 4> state;
    state.row(0) << msg->row_1.at(0), msg->row_1.at(1), msg->row_1.at(2), msg->row_1.at(3);
    state.row(1) << msg->row_2.at(0), msg->row_2.at(1), msg->row_2.at(2), msg->row_2.at(3);
    state.row(2) << msg->row_3.at(0), msg->row_3.at(1), msg->row_3.at(2), msg->row_3.at(3);
    state.row(3) << msg->row_4.at(0), msg->row_4.at(1), msg->row_4.at(2), msg->row_4.at(3);
    rob_state = img_transform::trans_to_vec(state);

    img_transform::Vertex vertex;
    vertex.id = id_rob_state;
    vertex.state = rob_state;
    vertices.push_back(vertex);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_robot_state_;

  img_transform::Vector2D transform;
  img_transform::Vector2D rob_state;
  std::vector<img_transform::Edge> edges;
  std::vector<img_transform::Vertex> vertices{{0, {0.0, 0.0, 0.0}}};
  int id_trans = 0;
  int id_rob_state = 0;
  int a = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CPL_SLAM_NODE>());
  rclcpp::shutdown();
  return 0;
}
