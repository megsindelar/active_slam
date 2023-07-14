#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "img_transform/transform.hpp"
#include "std_srvs/srv/empty.hpp"
#include "img_transform/msg/transform.hpp"
#include "img_transform/transform.hpp"
#include <vector>
#include <CPL-SLAM/CPL-SLAM.h>
#include <SESync/SESync.h>

#include "CPL-SLAM/CPL-SLAM_utils.h"

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



    ///////////////////////////////////////////////////////////////////////////////////////////
    // CPL_SLAM
    ///////////////////////////////////////////////////////////////////////////////////////////

    CPL_SLAM::measurements_t measurements;

    // Preallocate output
    CPL_SLAM::measurements_pose_t &measurements_pose = measurements.first;
    CPL_SLAM::measurements_landmark_t &measurements_landmark = measurements.second;
    size_t &num_poses = measurements.num_poses;
    size_t &num_landmarks = measurements.num_landmarks;

    CPL_SLAM::RelativePoseMeasurement measurement_pose;
    CPL_SLAM::RelativeLandmarkMeasurement measurement_landmark;

    // Preallocate various useful quantities
    CPL_SLAM::Scalar dx, dy, dz, dtheta, dqx, dqy, dqz, dqw, I11, I12, I13, I14, I15, I16,
        I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66;

    CPL_SLAM::Scalar ss, cc;

    size_t i, j;

    num_poses = 0;
    num_landmarks = 0;

    std::unordered_map<size_t, size_t> poses;
    std::unordered_map<size_t, size_t> landmarks;

    for (int i = 0; i < edges.size(); i++){
        // Extract formatted output
        img_transform::Edge edge;
        i = edge.id_a;
        j = edge.id_b;
        dx = edge.transform.x;
        dy = edge.transform.y;
        dtheta = edge.transform.theta;
        I11 = edge.info_matrix(0,0);
        I12 = edge.info_matrix(0,1);
        I13 = edge.info_matrix(0,2);
        I22 = edge.info_matrix(1,0);
        I23 = edge.info_matrix(1,1);
        I33 = edge.info_matrix(1,2);


        if (poses.insert({i, num_poses}).second) num_poses++;

        if (poses.insert({j, num_poses}).second) num_poses++;

        // Fill in elements of this measurement

        // Pose ids
        measurement_pose.i = poses[i];
        measurement_pose.j = poses[j];

        // Raw measurements
        sincos(dtheta, &ss, &cc);
        measurement_pose.t = CPL_SLAM::Complex(dx, dy);
        measurement_pose.R = CPL_SLAM::Complex(cc, ss);
        measurement_pose.dx = dx;
        measurement_pose.dy = dy;
        measurement_pose.dtheta = dtheta;

        Eigen::Matrix<CPL_SLAM::Scalar, 2, 2> TranCov;
        TranCov << I11, I12, I12, I22;
        measurement_pose.tau = 2 / TranCov.inverse().trace();

        measurement_pose.kappa = 2 * I33;

        // Update maximum value of poses found so far
        measurements_pose.push_back(measurement_pose);
    }

    CPL_SLAM::CPL_SLAMOpts opts;
    opts.r0 = 2;
    opts.verbose = true;  // Print output to stdout
    opts.reg_Cholesky_precon_max_condition_number = 2e6;
    // opts.num_threads = 4;

    CPL_SLAM::CPL_SLAMResult results = CPL_SLAM::CPL_SLAM(measurements, opts);

    // this is the updated robot pose
    // results.xhat;

    // CPL_SLAM::RealMatrix X(
    //     2, measurements.num_landmarks + 3 * measurements.num_poses);

    // const size_t size = measurements.num_landmarks + measurements.num_poses;

    // const CPL_SLAM::RealVector c = results.xhat.real();
    // const CPL_SLAM::RealVector s = results.xhat.imag();

    // X.topLeftCorner(1, size) = c.head(size).transpose();
    // X.bottomLeftCorner(1, size) = s.head(size).transpose();

    // for (size_t n = 0; n < measurements.num_poses; n++) {
    //     size_t j = size + 2 * n;
    //     size_t k = size + n;
    //     X(0, j) = c[k];
    //     X(1, j) = s[k];
    //     X(0, j + 1) = -s[k];
    //     X(1, j + 1) = c[k];
    // }

    // Eigen::Matrix<size_t, 2, Eigen::Dynamic> edge_s;

    // edge_s.setZero(2, measurements.first.size());

    // for (size_t e = 0; e < measurements.first.size(); e++) {
    //     edge_s(0, e) = measurements.first[e].i;
    //     edge_s(1, e) = measurements.first[e].j;
    // }

    // edge_s.setZero(2, measurements.second.size());

    // for (size_t e = 0; e < measurements.second.size(); e++) {
    //     edge_s(0, e) = measurements.second[e].i;
    //     edge_s(1, e) = measurements.second[e].j;
    // }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // CPL_SLAM - done
    ///////////////////////////////////////////////////////////////////////////////////////////


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
    edge.type = "EDGE_SE2";
    edge.info_matrix = information_matrix;
    edges.push_back(edge);
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
    vertex.type = "VERTEX_SE2";
    vertices.push_back(vertex);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_transform_;
  rclcpp::Subscription<img_transform::msg::Transform>::SharedPtr sub_robot_state_;

  img_transform::Vector2D transform;
  img_transform::Vector2D rob_state;
  std::vector<img_transform::Edge> edges;
  std::vector<img_transform::Vertex> vertices{{"VERTEX_SE2", 0, {0.0, 0.0, 0.0}}};
  int id_trans = 0;
  int id_rob_state = 0;
  int a = 1;
  CPL_SLAM::measurements_t measurements;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CPL_SLAM_NODE>());
  rclcpp::shutdown();
  return 0;
}
