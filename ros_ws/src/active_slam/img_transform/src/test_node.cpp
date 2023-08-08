#include "rclcpp/rclcpp.hpp"
#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"

#include <fstream>

// #ifdef GPERFTOOLS
// #include <gperftools/profiler.h>
// #endif

using namespace std;
using namespace SESync;

bool write_poses = false;

class SE_SYNCTest : public rclcpp::Node
{
public:
  SE_SYNCTest()
  : Node("se_synctest")
  {
    this->declare_parameter("frequency", 100);
    int rate_ms = 1000 / (this->get_parameter("frequency").get_parameter_value().get<int>());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&SE_SYNCTest::timer_callback, this));
  }
private:
  /// \brief timer callback running at a set frequency
  void timer_callback()
  {
    if (run){
        size_t num_poses = 0;
        // measurements_t measurements = read_g2o_file("/home/megsindelar/Final_Project/ros_ws/src/active_slam/img_transform/data/meg_data.g2o", num_poses);
        // std::string token = "EDGE_SE2";

        // Preallocate output vector
        measurements_t measurements;

        // A single measurement, whose values we will fill in
        SESync::RelativePoseMeasurement measurement;

        // A string used to contain the contents of a single line
        // std::string line;

        // A string used to extract tokens from each line one-by-one
        // std::string token;

        // Preallocate various useful quantities
        Scalar dx, dy, dz, dtheta, dqx, dqy, dqz, dqw, I11, I12, I13, I14, I15, I16,
            I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66;

        size_t i, j;

        num_poses = 0;


        // This is a 2D pose measurement

        /** The g2o format specifies a 2D relative pose measurement in the
         * following form:
         *
         * EDGE_SE2 id1 id2 dx dy dtheta, I11, I12, I13, I22, I23, I33
         *
         */

        i = 0;
        j = 1;
        dx = 0.0;
        dy = 0.0;
        dtheta = 0.0249173;
        I11 = 50;
        I12 = 0;
        I13 = 0;
        I22 = 50;
        I23 = 0;
        I33 = 100;


        // Extract formatted output

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i;
        measurement.j = j;

        // Raw measurements
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;


        // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // num_poses = ((max_pair > num_poses) ? max_pair : num_poses);
        measurements.push_back(measurement);

        i = 0;
        j = 1;
        dx = 0.5;
        dy = 0.09;
        dtheta = 0.0249173;
        I11 = 50;
        I12 = 0;
        I13 = 0;
        I22 = 50;
        I23 = 0;
        I33 = 100;


        // Extract formatted output

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i;
        measurement.j = j;

        // Raw measurements
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;


        // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // num_poses = ((max_pair > num_poses) ? max_pair : num_poses);
        measurements.push_back(measurement);

        i = 0;
        j = 1;
        dx = 0.62;
        dy = 0.59;
        dtheta = 0.056;
        I11 = 5;
        I12 = 5;
        I13 = 0;
        I22 = 5;
        I23 = 0;
        I33 = 10;


        // Extract formatted output

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i;
        measurement.j = j;

        // Raw measurements
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;


        // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // num_poses = ((max_pair > num_poses) ? max_pair : num_poses);
        measurements.push_back(measurement);

        i = 0;
        j = 1;
        dx = 0.0;
        dy = 0.64;
        dtheta = 0.15;
        I11 = 50;
        I12 = 0;
        I13 = 0;
        I22 = 50;
        I23 = 0;
        I33 = 100;


        // Extract formatted output

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i;
        measurement.j = j;

        // Raw measurements
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;


        // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // num_poses = ((max_pair > num_poses) ? max_pair : num_poses);
        measurements.push_back(measurement);

        i = 0;
        j = 1;
        dx = 0.02;
        dy = 0.0;
        dtheta = 0.24;
        I11 = 50;
        I12 = 0;
        I13 = 0;
        I22 = 50;
        I23 = 0;
        I33 = 100;


        // Extract formatted output

        // Fill in elements of this measurement

        // Pose ids
        measurement.i = i;
        measurement.j = j;

        // Raw measurements
        measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        TranInfo << I11, I12, I12, I22;
        measurement.tau = 2 / TranInfo.inverse().trace();

        measurement.kappa = I33;


        // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // num_poses = ((max_pair > num_poses) ? max_pair : num_poses);
        measurements.push_back(measurement);

        // Preallocate output vector
        // measurements_t measurements;

        // A single measurement, whose values we will fill in
        // SESync::RelativePoseMeasurement measurement;

        // // A string used to contain the contents of a single line
        // std::string line;

        // // A string used to extract tokens from each line one-by-one
        // std::string token;

        // Preallocate various useful quantities
        // Scalar dx, dy, dz, dtheta, dqx, dqy, dqz, dqw, I11, I12, I13, I14, I15, I16,
        //     I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66;

        // size_t i, j;

        // num_poses = 0;
// 

        // This is a 2D pose measurement

        /** The g2o format specifies a 2D relative pose measurement in the
         * following form:
         *
         * EDGE_SE2 id1 id2 dx dy dtheta, I11, I12, I13, I22, I23, I33
         *
         */

        // i = 0;
        // j = 1;
        // dx = 0.974351;
        // dy = -0.014717;
        // dtheta = 0.0249173;
        // I11 = 50;
        // I12 = 50;
        // I13 = 0;
        // I22 = 0;
        // I23 = 50;
        // I33 = 100;


        // // Extract formatted output

        // // Fill in elements of this measurement

        // // Pose ids
        // measurement.i = i;
        // measurement.j = j;

        // // Raw measurements
        // measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        // measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        // TranInfo << I11, I12, I12, I22;
        // measurement.tau = 2 / TranInfo.inverse().trace();

        // measurement.kappa = I33;

        // RelativePoseMeasurement measurement;
        // i_ = 0;
        // j_ = 1;
        // dx = 0.974351;
        // dy = -0.014717;
        // dtheta = 0.0249173;
        // I11 = 50;
        // I12 = 50;
        // I13 = 0;
        // I22 = 0;
        // I23 = 50;
        // I33 = 100;

        // // Fill in elements of this measurement

        // // Pose ids
        // measurement.i = i_;
        // measurement.j = j_;

        // // Raw measurements
        // measurement.t.resize(2);
        // measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        // measurement.R.resize(2,2);
        // measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // Eigen::Matrix<Scalar, 2, 2> TranInfo;
        // TranInfo << I11, I12, I12, I22;
        // measurement.tau = 2 / TranInfo.inverse().trace();

        // measurement.kappa = I33;
        
        // // Update maximum value of poses found so far
        // size_t max_pair = std::max<size_t>(measurement.i, measurement.j);

        // measurements.push_back(measurement);



        // i_ = 1;
        // j_ = 2;
        // dx = 0.999423;
        // dy = 0.0164093;
        // dtheta = -0.00521262;

        // // Fill in elements of this measurement

        // // Pose ids
        // measurement.i = i_;
        // measurement.j = j_;

        // // Raw measurements
        // measurement.t.resize(2);
        // measurement.t = Eigen::Matrix<Scalar, 2, 1>(dx, dy);
        // measurement.R.resize(2,2);
        // measurement.R = Eigen::Rotation2D<Scalar>(dtheta).toRotationMatrix();

        // TranInfo << I11, I12, I12, I22;
        // measurement.tau = 2 / TranInfo.inverse().trace();

        // measurement.kappa = I33;
        
        // // Update maximum value of poses found so far
        // max_pair = std::max<size_t>(measurement.i, measurement.j);

        // measurements.push_back(measurement);

        cout << "Loaded " << measurements.size() << " measurements between "
            << endl;
        if (measurements.size() == 0) {
            cout << "Error: No measurements were read!"
                << " Are you sure the file exists?" << endl;
            exit(1);
        }

        SESyncOpts opts;
        opts.verbose = true; // Print output to stdout

        // Initialization method
        // Options are:  Chordal, Random
        opts.initialization = Initialization::Chordal;

        // Specific form of the synchronization problem to solve
        // Options are: Simplified, Explicit, SOSync
        opts.formulation = Formulation::Simplified;

        //   Initial
        opts.num_threads = 4;

        #ifdef GPERFTOOLS
        ProfilerStart("SE-Sync.prof");
        #endif

        //   / RUN SE-SYNC!
        SESyncResult results = SESync::SESync(measurements, opts);

        #ifdef GPERFTOOLS
        ProfilerStop();
        #endif

        if (write_poses) {
            // Write output
            string filename = "poses.txt";
            cout << "Saving final poses to file: " << filename << endl;
            ofstream poses_file(filename);
            poses_file << results.xhat;
            poses_file.close();
        }
        run = false; 
    }
    
    }

  rclcpp::TimerBase::SharedPtr timer_;
//   measurements_t measurements;
//   size_t i_ = 0;
//   size_t j_ = 1;
//   Scalar dx = 0.0;
//   Scalar dy = 0.0;
//   Scalar dtheta = 0.0;
//   Scalar I11 = 50;
//   Scalar I12 = 0;
//   Scalar I13 = 0;
//   Scalar I22 = 50;
//   Scalar I23 = 0;
//   Scalar I33 = 100;
  bool run = true;

};


int main(int argc, char * argv[]) {
//   if (argc != 2) {
//     cout << "Usage: " << argv[0] << " [input .g2o file]" << endl;
//     exit(1);
//   }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SE_SYNCTest>());
    try {
        rclcpp::spin(std::make_shared<SE_SYNCTest>());
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
        std::make_shared<SE_SYNCTest>()->get_logger(),
        "ERROR: " << e.what());
        rclcpp::shutdown();
    }
    rclcpp::shutdown();
    return 0;
}