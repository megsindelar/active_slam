#include <CPL-SLAM/CPL-SLAM.h>
#include <SESync/SESync.h>

#include <fstream>

int main() {
  bool use_CPL = true;
  bool write = false;

//   if (argc < 2 || argc > 4) {
//     std::cout << "Usage: " << argv[0] << " [input .g2o file]" << std::endl;
//     exit(1);
//   }

//   if (argc >= 3) {
//     std::string alg(argv[2]);

//     if (alg == "CPL") {
//       use_CPL = true;
//     } else if (alg == "SE") {
//       use_CPL = false;
//     } else {
//       std::cout << "The second argument must be either \"CPL\" or \"SE\""
//                 << std::endl;
//       exit(1);
//     }
//   }

//   if (argc >= 4) {
//     write = true;
//   }

  size_t num_poses;


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
    CPL_SLAM::ComplexMatrix Y0;

    CPL_SLAM::CPL_SLAMResult results = CPL_SLAM::CPL_SLAM(measurements, opts);
}