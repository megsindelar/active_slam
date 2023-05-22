// SOURCE: https://github.com/sunzuolei/orb/blob/master/feature_extration.cpp

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include "matplotlibcpp.h"
#include <random>
#include <chrono>

using namespace std;
using namespace cv;
namespace plt = matplotlibcpp;

// Macro constants for generating noise and outliers
#define NOISE_BOUND 0.05
#define N_OUTLIERS 1700
#define OUTLIER_TRANSLATION_LB 5
#define OUTLIER_TRANSLATION_UB 10



inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
  return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

void addNoiseAndOutliers(Eigen::Matrix<double, 3, Eigen::Dynamic>& tgt) {
  // Add uniform noise
  Eigen::Matrix<double, 3, Eigen::Dynamic> noise =
      Eigen::Matrix<double, 3, Eigen::Dynamic>::Random(3, tgt.cols()) * NOISE_BOUND;
  NOISE_BOUND / 2;
  tgt = tgt + noise;

  // Add outliers
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis2(0, tgt.cols() - 1); // pos of outliers
  std::uniform_int_distribution<> dis3(OUTLIER_TRANSLATION_LB,
                                       OUTLIER_TRANSLATION_UB); // random translation
  std::vector<bool> expected_outlier_mask(tgt.cols(), false);
  for (int i = 0; i < N_OUTLIERS; ++i) {
    int c_outlier_idx = dis2(gen);
    assert(c_outlier_idx < expected_outlier_mask.size());
    expected_outlier_mask[c_outlier_idx] = true;
    tgt.col(c_outlier_idx).array() += dis3(gen); // random translation
  }
}


int main ( int argc, char** argv )
{
    auto begin_total = std::chrono::high_resolution_clock::now();
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    // read images
    Mat img_1 = imread ( argv[1], IMREAD_COLOR );
    Mat img_2 = imread ( argv[2], IMREAD_COLOR );
    // imshow ( "original image 1", img_1);
    // imshow ( "original image 2", img_2);

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     std::cout << "matches k1: " << keypoints_1.at(i).pt.x << std::endl;
    // }

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow("keypoint detection image",outimg1);
    Mat outimg2;
    drawKeypoints( img_2, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    Mat outimg3;
    drawKeypoints( img_1, keypoints_2, outimg3, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

    vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    double min_dist=10000, max_dist=0;

    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     double dist = matches[i].distance;
    //     if ( dist < min_dist ) min_dist = dist;
    //     if ( dist > max_dist ) max_dist = dist;
    // }

    // printf ( "-- Max dist : %f \n", max_dist );
    // printf ( "-- Min dist : %f \n", min_dist );

    min_dist = 10.0;

    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= min ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
            // std::cout << "matches q: " << matches[i].queryIdx << std::endl;
            // std::cout << "matches t: " << matches[i].trainIdx << std::endl;
            // std::cout << "matches k1: " << keypoints_1.at(matches[i].trainIdx) << std::endl;
            // std::cout << "matches k2: " << keypoints_2.at(matches[i].trainIdx) << std::endl;
        }
    }

    std::cout << "num of matches: " << good_matches.size() << std::endl;

    // Mat img_match;
    // Mat img_goodmatch;
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    // imshow ( "image match", img_match );
    // imshow ( "good match", img_goodmatch );
    // imwrite("good_matches.jpg",img_goodmatch);
    // waitKey(0);



    float z = 1.0;
    float f_1 = 295.009696;
    float f_2 = 296.22399483;
    float p_1 = 195.515395;
    float p_2 = 131.035055713;
    std::vector<float> key1_xw;
    std::vector<float> key1_yw;
    std::vector<float> key1_zw;
    std::vector<float> key1_xp;
    std::vector<float> key1_yp;
    std::vector<float> key2_xw;
    std::vector<float> key2_yw;
    std::vector<float> key2_zw;
    std::vector<float> key2_xp;
    std::vector<float> key2_yp;
    // first get transform point by multiplying by camera matrix
    for (int i=0; i<good_matches.size(); i++){
        float k1_xw = (1/f_1)*(keypoints_1.at(good_matches[i].queryIdx).pt.x - p_1*z);
        float k1_yw = (1/f_2)*(keypoints_1.at(good_matches[i].queryIdx).pt.y - p_2*z);
        float k2_xw = (1/f_1)*(keypoints_2.at(good_matches[i].trainIdx).pt.x - p_1*z);
        float k2_yw = (1/f_2)*(keypoints_2.at(good_matches[i].trainIdx).pt.y - p_2*z);
        key1_xw.push_back(k1_xw);
        // std::cout << "x1: " << k1_xw << std::endl;
        key1_yw.push_back(k1_yw);
        key1_zw.push_back(z);
        key1_xp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.x);
        key1_yp.push_back(keypoints_1.at(good_matches[i].queryIdx).pt.y);
        key2_xw.push_back(k2_xw);
        key2_yw.push_back(k2_yw);
        key2_zw.push_back(z);
        key2_xp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.x);
        key2_yp.push_back(keypoints_2.at(good_matches[i].trainIdx).pt.y);
    }

    // plt::plot(key1_xw, key1_yw, {{"color", "blue"}, {"marker", "o"}, {"linestyle", " "}});
    // // plt::plot(key1_xp, key1_yp, {{"color", "green"}, {"marker", "o"}, {"linestyle", " "}});
    // plt::plot(key2_xw, key2_yw, {{"color", "red"}, {"marker", "o"}, {"linestyle", " "}});
    // // plt::plot(key2_xp, key2_yp, {{"color", "yellow"}, {"marker", "o"}, {"linestyle", " "}});
    // plt::show();


    // // Load the .ply file
    // teaser::PLYReader reader;
    // teaser::PointCloud src_cloud;
    // auto status = reader.read("./example_data/bun_zipper_res3.ply", src_cloud);
    // int N = src_cloud.size();

    // Convert the point cloud to Eigen
    int N = good_matches.size();

    // std::vector<float> key_xw = key1_xw;
    // std::vector<float> key_yw = key1_yw;
    // std::vector<float> key_zw = key1_zw;
    // for (int i = 0; i < N; i++){
    //     // key_xw.push_back(key1_xw.at(i));
    //     // key_xw.push_back(key1_xw.at(i));
    //     // key_xw.push_back(key1_xw.at(i));
    //     key_xw.push_back(key2_xw.at(i));
    //     key_yw.push_back(key2_yw.at(i));
    //     key_zw.push_back(key2_zw.at(i));
    // }
    // for (int i = 0; i < 2*N; i++){
    //     std::cout << "x: " << key_xw.at(i) << std::endl;
    // }


    Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
    for (size_t i = 0; i < N; ++i) {
        // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
        src.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
        std::cout << "z1: " << key1_zw.at(i) << std::endl;
    }

    // Homogeneous coordinates
    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    src_h.resize(4, src.cols());
    src_h.topRows(3) = src;
    src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

    // Apply an arbitrary SE(3) transformation
    Eigen::Matrix4d T;
    // clang-format off
    // T << 9.96926560e-01,  6.68735757e-02, -4.06664421e-02, -1.15576939e-01,
    //     -6.61289946e-02, 9.97617877e-01,  1.94008687e-02, -3.87705398e-02,
    //     4.18675510e-02, -1.66517807e-02,  9.98977765e-01, 1.14874890e-01,
    //     0,              0,                0,              1;
    // clang-format on

    // Apply transformation
    // Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
    // Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt(3, N);
    for (size_t i = 0; i < N; ++i) {
        // src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
        tgt.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
        std::cout << "z2: " << key2_zw.at(i) << std::endl;
    }

    // Add some noise & outliers
    // addNoiseAndOutliers(tgt);

    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    solver.solve(src, tgt);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    auto solution = solver.getSolution();

    // Compare results
    std::cout << "=====================================" << std::endl;
    std::cout << "          TEASER++ Results           " << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Expected rotation: " << std::endl;
    std::cout << T.topLeftCorner(3, 3) << std::endl;
    std::cout << "Estimated rotation: " << std::endl;
    std::cout << solution.rotation << std::endl;
    std::cout << "Error (deg): " << getAngularError(T.topLeftCorner(3, 3), solution.rotation)
                << std::endl;
    std::cout << std::endl;
    std::cout << "Expected translation: " << std::endl;
    std::cout << T.topRightCorner(3, 1) << std::endl;
    std::cout << "Estimated translation: " << std::endl;
    std::cout << solution.translation << std::endl;
    std::cout << "Error (m): " << (T.topRightCorner(3, 1) - solution.translation).norm() << std::endl;
    std::cout << std::endl;
    std::cout << "Number of correspondences: " << N << std::endl;
    std::cout << "Number of outliers: " << N_OUTLIERS << std::endl;
    std::cout << "Time taken (s): "
                << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                    1000000.0
                << std::endl;





    // draw transformation on first image to visualize
    std::vector<float> trans_x;
    Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts(3, N);
    Eigen::Matrix<double, 3, Eigen::Dynamic> keypnts2(3, N);
    // std::cout << "sizes: " << key1_xw.size() << " " << key1_yw.size() << " " << key1_zw.size() << " " << N << std::endl;
    for(int i = 0; i < N; i++){
        // std::cout << "i " << i << std::endl;
        keypnts.col(i) << key1_xw.at(i), key1_yw.at(i), key1_zw.at(i);
        keypnts2.col(i) << key2_xw.at(i), key2_yw.at(i), key2_zw.at(i);
    }
    Eigen::Matrix<double, 3, 3> transformation;
    // std::cout << "rotation 0" << solution.rotation(0,0) << std::endl;
    for (int i = 0; i < 2; i++){
        // std::cout << solution.rotation(i,0) << " " << solution.rotation(i,1) << " " << solution.translation(i) << std::endl;
        transformation.row(i) << solution.rotation(i,0), solution.rotation(i,1), solution.translation(i);
    }
    transformation.row(2) << solution.rotation(2,0), solution.rotation(2,1), solution.rotation(2,2);
    std::cout << transformation << std::endl;

    std::vector<KeyPoint> key_trans_2 = keypoints_2;
    std::vector<KeyPoint> key_trans_1 = keypoints_1;
    for(int i = 0; i < N; i++){
        // std::cout << "trans " << transformation << std::endl;
        // std::cout << "inv trans " << transformation.inverse() << std::endl;
        // std::cout << "keypnts col " << keypnts.col(i) << std::endl;
        Eigen::Matrix<double, 3, 1> new_2 = transformation.inverse()*keypnts.col(i);
        Eigen::Matrix<double, 3, 1> new_1 = transformation*keypnts2.col(i);
        // trans_x.push_back(x);
        // // trans_y.push_back(y);
        // std::cout << "new 2: " << new_2 << std::endl;
        // std::cout << "x: " << new_2(0) << std::endl;
        // std::cout << "y: " << new_2(1) << std::endl;
        key_trans_2.at(i).pt.x = new_2(0);
        // std::cout << "x: " << key_trans_2.at(i).pt.x << std::endl;
        key_trans_2.at(i).pt.y = new_2(1);
        key_trans_1.at(i).pt.x = new_1(0);
        key_trans_1.at(i).pt.y = new_1(1);
    }

    // Mat img_match;
    // Mat img_goodmatch;
    Mat outimg_test_2;
    drawKeypoints ( img_2, key_trans_2, outimg_test_2);
    Mat outimg_test_1;
    drawKeypoints ( img_2, key_trans_2, outimg_test_1);
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    // imshow ( "image match", img_match );
    // imshow ("transformed keypoints (keypoint 1 to image 2)", outimg_test_2);
    // imshow ("transformed keypoints (keypoint 2 to image 1)", outimg_test_1);
    // imshow("keypoint 1 detection image 1", outimg1);
    // imshow("keypoint 2 detection image 2", outimg2);
    // imshow("keypoint 2 on detection image 1", outimg3);
    // imwrite("good_matches.jpg",outimg_test);
    // waitKey(0);

    // compute entire time of program
    // later, add optimization to cmake to make program time faster
    // assemble robot with cam facing down

    // Mat outimg_test;
    // drawKeypoints( img_1, keypoints_1, outimg_test, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // imshow("keypoint detection image",outimg1);
    // imshow("keypoint2 after translated image",outimg_test);

    auto end_total = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end_total - begin_total);
    printf("Time to complete program: %.3f seconds\n", elapsed.count()*1e-9);


    return 0;
}