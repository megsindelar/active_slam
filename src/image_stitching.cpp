
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>

int main( int argc, char** argv ) {
\
    auto begin_total = std::chrono::high_resolution_clock::now();
    if ( argc != 3 )
    {
        std::cout<<"usage: image_stitching before_file after_file"<<std::endl;
        return 1;
    }

    cv::FileStorage fs;
    fs.open(argv[2], cv::FileStorage::READ);

    cv::FileStorage fs_before;
    fs_before.open(argv[1], cv::FileStorage::READ);

    int num = (int) fs["num"];

    std::cout << "num: " << num << std::endl;

    cv::Mat image1 = fs["image0"].mat();

    double theta = 0.0;
    double rotation_angle = 0.0;
    double delta_angle = 0.0;
    int img2_x = 0;
    int img2_y = 0;
    double curr_pos_x = 0.0;
    double curr_pos_y = 0.0;
    int ind_loop = 0;
    bool loop = false;

    int loop_id_1 = num-1;

    for (int i = 0; i < num; i++){

        double window_col = 0.07629;
        double window_row = 0.07429;

        cv::Mat image2 = fs["image" + std::to_string(i+1)].mat();

        double translation_x = (double) fs["transform_x" + std::to_string(i)];
        double translation_y = (double) fs["transform_y" + std::to_string(i)];

        theta = -(double) fs_before["transform_theta" + std::to_string(i)];

        rotation_angle += theta;

        if (i == num-1){
            ind_loop = 0;
            loop = true;
        }


        // Compute the rotation matrix
        cv::Point2f center(image2.cols / 2.0, image2.rows / 2.0); 
        cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, rotation_angle, 1.0);

        // pixel per meter
        double ppm_col = image2.cols/window_col;
        double ppm_row = image2.rows/window_row;

        // Apply translation to the rotation matrix
        rotation_matrix.at<double>(0, 2) += translation_x; //*ppm_row;
        rotation_matrix.at<double>(1, 2) += translation_y; //*ppm_col;

        cv::Size rotatedSize(image2.cols, image2.rows);

        cv::Mat warped_image2;
        cv::warpAffine(image2, warped_image2, rotation_matrix, rotatedSize);

        // Calculate dimensions of the stitched image

        // stitching vertically
        int stitched_width = (image1.cols + abs(translation_x)*ppm_row);
        int stitched_height = (image1.rows + abs(translation_y)*ppm_col);

        std::cout << "ppm: row, col " << ppm_row << " " << ppm_col << std::endl;
        std::cout << "image1: row, col " << image1.rows << " " << image1.cols << std::endl;
        std::cout << "together: row, col " << translation_x*ppm_row << " " << translation_y*ppm_col << std::endl;
        std::cout << "stitched dim: row, col " << stitched_width << " " << stitched_height << std::endl;

        int stitch_x = 0;
        int stitch_y = 0;
        int t_x = 0;
        int t_y = 0;

        cv::Mat stitched_image(stitched_height, stitched_width, image1.type());
        if (translation_x > 0.0 && translation_y > 0.0){
            std::cout << "Test 1" << std::endl;
            stitch_x = 0;
            stitch_y = 0;
            t_x = curr_pos_x + translation_x*ppm_row;
            t_y = curr_pos_y + translation_y*ppm_col;
            std::cout << "t_x, t_y: " << t_x << " " << t_y << std::endl;
            curr_pos_x = t_x;
            curr_pos_y = t_y;
        }
        else if (translation_x < 0.0 && translation_y > 0.0){
            std::cout << "Test 2" << std::endl;
            stitch_x = (int)(abs(translation_x*ppm_row));
            stitch_y = 0;
            t_x = curr_pos_x + translation_x*ppm_row;
            t_y = curr_pos_y + translation_y*ppm_col;
            curr_pos_x += 0.0;
            curr_pos_y = t_y;
        }
        else if (translation_x > 0.0 && translation_y < 0.0){
            std::cout << "Test 3" << std::endl;
            stitch_x = 0;
            stitch_y = (int)(abs(translation_y*ppm_col));
            t_x = curr_pos_x + translation_x*ppm_row;
            t_y = curr_pos_y + translation_y*ppm_row;
            curr_pos_x = t_x;
            curr_pos_y += 0.0;
        }
        else{
            std::cout << "Test 4" << std::endl;
            stitch_x = (int)(abs(translation_x*ppm_row));
            stitch_y = (int)(abs(translation_y*ppm_col));
            t_x = curr_pos_x + translation_x*ppm_row;
            t_y = curr_pos_y + translation_y*ppm_col;
            curr_pos_x += 0.0;
            curr_pos_y += 0.0;
        }

        image1.copyTo(stitched_image(cv::Rect(stitch_x, stitch_y, image1.cols, image1.rows)));
        warped_image2.copyTo(stitched_image(cv::Rect(stitch_x + t_x, stitch_y + t_y, warped_image2.cols, warped_image2.rows)));

        std::cout << "curr pos: x, y " << curr_pos_x << " " << curr_pos_y << std::endl;

        image1 = stitched_image;
    }

    // Display the stitched image
    cv::Mat stitched_resized;
    cv::resize(image1, stitched_resized, cv::Size(image1.cols/8.0, image1.rows/8.0), 0, 0, cv::INTER_LINEAR);
    std::cout << "stitched_resized: row, col " << stitched_resized.rows << " " << stitched_resized.cols << std::endl;
    cv::imshow("Stitched Image", stitched_resized);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}