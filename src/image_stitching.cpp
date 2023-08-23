
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>

// using namespace std;
// using namespace cv;

// bool divide_images = false;
// Stitcher::Mode mode = Stitcher::PANORAMA;
// vector<Mat> imgs;
// string result_name = "result.jpg";

// void printUsage(char** argv);
// int parseCmdArgs(int argc, char** argv);

// int main(int argc, char* argv[])
// {
//     int retval = parseCmdArgs(argc, argv);
//     if (retval) return EXIT_FAILURE;

//     //![stitching]
//     Mat pano;
//     Ptr<Stitcher> stitcher = Stitcher::create(mode);
//     Stitcher::Status status = stitcher->stitch(imgs, pano);

//     if (status != Stitcher::OK)
//     {
//         cout << "Can't stitch images, error code = " << int(status) << endl;
//         return EXIT_FAILURE;
//     }
//     //![stitching]

//     imwrite(result_name, pano);
//     cout << "stitching completed successfully\n" << result_name << " saved!";
//     return EXIT_SUCCESS;
// }


// void printUsage(char** argv)
// {
//     cout <<
//          "Images stitcher.\n\n" << "Usage :\n" << argv[0] <<" [Flags] img1 img2 [...imgN]\n\n"
//          "Flags:\n"
//          "  --d3\n"
//          "      internally creates three chunks of each image to increase stitching success\n"
//          "  --mode (panorama|scans)\n"
//          "      Determines configuration of stitcher. The default is 'panorama',\n"
//          "      mode suitable for creating photo panoramas. Option 'scans' is suitable\n"
//          "      for stitching materials under affine transformation, such as scans.\n"
//          "  --output <result_img>\n"
//          "      The default is 'result.jpg'.\n\n"
//          "Example usage :\n" << argv[0] << " --d3 --mode scans img1.jpg img2.jpg\n";
// }


// int parseCmdArgs(int argc, char** argv)
// {
//     if (argc == 1)
//     {
//         printUsage(argv);
//         return EXIT_FAILURE;
//     }

//     for (int i = 1; i < argc; ++i)
//     {
//         if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
//         {
//             printUsage(argv);
//             return EXIT_FAILURE;
//         }
//         else if (string(argv[i]) == "--d3")
//         {
//             divide_images = true;
//         }
//         else if (string(argv[i]) == "--output")
//         {
//             result_name = argv[i + 1];
//             i++;
//         }
//         else if (string(argv[i]) == "--mode")
//         {
//             if (string(argv[i + 1]) == "panorama")
//                 mode = Stitcher::PANORAMA;
//             else if (string(argv[i + 1]) == "scans")
//                 mode = Stitcher::SCANS;
//             else
//             {
//                 cout << "Bad --mode flag value\n";
//                 return EXIT_FAILURE;
//             }
//             i++;
//         }
//         else
//         {
//             Mat img = imread(samples::findFile(argv[i]));
//             if (img.empty())
//             {
//                 cout << "Can't read image '" << argv[i] << "'\n";
//                 return EXIT_FAILURE;
//             }

//             if (divide_images)
//             {
//                 Rect rect(0, 0, img.cols / 2, img.rows);
//                 imgs.push_back(img(rect).clone());
//                 rect.x = img.cols / 3;
//                 imgs.push_back(img(rect).clone());
//                 rect.x = img.cols / 2;
//                 imgs.push_back(img(rect).clone());
//             }
//             else
//                 imgs.push_back(img);
//         }
//     }
//     return EXIT_SUCCESS;
// }

int main() {
    cv::Mat image1 = cv::imread("image/image.jpg");
    cv::Mat image2 = cv::imread("image/image2.jpg");

    double rotation_angle = 20;
    double translation_x = -20;
    double translation_y = 10;

    // Compute the rotation matrix
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(0, 0), rotation_angle, 1);

    // Apply translation to the rotation matrix
    rotation_matrix.at<double>(0, 2) += translation_x;
    rotation_matrix.at<double>(1, 2) += translation_y;

    cv::Mat warped_image2;
    cv::warpAffine(image2, warped_image2, rotation_matrix, image2.size());


    // Calculate dimensions of the stitched image


    // stitching horizontally
    // int stitched_width = image1.cols + image2.cols;
    // int stitched_height = std::max(image1.rows, image2.rows);

    // cv::Mat stitched_image(stitched_height, stitched_width, image1.type());
    // image1.copyTo(stitched_image(cv::Rect(0, 0, image1.cols, image1.rows)));
    // warped_image2.copyTo(stitched_image(cv::Rect(image1.cols, 0, warped_image2.cols, warped_image2.rows)));


    // stitching vertically
    int stitched_width = std::max(image1.cols, image2.cols);
    int stitched_height = image1.rows + image2.rows;

    cv::Mat stitched_image(stitched_height, stitched_width, image1.type());
    image1.copyTo(stitched_image(cv::Rect(0, 0, image1.cols, image1.rows)));
    warped_image2.copyTo(stitched_image(cv::Rect(0, image1.rows, warped_image2.cols, warped_image2.rows)));


    // Display the stitched image
    cv::imshow("Stitched Image", stitched_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}