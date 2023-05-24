// #include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
using namespace cv;
using namespace std;
int main(int, char**)
{
    Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = -1;             // 0 = open default camera
    int apiID = cv::CAP_V4L;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    // for (;;)
    // {
    //     // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
        cerr << "ERROR! blank frame grabbed\n";
        // break;
    }

    vector<int>params;
    params.push_back(IMWRITE_PNG_COMPRESSION);
    params.push_back(9);
    bool image_saved = false;
    try{
        image_saved = imwrite("test_pic.png", frame, params);
    }
    catch (const Exception& ex)
    {
        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    }
    printf("image saved: %c\n", image_saved);
    if (image_saved)
    {
        printf("Image saved!");
    }
    else
    {
        printf("Image not saved :(");
    }
        // // show live and wait for a key with timeout long enough to show images
        // imshow("Live", frame);
        // if (waitKey(5) >= 0)
        //     break;
    // }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}