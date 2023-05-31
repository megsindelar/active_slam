# Active learning using a graph-based SLAM using only visual sensor measurements
* Megan Sindelar
* Spring/Summer 2023
# Package List
This repository consists of several ROS packages
- turtlebot_control - publishes video stream from the Raspberry Pi camera and receives controls for the turtlebot actions
- img_transform - subscribes to the video stream to feature match and find the SE(2) transformation between two frames

The goal of this project is to implement a novel approach to Ergodic Active SLAM using only visual sensor measurements to map and localize a turtlebot to recreate a map of a poster that it's driving over.


Separate scripts
- in the src folder at the base of this respository, there are several scripts that can run
specific testing files, including...
    - `camera_calibration.cpp`
      used to calibrate the camera

    - `capture_image.cpp`
      used to test whether the video stream on the Raspberry Pi is working

    - `feature_matching.cpp`
      used to test the feature matching between two images (note, must pass in two paths to two
      images when calling this file to work)

    - `image_stitching.cpp`
      used to stitch a panorama of images together (note, must pass in multiple paths or a file of
      paths of images when calling this file to work)


    -> using the CMakeLists.txt that is added, one can make a build directory
    and use cmake and make to build and run these four files