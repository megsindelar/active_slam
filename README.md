# Active learning using a graph-based SLAM using visual sensor measurements
* Megan Sindelar
* Spring/Summer 2023
* Portfolio link: [Portfolio](https://megsindelar.github.io/Active-SLAM.html)
# Package List
This repository consists of several ROS packages
- `turtlebot_control` - publishes video stream from the Raspberry Pi camera and receives controls for the turtlebot actions
- `img_transform` - subscribes to the video stream to feature match and find the SE(2) transformation between frames, 
                    control wheel odometry, and update nodes in the pose graph with the SE-Sync library

The goal of this project is to implement an Active SLAM method using odometry and visual sensor measurements to map and localize a turtlebot to recreate a map of a poster that it's driving over. A special focus is placed on the visual search method.


Click the link below to the README file for turtlebot_control:

[README](raspberry_pi_ws/src/turtlebot_control/README.md)


Click the link below to the README file for img_transform:

[README](ros_ws/src/active_slam/img_transform/README.md)


Here is a video below of the project:

[![YOUTUBE](https://img.youtube.com/vi/yB6os3cmF2M/0.jpg)](https://www.youtube.com/watch?v=yB6os3cmF2M)


When building this project, build the ros_ws package on a laptop, build the raspberry_pi_ws on the
turtlebot, and build the src separate scripts separately with the external CMakeLists.txt


## Required package links:
Teaser: https://github.com/MIT-SPARK/TEASER-plusplus

Bag of words: https://github.com/dorian3d/DBoW2

SESync: https://github.com/david-m-rosen/SE-Sync/tree/master

Optimization: https://github.com/david-m-rosen/Optimization/tree/6d87cc2e46a0ece2720c4b94de943641d9750e50

Image transport: 
sudo apt install ros-humble-compressed-image-transport
sudo apt install ros-humble-theora-image-transport

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
      used to stitch a panorama of images together (note, must pass in a before and after ext file of the images with transforms)


    -> using the CMakeLists.txt that is added, one can make a build directory
    and use cmake and make to build and run these four files


![turtlebot](https://github.com/megsindelar/active_slam/assets/87098227/77b49029-1ac1-4719-a477-6644a414a140)

