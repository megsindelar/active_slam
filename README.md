# Active learning using a graph-based SLAM using only visual sensor measurements
* Megan Sindelar
* Spring/Summer 2023
# Package List
This repository consists of several ROS packages
- turtlebot_control - publishes video stream from the Raspberry Pi camera and receives controls for the turtlebot actions
- img_transform - subscribes to the video stream to feature match and find the SE(2) transformation between two frames

The goal of this project is to implement a novel approach to Ergodic Active SLAM using only visual sensor measurements to map and localize a turtlebot to recreate a map of a poster that it's driving over.
