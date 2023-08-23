# Img_Transform Description
A package to find the SE(2) transformation between two frames from a subsribed video stream from the turtlebot.

Launch file:
`ros2 launch img_transform active_slam.launch.xml`

Service calls:

    `stop_moving`:
        used to stop any motion of the turtlebot 
        topic: /stop_moving (std_srvs/srv/Empty)
        example:

        `ros2 service call /stop_moving std_srvs/srv/Empty`