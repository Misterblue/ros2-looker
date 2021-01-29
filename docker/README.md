To keep compile time small, the project has been broken into
multiple Docker images.

ros2-base: Ubuntu 20.04 with ROS2 foxy installed

ros2-looker-base: ros2-base with libraries needed by the Python
    ROS2 applications. This exists mainly because dlib (face
    recognition library) installation takes a Very Long Time.

ros2-looker: ros2-looker-base plus the ros2-looker package nodes

