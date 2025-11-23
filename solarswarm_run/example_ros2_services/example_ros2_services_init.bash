#!/bin/bash
# source /opt/ros/jazzy/setup.bash;
# python3 unzip.py "install.zip";
source install/setup.bash;
ros2 run mock_robot $EXAMPLE_ROS2_SERVICE;
