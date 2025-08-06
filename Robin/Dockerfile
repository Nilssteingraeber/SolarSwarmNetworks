FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions git \
    ros-jazzy-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
RUN git clone -b jazzy https://github.com/ros2/examples.git

WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run examples_rclcpp_minimal_subscriber minimal_subscriber"]
