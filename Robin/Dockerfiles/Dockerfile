FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions git \
    ros-jazzy-example-interfaces \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
RUN git clone --depth=1 https://github.com/Nilssteingraeber/SolarSwarmNetworks.git /tmp/SolarSwarmNetworks \
    && mv /tmp/SolarSwarmNetworks/Robin/ros2_ws /ros2_ws \
    && rm -rf /tmp/SolarSwarmNetworks

WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

CMD ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && py_pubsub turtle_publisher"]
