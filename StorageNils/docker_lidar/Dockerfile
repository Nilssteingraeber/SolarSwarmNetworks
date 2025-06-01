FROM ros:jazzy

# install ros package

# Stupid old key.. Amazingly, we don't have curl so I added 
# the ros.key by hand.
COPY ros.key /usr/share/keyrings/ros-archive-keyring.gpg
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install

CMD ["/bin/bash"]
