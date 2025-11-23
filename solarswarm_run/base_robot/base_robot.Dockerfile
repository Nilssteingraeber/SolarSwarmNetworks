FROM ros:jazzy
COPY ../solarswarm/ /home/ubuntu/solarswarm/
COPY base_robot_init.bash /home/ubuntu/solarswarm
# COPY unzip.py /home/ubuntu/solarswarm
WORKDIR /home/ubuntu/solarswarm
SHELL ["/bin/bash", "-c"]

RUN apt-get update && rosdep update && rosdep install -i --from-path src --rosdistro jazzy -y
RUN source /opt/ros/jazzy/setup.bash && colcon build

ENTRYPOINT ["bash", "base_robot_init.bash"]
