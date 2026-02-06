FROM ros:jazzy
COPY solarswarm/ /home/ubuntu/solarswarm/
COPY base_robot/mock_robot_init.bash /home/ubuntu/solarswarm
# COPY unzip.py /home/ubuntu/solarswarm
WORKDIR /home/ubuntu/solarswarm
SHELL ["/bin/bash", "-c"]

RUN mkdir /var/tmp/solarswarm
COPY tmp/env /var/tmp/solarswarm/

RUN apt-get update && rosdep update && rosdep install -i --from-path src --rosdistro jazzy -y
RUN source /opt/ros/jazzy/setup.bash && colcon build

ENTRYPOINT ["bash", "mock_robot_init.bash"]
# For other services: Use base_robot as image and replace entrypoint with a different script
