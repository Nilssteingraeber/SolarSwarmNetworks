FROM ros:jazzy
COPY mock_robot/ /home/ubuntu/solarswarm/src/mock_robot/
COPY custom_interfaces/ /home/ubuntu/solarswarm/src/custom_interfaces/
WORKDIR /home/ubuntu/solarswarm
SHELL ["/bin/bash", "-c"]

RUN apt-get update && rosdep update && rosdep install -i --from-path src --rosdistro jazzy -y
RUN source /opt/ros/jazzy/setup.bash && colcon build
RUN source install/local_setup.bash

# Usage:
# 1. Build with internet using
#   sudo docker build --tag mock_robot_test -f mock_robot_test.dockerfile .
# 2. Run using
#   sudo docker run -it mock_robot_test
# 3. Test using
#   source install/local_setup.bash
#   colcon test --packages-select mock_robot --event-handlers console_direct+