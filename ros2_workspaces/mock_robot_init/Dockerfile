FROM ros:jazzy
ADD ros_packages /home/ubuntu/solarswarm
WORKDIR /home/ubuntu/solarswarm
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["bash", "mock_data_init.bash"]