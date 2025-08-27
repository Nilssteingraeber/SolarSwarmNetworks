FROM ros:jazzy
COPY sw_robot_init /home/ubuntu/solarswarm/
COPY sw_robot_install.zip /home/ubuntu/solarswarm/
COPY unzip.py /home/ubuntu/solarswarm/
WORKDIR /home/ubuntu/solarswarm
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["bash", "mock_data_init.bash"]