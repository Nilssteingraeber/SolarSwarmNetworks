FROM base_robot
COPY data_sink/sw_robot_init.bash /home/ubuntu/solarswarm

ENTRYPOINT ["bash", "sw_robot_init.bash"]
