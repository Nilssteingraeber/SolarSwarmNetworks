FROM base_robot
COPY sw_robot_init.bash /home/ubuntu/solarswarm

ENTRYPOINT ["bash", "sw_robot_init.bash"]
