FROM base_robot
COPY example_ros2_services/example_ros2_services_init.bash /home/ubuntu/solarswarm

ENTRYPOINT ["bash", "example_ros2_services_init.bash"]
