source /opt/ros/jazzy/setup.bash
# rosdep install -i --from-path src --rosdistro jazzy -y
# colcon build --packages-select custom_interfaces
# colcon build --packages-select mock_robot
python3 unzip.py "sw_robot_install.zip"
source sw_robot_init/local_setup.bash
ros2 run sw_robot data_sink