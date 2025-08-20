source /opt/ros/jazzy/setup.bash
# rosdep install -i --from-path src --rosdistro jazzy -y
# colcon build --packages-select custom_interfaces
# colcon build --packages-select mock_robot
source install/local_setup.bash
ros2 run mock_robot mock_data