import subprocess
import time

setup = 'source /opt/ros/jazzy/setup.bash && source /lidar_node_ws/install/setup.bash &&'
rviz_cmd = f'{setup} ros2 run rviz2 rviz2'
rviz_proc = subprocess.Popen(rviz_cmd, shell=True, executable='/bin/bash')
rviz_proc.wait()
