import subprocess
import time

setup = 'source /opt/ros/jazzy/setup.bash && source /lidar_node_ws/install/setup.bash &&'

rviz_cmd = f'{setup} ros2 run rviz2 rviz2'
lidar_cmd = f'{setup} ros2 launch ldlidar_node ldlidar_bringup.launch.py '
lifecycle_cmd = f'{setup} ros2 lifecycle set /ldlidar_node configure && \
ros2 lifecycle set /ldlidar_node activate && ros2 topic list'

#res = subprocess.run(["lsusb"], capture_output=True, text=True, check=True)
#print(res.stdout, "\n\n")


lidar_proc = subprocess.Popen(lidar_cmd, shell=True, executable='/bin/bash')
time.sleep(2) # Let's wait a bit
lifecycle_proc = subprocess.Popen(lifecycle_cmd, shell=True, executable='/bin/bash')
time.sleep(2) # Let's wait a bit
rviz_proc = subprocess.Popen(rviz_cmd, shell=True, executable='/bin/bash')

rviz_proc.wait()
lidar_proc.wait()
lidar_proc.wait()