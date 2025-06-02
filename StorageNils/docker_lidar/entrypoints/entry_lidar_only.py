import subprocess
import time

src = 'source /opt/ros/jazzy/setup.bash && source /lidar_node_ws/install/setup.bash &&'
lidar_cmd = f'{src} ros2 launch ldlidar_node ldlidar_bringup.launch.py '
lifecycle_cmd = f'{src} ros2 lifecycle set /ldlidar_node configure && \
ros2 lifecycle set /ldlidar_node activate && ros2 topic list'

lidar_proc = subprocess.Popen(lidar_cmd, shell=True, executable='/bin/bash')
# Maybe we should wait a bit before setting the lifecycles manually.. But idk
time.sleep(5)
lifecycle_proc = subprocess.Popen(lifecycle_cmd, shell=True, executable='/bin/bash')

print("Now running LIDAR. (Hopefully)")

lidar_proc.wait()
lifecycle_proc.wait()


