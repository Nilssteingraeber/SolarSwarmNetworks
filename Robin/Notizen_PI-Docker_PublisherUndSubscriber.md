# Running a ROS2 Jazzy Subscriber in Docker

## Step 1: Prepare Dockerfile

- Start from the official `ros:jazzy` image.
- Install dependencies (`colcon`, `git`, `ros-jazzy-example-interfaces`).
- Clone the ROS2 `examples` repo (jazzy branch) with subscriber code.
- Build the workspace with `colcon build --symlink-install`.
- Set the default command to run an **existing** subscriber executable (e.g., `subscriber_member_function`).
- [Intruction to Docker-Inage missing]
- Test:
```bash
sudo docker pull robincoding97/ros2-jazzy-subscriber:latest
```
Doesn't work on arm64 (Raspberry PI 4) -> Do following steps on Machine that created Image:
```bash
docker buildx create --use
```
```bash
docker buildx build --platform linux/arm64/v8 -t username/ros2-jazzy-subscriber:arm64 --push .
```

```bash
sudo docker pull robincoding97/ros2-jazzy-subsriber:arm64
```

## Step 2: Build the Docker Image

```bash
docker build -t ros2-jazzy-subscriber .
```

## Step3: Run the Docker Container Interactively for Debugging
```bash
docker run -it --rm ros2-jazzy-subscriber /bin/bash
```
## Step 4:  Inside the Container, Source ROS2 and Workspace
```bash
source /opt/ros/jazzy/setup.sh
source /ros2_ws/install/setup.bash
```

## Step 5:  Verify Subscriber Executables
```bash
ls /ros2_ws/install/examples_rclcpp_minimal_subscriber/lib/examples_rclcpp_minimal_subscriber/
```
## Step 6: Run a Valid Subscriber Executable
```bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

## Step 7: (Optional): Test Messaging with Publisher
```bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

## Step 8: Run the Container Normally
```bash
docker run --rm ros2-jazzy-subscriber
```

## Extra Information: How to run Publisher and Subscriber
## Raspberry PI 4 (Subscriber) ThinkPad (Publisher)

## Publisher

```bash
docker run --rm -it --network host robincoding97/ros2-jazzy-publisher /bin/bash
```
## In the Dockerterminal

```bash
source /opt/ros/jazzy/setup.bash
```
```bash
source /ros2_ws/install/setup.bash
```

```bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```

## Subscriber
```bash
docker run --rm -it --network host robincoding97/ros2-jazzy-publisher:arm64 /bin/bash
```
## In the Dockerterminal

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
source /ros2_ws/install/setup.bash
```

```bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```
## Problem
PI hat als Subscriber nichts empfangen. Lösung?
## Troubleshooting, funktioniert!
Eine XML Datei erstellen:
```bash
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.42.0.3</address> <!-- bat0 IP vom Pi -->
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
</profiles>
```
Und dann mit

```bash
docker run -it --rm --net=host \
  -v /home/pi/Documents/Softwarepraktikum/fastdds.xml:/root/fastdds.xml \
  robincoding97/ros2-jazzy-subscriber:arm64 /bin/bash
```
starten.

Im Container selber:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

