ros2_turtlebot_gazebo_test
===========================

Expermantal packages for turtlebot on ROS2 & Gazebo

# example

![](https://raw.githubusercontent.com/wiki/otamachan/ros2_turtlebot_gazebo_test/demo.gif)

# How to

OS: Ubuntu 18.04

1. Install ROS2 Crystal

Install ``ros-crystal-desktop``.
See https://index.ros.org/doc/ros2/Linux-Install-Debians/#install-ros-2-packages .

2. Build

```bash
sudo apt install python3-vsctool
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/otamachan/ros2_turtlebot_gazebo_test/master/test.repos
vcs import src < test.repos
source /opt/ros/crystal/setup.bash
colcon build --symlink-install
```

3. Run

```bash
cd ~/ros2_ws
source /opt/ros/crystal/setup.bash
source install/setup.bash
ros2 launch ros2 launch ros2_turtlebot_gazebo_test empty_world.launch.py
```
