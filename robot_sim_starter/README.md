# Four-wheel ROS 2 Jazzy + Gazebo Harmonic starter

Prototype dimensions are illustrative, not chassis measurements. Wheels are skid-steer (two joints per side), LiDAR is a GPU 360-degree 10 Hz scanner, and a box is ahead of the robot.

## Install (Ubuntu 24.04 with ROS Jazzy apt configured)
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard
source /opt/ros/jazzy/setup.bash
```

## Run: terminal 1
```bash
gz sim -r -v 4 worlds/four_wheel_lidar.sdf
```

## Run: terminal 2
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/config/bridge.yaml -p use_sim_time:=true
```

## Inspect: terminal 3
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## Drive: terminal 4
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

The simulator is deliberately kept as an SDF-world-first milestone. URDF/Xacro, robot_state_publisher, TF bridge and ros2_control may be added in subsequent milestones. The /odom message does not by itself provide a ROS /tf tree. Nav2 is not yet configured. Teleop does not enforce collision avoidance.
