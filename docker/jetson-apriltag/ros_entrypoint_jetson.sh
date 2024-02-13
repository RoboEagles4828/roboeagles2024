#!/bin/bash
source /opt/ros/${ROS_DISTRO}/install/setup.bash
source ./src/install/setup.bash

ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py