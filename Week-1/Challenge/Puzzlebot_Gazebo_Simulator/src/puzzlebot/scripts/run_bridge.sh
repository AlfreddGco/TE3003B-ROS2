#!/bin/bash
source /opt/ros/noetic/setup.sh
source /opt/ros/foxy/setup.sh
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
