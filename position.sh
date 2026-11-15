#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
ros2 topic echo /model/quadcopter/odometry --once --field pose.pose.position --qos-reliability best_effort
