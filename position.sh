#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash

echo "Final Drone Position:"
ros2 topic echo /model/quadcopter/odometry \
  --once --field pose.pose.position \
  --qos-reliability best_effort
