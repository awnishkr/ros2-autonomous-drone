#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
source ~/drone_landing_venv/bin/activate

echo "YOLO11n Precision Landing Camera"

exec python ~/drone_landing_assets/ai_camera_view.py
