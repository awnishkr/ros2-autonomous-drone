#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS and venv initialization may reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/drone_landing_venv/bin/activate" ]]; then
  source "$HOME/drone_landing_venv/bin/activate"
fi
set -u
export DRONE_PROJECT_ROOT="$PROJECT_ROOT"
exec python "$PROJECT_ROOT/vision_landing_assets/landing_camera_viewer.py"
