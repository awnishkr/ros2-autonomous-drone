#!/usr/bin/env bash
# Run the complete mission from this extracted project (no install into ~/drone_ws).
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is not installed at /opt/ros/jazzy.' >&2
  exit 1
fi
# ROS and venv initialization may reference unset variables; disable nounset temporarily.
set +u
source /opt/ros/jazzy/setup.bash

# Reuse your existing camera dependencies, but never install into the old workspace.
if [[ -f "$HOME/drone_landing_venv/bin/activate" ]]; then
  source "$HOME/drone_landing_venv/bin/activate"
fi
set -u

for program in ros2 gz python; do
  if ! command -v "$program" >/dev/null 2>&1; then
    echo "Required command not available: $program" >&2
    exit 1
  fi
done
for package in ros_gz_bridge ros_gz_image ros_gz_interfaces; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Required ROS package not available: $package" >&2
    exit 1
  fi
done

python - <<'PY'
import sys
for module in ('rclpy', 'cv2', 'numpy', 'onnxruntime'):
    try:
        __import__(module)
    except Exception as exc:
        print(f'Python dependency {module} unavailable: {exc}', file=sys.stderr)
        raise SystemExit(1)
PY

export DRONE_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src/drone_controller:${PYTHONPATH:-}"
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/src/drone_description/worlds:$PROJECT_ROOT/src/drone_description/models:${GZ_SIM_RESOURCE_PATH:-}"

WORLD="$PROJECT_ROOT/src/drone_description/worlds/mission_vision_landing.sdf"
MODEL="$PROJECT_ROOT/vision_landing_assets/landing_detector.onnx"
MAP="$PROJECT_ROOT/maps/lidar_obstacle_regions.json"
for file in "$WORLD" "$MODEL" "$MAP" "$PROJECT_ROOT/src/drone_description/worlds/landing_pad.png"; do
  if [[ ! -f "$file" ]]; then
    echo "Project file not found: $file" >&2
    exit 1
  fi
done

LOG_DIR="$PROJECT_ROOT/mission_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

launch() {
  local name="$1"
  shift
  "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  echo "Started $name (PID $!)"
}

echo "PROJECT: $PROJECT_ROOT"
echo "WORLD: $WORLD"
echo "MODEL: $MODEL"
echo "MAP: $MAP"
echo 'Close any previously running drone mission before launching this one.'
echo 'Do not press Play in Gazebo until all controllers have started.'

launch gazebo gz sim -v 3 "$WORLD"
sleep 5
launch bridge ros2 run ros_gz_bridge parameter_bridge \
  '/quadcopter/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/quadcopter/enable@std_msgs/msg/Bool]gz.msgs.Boolean' \
  '/model/quadcopter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/quadcopter/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/world/first_flight/model/quadcopter/link/base_link/sensor/body_collision_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' \
  '/world/first_flight/model/quadcopter/link/base_link/sensor/arm_1_collision_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' \
  '/world/first_flight/model/quadcopter/link/base_link/sensor/arm_2_collision_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'

launch camera_bridge ros2 run ros_gz_image image_bridge \
  /quadcopter/landing_camera/image --ros-args -p qos:=sensor_data
sleep 3
launch detector python "$PROJECT_ROOT/vision_landing_assets/live_landing_detector.py"
launch router python "$PROJECT_ROOT/src/drone_controller/drone_controller/mission_router.py"
launch landing python "$PROJECT_ROOT/src/drone_controller/drone_controller/mission_landing.py" \
  --ros-args \
  -p flight_enabled:=true \
  -p horizontal_enabled:=true \
  -p descent_enabled:=true \
  -p shutdown_on_contact:=true \
  -r /quadcopter/cmd_vel:=/mission/landing/cmd_vel \
  -r /quadcopter/enable:=/mission/landing/enable
launch navigation python "$PROJECT_ROOT/src/drone_controller/drone_controller/navigation_handoff.py" \
  --ros-args \
  -p dry_run:=false \
  -r /quadcopter/cmd_vel:=/mission/nav/cmd_vel \
  -r /quadcopter/enable:=/mission/nav/enable

echo
printf '%s\n' 'Mission processes launched. Now press Play in Gazebo.'
echo "Logs: $LOG_DIR"
echo 'To open the camera, run: bash view_landing_camera.sh in another terminal.'
echo 'Ctrl+C here stops this mission.'

# Surface key errors while maintaining the lifetime of Gazebo and bridges.
tail -n 0 -F "$LOG_DIR/navigation.log" "$LOG_DIR/landing.log" "$LOG_DIR/router.log" &
PIDS+=("$!")
wait "${PIDS[0]}"
