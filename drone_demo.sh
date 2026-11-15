#!/usr/bin/env bash
set -eo pipefail

WS="$HOME/drone_ws"
WORLD="$WS/src/drone_description/worlds/dynamic_obstacle_test.sdf"
LOG_DIR="$WS/demo_logs/$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"

GZ_PID=""
BRIDGE_PID=""
CTRL_PID=""

cleanup() {
    trap - EXIT INT TERM

    for pid in "$CTRL_PID" "$BRIDGE_PID" "$GZ_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
}

trap cleanup EXIT
trap 'exit 130' INT TERM

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

[[ -f "$WORLD" ]] || {
    echo "ERROR: Gazebo world missing."
    exit 1
}

echo
echo "======================================="
echo "     ROS 2 AUTONOMOUS DRONE DEMO"
echo "======================================="
echo

echo "[1/4] Starting Gazebo..."

gz sim -r -v 3 "$WORLD" \
    >"$LOG_DIR/gazebo.log" 2>&1 &

GZ_PID=$!

sleep 2

if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "ERROR: Gazebo failed to start."
    exit 1
fi

echo "[2/4] Starting ROS 2 bridge..."

ros2 run ros_gz_bridge parameter_bridge \
'/quadcopter/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
'/quadcopter/enable@std_msgs/msg/Bool]gz.msgs.Boolean' \
'/model/quadcopter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
'/quadcopter/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
>"$LOG_DIR/bridge.log" 2>&1 &

BRIDGE_PID=$!

echo "[3/4] Performing safety checks..."

python3 <<'PY'
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rclpy.init()
node = Node("professor_demo_preflight")

data = {
    "position": None,
    "odom": 0.0,
    "scan": 0.0
}

def on_odom(msg):
    p = msg.pose.pose.position
    data["position"] = (p.x, p.y, p.z)
    data["odom"] = time.monotonic()

def on_scan(msg):
    if msg.ranges:
        data["scan"] = time.monotonic()

subscriptions = [
    node.create_subscription(
        Odometry,
        "/model/quadcopter/odometry",
        on_odom,
        qos_profile_sensor_data
    ),
    node.create_subscription(
        LaserScan,
        "/quadcopter/scan",
        on_scan,
        qos_profile_sensor_data
    )
]

try:
    deadline = time.monotonic() + 30

    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

        now = time.monotonic()

        if (
            data["position"] is not None
            and now - data["odom"] < 1
            and now - data["scan"] < 1
        ):
            break

    now = time.monotonic()
    p = data["position"]

    sensors = (
        p is not None
        and now - data["odom"] < 1
        and now - data["scan"] < 1
    )

    grounded = (
        p is not None
        and math.hypot(p[0], p[1]) < 0.3
        and 0 <= p[2] < 0.2
    )

    publishers = sum(
        len(node.get_publishers_info_by_topic(topic))
        for topic in (
            "/quadcopter/cmd_vel",
            "/quadcopter/enable"
        )
    )

    print("Sensors:", "PASS" if sensors else "FAIL")
    print("Starting position:", p)
    print("Grounded:", "PASS" if grounded else "FAIL")
    print("Other controllers:", publishers)

    if not (sensors and grounded and publishers == 0):
        raise SystemExit("PREFLIGHT FAILED — Flight blocked")

    print("PREFLIGHT PASSED")

finally:
    node.destroy_node()
    rclpy.shutdown()
PY

echo
echo "======================================="
echo "      READY FOR AUTONOMOUS MISSION"
echo "======================================="
echo

read -r -p \
"Press ENTER to START flight (type q to cancel): " RESPONSE

if [[ -n "$RESPONSE" ]]; then
    echo "Flight cancelled."
    exit 0
fi

echo "[4/4] Starting autonomous drone..."

ros2 run drone_controller dynamic_autonomy \
    --ros-args \
    -p dry_run:=false \
    -p takeoff_only:=false \
    -p waypoint_test:=false \
    -p recovery_test:=false \
    >"$LOG_DIR/mission.log" 2>&1 &

CTRL_PID=$!

echo
echo "MISSION RUNNING — Watch Gazebo."
echo "Pause Gazebo immediately if the drone behaves unexpectedly."
echo

seen=0
completed=0

while kill -0 "$CTRL_PID" 2>/dev/null; do

    total=$(wc -l <"$LOG_DIR/mission.log")

    if (( total > seen )); then
        sed -n "$((seen+1)),${total}p" \
            "$LOG_DIR/mission.log" |
            grep -E \
            'Takeoff complete|Waypoint [0-9]+ reached|SAFETY STOP|Recovery proposed|NEW A\* ROUTE|Destination reached|Hover complete|Touchdown confirmed|MISSION COMPLETE|SENSOR LOSS|Recovery stopped|No verified recovery|ERROR|HOLD' |
            sed -E 's/^.*\[dynamic_autonomy\]: //' || true

        seen=$total
    fi

    if grep -q \
        'MISSION COMPLETE: Landed and disarmed' \
        "$LOG_DIR/mission.log"; then

        completed=1
        break
    fi

    sleep 0.5
done

if (( completed )); then
    echo
    echo "======================================="
    echo "        MISSION SUCCESSFUL"
    echo "       DRONE LANDED SAFELY"
    echo "======================================="
else
    echo
    echo "Mission did not complete normally."
    echo "Check: $LOG_DIR/mission.log"
fi

echo
read -r -p "Press ENTER to close the demo: " _ || true
