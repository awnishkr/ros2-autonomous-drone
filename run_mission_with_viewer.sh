#!/usr/bin/env bash
set -e
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Camera opens immediately and waits for the incoming image topic.
bash "$BASE/launch_mission.sh" &
MISSION_PID=$!
cleanup() { kill "$MISSION_PID" 2>/dev/null || true; wait "$MISSION_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 10
bash "$BASE/view_landing_camera.sh"
