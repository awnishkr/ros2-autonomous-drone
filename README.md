# ROS 2 Autonomous Drone — Navigation and Vision-Guided Landing

A ROS 2 Jazzy / Gazebo Harmonic simulation demonstrating A* navigation, LiDAR obstacle response, mission handover, and YOLO11n-based landing-pad detection. The vision model is supplied as an ONNX file for **inference**; this project does not claim to have trained a new neural network.

This version was rebuilt from the supplied `drone_ws_1.zip`. It excludes old compiled output, Git internals, temporary backups, and machine-specific paths. The active world and controller paths have professional descriptive names. ROS 2 topic names for the detector, camera overlay, landing controller and mission router are aligned.

## Main demonstration

- **Navigation goal:** (5.0, 5.0) m; nominal flight altitude 2.0 m.
- **Landing-pad center:** (6.0, 6.0) m in `mission_vision_landing.sdf`.
- **Navigation:** A* route planning and waypoint following, LiDAR safety stops, recovery and replanning.
- **Landing:** explicit controller handover, live YOLO11n/ONNX image inference, image-centering alignment and controlled touchdown.
- The number of A* waypoints can change when the drone replans around sensed obstacles.

## Requirements

Ubuntu system with `/opt/ros/jazzy/setup.bash`, `gz sim`, `ros_gz_bridge`, `ros_gz_image`, `ros_gz_interfaces`, and Python packages `rclpy`, `numpy`, `cv2`, and `onnxruntime`. This package does not install ROS 2 or Gazebo.

The launcher optionally activates your existing `~/drone_landing_venv` so the camera dependencies are available. A venv must also be able to import ROS 2's `rclpy` module; use `--system-site-packages` when constructing a new venv. If no compatible venv is present, install the ONNX/OpenCV Python packages into a suitable ROS-compatible environment.

## Run directly from the extracted directory

Extract the ZIP to Downloads, then:

```bash
cd ~/Downloads/ROS2_Autonomous_Drone_Verified/ros2-autonomous-drone
python3 tests/validate_project.py
bash launch_mission.sh
```

Wait until the terminal prints **Mission processes launched**, then press **Play** in Gazebo. Do not start a second copy of the mission while the first is running. Running `gz sim` alone does **not** start the flight controller and the drone will fall under gravity.

Open the live camera feed in a second terminal:

```bash
cd ~/Downloads/ROS2_Autonomous_Drone_Verified/ros2-autonomous-drone
bash view_landing_camera.sh
```

You can also run `bash run_mission_with_viewer.sh` to start the mission and camera from one terminal, but the two-terminal procedure is easier to debug.

Position check, in a third terminal: `bash position.sh`.

### Controls and troubleshooting

- `Ctrl+C` in the mission terminal stops its processes; press `q` in the camera viewer to close its window.
- `gz` and ROS 2 must be installed and on your PATH. To inspect errors, read `mission_logs/<timestamp>/{gazebo,bridge,navigation,landing,detector}.log`.
- If the drone falls, verify that the bridge and flight controller started before pressing Play.
- If the controller prints **SENSOR LOSS**, inspect the printed odometry and LiDAR ages; the safety HOLD stays enabled.
- If the camera says **WAITING FOR CAMERA**, confirm `/quadcopter/landing_camera/image` is being published. Detection output uses `/landing/detection`.

## Key files

| File or folder | Function |
|---|---|
| `launch_mission.sh` | Start the complete mission from this folder |
| `view_landing_camera.sh` | Display camera and detection overlay |
| `src/drone_description/worlds/mission_vision_landing.sdf` | Gazebo world with landing pad |
| `src/drone_controller/drone_controller/navigation_handoff.py` | Navigation and handover controller |
| `src/drone_controller/drone_controller/mission_router.py` | Command arbitration |
| `src/drone_controller/drone_controller/mission_landing.py` | Vision-guided landing stage |
| `src/drone_controller/drone_controller/dynamic_navigation.py` | A* planner |
| `vision_landing_assets/live_landing_detector.py` | YOLO11n ONNX inference |
| `vision_landing_assets/landing_detector.onnx` | Unmodified ONNX weights from uploaded ZIP |
| `maps/lidar_obstacle_regions.json` | Map-based obstacle regions |
| `tests/validate_project.py` | Static integrity and A* smoke test |

See `docs/PROJECT_VERIFICATION.md` for the validation scope and important limitations.

## Technical limitations

This is a **simulation prototype**, not a safety-certified physical drone system. YOLO confidence is not accuracy. Some components use known simulation geometry and odometry, so precision landing is not a purely vision-only method. Local static checks cannot prove that a complete Gazebo flight will run correctly on your machine.

## License

The supplied ROS 2 packages declare Apache-2.0; see `LICENSE` and the package license files.
