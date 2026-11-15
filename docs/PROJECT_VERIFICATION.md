# Project verification and source notes

## Source

The source of this release is the user-provided `drone_ws_1.zip` (2026-11-15). Only active ROS 2 source files, simulation files, sample evidence, maps and camera assets were copied. Generated build/install directories, `.git`, compiled files, old backups and mission logs were excluded.

## Consistency changes

- Renamed `ai_landing_assets/` to `vision_landing_assets/` and `best.onnx` to `landing_detector.onnx`; model bytes remain identical.
- Renamed the viewer to `landing_camera_viewer.py` and the preview controller to `landing_alignment_preview.py`.
- Aligned the detector and viewer with `/landing/detection` and handover with `/mission/handoff`.
- Made active mission launchers resolve paths relative to the extracted directory, not `~/drone_ws` or `~/drone_landing_assets`.
- Preserved the main Gazebo SDF world and A* map, with one worker-thread fix for long recovery A* calculations so callbacks are not blocked. The sensor-loss HOLD remains enabled.

## Not proven by offline validation

A full ROS 2 / Gazebo run was not available in the build environment. You must test takeoff, live LiDAR obstacle response, navigation handover, camera image bridge and landing in your own Ubuntu system. No flight accuracy or detection accuracy measurement is claimed.
