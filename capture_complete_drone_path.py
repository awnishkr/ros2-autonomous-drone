#!/usr/bin/env python3
"""Record the real ROS 2 drone odometry path and create a report-ready PNG.

Start this after Gazebo/bridge launch but BEFORE starting the drone mission.
Stop with Ctrl+C after the drone has landed. Requires rclpy, nav_msgs and matplotlib.
"""
from __future__ import annotations

import csv
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

HOME = Path.home()
REPORT_DIR = HOME / 'drone_ws' / 'report_screenshots'
CSV_PATH = REPORT_DIR / '23_complete_drone_flight_path.csv'
PNG_PATH = REPORT_DIR / '23_complete_drone_flight_path.png'
WORLD_PATH = HOME / 'drone_ws/src/drone_description/worlds/ai_demo_offset.sdf'
TOPIC = '/model/quadcopter/odometry'


def world_obstacles():
    """Read real rectangle footprints from the world file, never inferred from pixels."""
    if not WORLD_PATH.is_file():
        return []
    try:
        root = ET.parse(WORLD_PATH).getroot()
        boxes = []
        for m in root.findall('.//model'):
            name = m.get('name', '')
            if not (name.startswith('wall_') or name == 'unexpected_wall'):
                continue
            pose = m.findtext('pose')
            dims = m.findtext('./link/collision/geometry/box/size')
            if not pose or not dims:
                continue
            x, y = (float(v) for v in pose.split()[:2])
            sx, sy = (float(v) for v in dims.split()[:2])
            boxes.append((name, x, y, sx, sy))
        return boxes
    except (ET.ParseError, ValueError, OSError) as exc:
        print(f'Could not parse obstacles from {WORLD_PATH}: {exc}')
        return []


class PathLogger(Node):
    def __init__(self):
        super().__init__('flight_path_report_recorder')
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        self.file = CSV_PATH.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_s', 'x_m', 'y_m', 'z_m'])
        self.samples = []
        self.origin = None
        self.last_sample = None
        self.last_report = time.monotonic()
        self.create_subscription(Odometry, TOPIC, self.on_odom, qos_profile_sensor_data)
        print(f'Recording drone odometry from: {TOPIC}', flush=True)
        print('Keep this terminal open for the entire mission. After landing, press Ctrl+C.', flush=True)

    def on_odom(self, msg):
        now = time.monotonic()
        if self.origin is None:
            self.origin = now
        pos = msg.pose.pose.position
        xyz = (float(pos.x), float(pos.y), float(pos.z))
        if self.last_sample is not None:
            last_t, last_x, last_y, last_z = self.last_sample
            dt = now - last_t
            drift = math.dist(xyz, (last_x, last_y, last_z))
            if dt < 0.10 and drift < 0.025:
                return
        record = (round(now - self.origin, 3), *xyz)
        self.samples.append(record)
        self.writer.writerow([f'{record[0]:.3f}', *(f'{v:.6f}' for v in xyz)])
        self.last_sample = (now, *xyz)
        if len(self.samples) % 15 == 0:
            self.file.flush()
        if now - self.last_report > 7:
            self.last_report = now
            print(f'Samples: {len(self.samples)} | XY=({xyz[0]:.2f}, {xyz[1]:.2f}) | z={xyz[2]:.2f}m', flush=True)

    def save(self):
        self.file.flush()
        self.file.close()
        if len(self.samples) < 2:
            print('Not enough odometry received. Check Gazebo, ROS 2 bridge and topic.', flush=True)
            return
        ts = [row[0] for row in self.samples]
        xs = [row[1] for row in self.samples]
        ys = [row[2] for row in self.samples]
        zs = [row[3] for row in self.samples]
        total_xy = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1])
                       for i in range(1, len(xs)))

        fig = plt.figure(figsize=(13.5, 7.2), facecolor='white')
        grid = fig.add_gridspec(1, 2, width_ratios=(1.3, 1), wspace=0.24,
                               left=0.075, right=0.97, top=0.83, bottom=0.16)
        ax = fig.add_subplot(grid[0, 0])
        alt = fig.add_subplot(grid[0, 1])
        navy = '#174a78'
        for name, x, y, sx, sy in world_obstacles():
            rect = Rectangle((x-sx/2, y-sy/2), sx, sy,
                             facecolor='#cbd5df', edgecolor='#566779', linewidth=1.3, zorder=1)
            ax.add_patch(rect)
            ax.text(x, y, name.replace('unexpected_wall', 'unexpected\nwall'),
                    ha='center', va='center', fontsize=7.2, color='#253246', zorder=2)
        ax.plot(xs, ys, color=navy, linewidth=2.4, zorder=3, label='Recorded drone path')
        ax.scatter([xs[0]], [ys[0]], s=83, c='#268652', edgecolor='white', zorder=5)
        ax.scatter([5], [5], s=115, c='#efa825', marker='*', edgecolor='#735410', linewidth=.5, zorder=5)
        ax.scatter([6], [6], s=100, c='#7c4ba6', marker='s', edgecolor='white', zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], s=85, c='#cf5146', marker='X', edgecolor='white', zorder=6)
        ax.annotate('Start', (xs[0], ys[0]), xytext=(8, -15), textcoords='offset points', fontsize=9)
        ax.annotate('A* goal (5, 5)', (5, 5), xytext=(-90, 16), textcoords='offset points', fontsize=9)
        ax.annotate('Helipad (6, 6)', (6, 6), xytext=(10, 9), textcoords='offset points', fontsize=9)
        ax.set_title('Recorded XY trajectory', fontsize=13, color='#122d45', weight='bold', pad=11)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.25)
        for spine in ax.spines.values(): spine.set_color('#bdc8d1')

        alt.plot(ts, zs, color=navy, linewidth=2.2)
        alt.scatter([ts[0]], [zs[0]], s=55, c='#268652', zorder=4)
        alt.scatter([ts[-1]], [zs[-1]], s=55, c='#cf5146', zorder=4)
        alt.set_title('Altitude during mission', fontsize=13, color='#122d45', weight='bold', pad=11)
        alt.set_xlabel('Time since recording started (s)')
        alt.set_ylabel('Z (m)')
        alt.grid(alpha=.25)
        alt.set_ylim(bottom=min(0, min(zs)-0.1))
        for spine in alt.spines.values(): spine.set_color('#bdc8d1')

        fig.suptitle('Autonomous drone — complete recorded flight path', x=0.075,
                     y=0.955, ha='left', fontsize=18, weight='bold', color='#142e49')
        fig.text(.075, .905, 'ROS 2 /model/quadcopter/odometry • Gazebo simulation',
                 fontsize=10, color='#526275')
        landed = (zs[-1] < 0.25 and math.hypot(xs[-1]-6, ys[-1]-6) < 0.5)
        foot = f'{len(xs)} odometry samples  •  Recorded XY travel: {total_xy:.2f} m  •  Final XYZ: ({xs[-1]:.3f}, {ys[-1]:.3f}, {zs[-1]:.3f}) m'
        fig.text(.075, .08, foot, fontsize=9.4, color='#334a61')
        if landed:
            error = 100*math.hypot(xs[-1]-6, ys[-1]-6)
            fig.text(.075, .052, f'Helipad-centre horizontal error: {error:.2f} cm (based on final odometry sample)',
                     fontsize=9.2, color='#236b50')
        else:
            fig.text(.075, .052, 'Final sample is not verified as landed; stop the recorder only after touchdown.',
                     fontsize=9.2, color='#9a5631')
        fig.savefig(PNG_PATH, dpi=180, facecolor='white')
        plt.close(fig)
        print(f'Flight path image: {PNG_PATH}', flush=True)
        print(f'Raw trajectory CSV: {CSV_PATH}', flush=True)


def main():
    rclpy.init()
    node = PathLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
