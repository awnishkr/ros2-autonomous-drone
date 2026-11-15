#!/usr/bin/env python3
"""Offline consistency checks for the renamed ROS 2 / Gazebo project."""
import ast
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = ROOT / 'src/drone_description/worlds'
CTRL_DIR = ROOT / 'src/drone_controller/drone_controller'
VISION_DIR = ROOT / 'vision_landing_assets'


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def run():
    checked = {'python': 0, 'shell': 0, 'sdf': 0, 'xml': 0, 'json': 0, 'imports': 0}
    py_files = list(ROOT.rglob('*.py'))
    for path in py_files:
        compile(path.read_text(encoding='utf-8'), str(path), 'exec')
        checked['python'] += 1

    for path in ROOT.glob('*.sh'):
        subprocess.run(['bash', '-n', str(path)], check=True)
        checked['shell'] += 1

    for path in ROOT.rglob('*.sdf'):
        xml = ET.parse(path)
        checked['sdf'] += 1
        for map_node in xml.findall('.//albedo_map'):
            resource = (map_node.text or '').strip()
            if resource == 'landing_pad.png':
                require((path.parent / resource).is_file(), f'{path}: landing-pad texture is missing')
            require('home/awnish-kumar' not in resource, f'{path}: fixed machine path still used')

    for path in [*ROOT.rglob('package.xml'), *ROOT.rglob('model.config')]:
        ET.parse(path)
        checked['xml'] += 1

    for path in ROOT.rglob('*.json'):
        json.loads(path.read_text(encoding='utf-8'))
        checked['json'] += 1

    local_modules = {p.stem for p in CTRL_DIR.glob('*.py')} | {p.stem for p in VISION_DIR.glob('*.py')}
    for path in [*CTRL_DIR.glob('*.py'), *VISION_DIR.glob('*.py')]:
        tree = ast.parse(path.read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                if node.level == 0 and node.module.startswith(('landing_', 'mission_', 'ai_')):
                    require(node.module in local_modules, f'{path} imports missing {node.module}')
                    checked['imports'] += 1

    require(not any('ai_' in p.name for p in ROOT.rglob('*')), 'Legacy ai_ filenames remain')
    require((ROOT / 'launch_mission.sh').is_file(), 'Mission launcher missing')
    require((ROOT / 'view_landing_camera.sh').is_file(), 'Camera launcher missing')
    require((VISION_DIR / 'landing_camera_viewer.py').is_file(), 'Camera viewer missing')
    require('landing_camera_viewer.py' in (ROOT / 'view_landing_camera.sh').read_text(),
            'Camera launcher points to the wrong file')
    world = WORLD_DIR / 'mission_vision_landing.sdf'
    require(world.is_file(), 'Active world missing')
    require((WORLD_DIR / 'landing_pad.png').is_file(), 'Landing pad image missing')
    require((VISION_DIR / 'landing_detector.onnx').stat().st_size > 1_000_000, 'ONNX model missing or empty')

    world_tree = ET.parse(world)
    pad = world_tree.find('.//model[@name="landing_pad"]')
    require(pad is not None, 'Landing pad model missing in active world')
    pad_xyz = list(map(float, pad.findtext('pose').split()[:3]))
    require(abs(pad_xyz[0] - 6) < 1e-6 and abs(pad_xyz[1] - 6) < 1e-6, f'Unexpected pad position {pad_xyz}')

    launch = (ROOT / 'launch_mission.sh').read_text()
    for ref in [
        'mission_vision_landing.sdf',
        'navigation_handoff.py',
        'mission_router.py',
        'mission_landing.py',
        'live_landing_detector.py',
        'landing_detector.onnx',
    ]:
        require(ref in launch, f'Active mission launcher omits {ref}')

    for ref in ['/mission/landing/cmd_vel', '/mission/landing/enable', '/mission/nav/cmd_vel', '/mission/nav/enable']:
        require(ref in launch, f'Expected remapping is missing: {ref}')

    nav = (CTRL_DIR / 'navigation_handoff.py').read_text()
    route = (CTRL_DIR / 'mission_router.py').read_text()
    landing = (CTRL_DIR / 'mission_landing.py').read_text()
    vision = (VISION_DIR / 'live_landing_detector.py').read_text()
    align = (CTRL_DIR / 'landing_alignment.py').read_text()
    for name, body in [('navigation', nav), ('router', route), ('landing', landing)]:
        require("'/mission/handoff'" in body, f'Mission handover missing in {name}')
    require("'/landing/detection'" in vision and "'/landing/detection'" in align,
            'Detector and landing controller topic mismatch')
    require("'LANDING'" in route and "'NAV'" in route, 'Router modes inconsistent')

    sys.path.insert(0, str(CTRL_DIR.parent))
    from drone_controller.dynamic_navigation import AStarPlanner
    map_data = json.loads((ROOT / 'maps/lidar_obstacle_regions.json').read_text())
    walls = [(x['name'], x['x'], x['y'], x['width'], x['height']) for x in map_data['known_walls']]
    route = AStarPlanner(walls).plan((0.0, 0.0), (5.0, 5.0))
    require(len(route) > 1 and route[0] == (0.0, 0.0) and route[-1] == (5.0, 5.0), 'A* smoke test failed')

    print('Static validation passed')
    print('Counts:', checked)
    print('A* smoke-test:', len(route), 'waypoints, start', route[0], 'goal', route[-1])
    print('ROS 2 / Gazebo runtime is NOT tested by this script.')

if __name__ == '__main__':
    run()
