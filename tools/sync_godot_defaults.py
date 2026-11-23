#!/usr/bin/env python3
"""Synchronize the Godot and installed ROS copies of the shared defaults."""
import argparse
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DESTINATION = ROOT / 'godot/config/defaults.json'
ROS_COPY = ROOT / 'ros2_ws/src/urc_bringup/config/defaults.yaml'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='check both installed copies')
    args = parser.parse_args()
    canonical = (ROOT / 'config/defaults.yaml').read_text(encoding='utf-8')
    source = yaml.safe_load(canonical)
    runtime = {key: source[key] for key in ('contract_version', 'runtime', 'scene', 'sensors', 'perception', 'mission', 'safety')}
    output = json.dumps(runtime, indent=2, sort_keys=True) + '\n'
    for path, content in ((DESTINATION, output), (ROS_COPY, canonical)):
        if args.check:
            if not path.is_file() or path.read_text(encoding='utf-8') != content:
                parser.error(f'{path} differs. Run this command without --check.')
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
