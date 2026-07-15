#!/usr/bin/env python3
"""Sample only the native ROS 2 LV-DOT detector process."""

import argparse
import json
from pathlib import Path
import subprocess
import time


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=3600.0)
    parser.add_argument('--interval', type=float, default=1.0)
    return parser.parse_args()


def detector_usage():
    output = subprocess.check_output(
        ['ps', '-eo', 'pcpu=,rss=,args='], text=True
    )
    cpu = 0.0
    rss_kib = 0
    processes = 0
    for line in output.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) != 3:
            continue
        command = fields[2]
        if '/uav_usv_lv_dot_ros2/lv_dot_detector_node' not in command:
            continue
        cpu += float(fields[0])
        rss_kib += int(fields[1])
        processes += 1
    return cpu, rss_kib / 1024.0, processes


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + max(0.0, args.duration)
    with args.output.open('w') as stream:
        while time.monotonic() < deadline:
            cpu, rss_mb, processes = detector_usage()
            sample = {
                'time': time.time(),
                'detector_cpu_percent': cpu,
                'detector_rss_mb': rss_mb,
                'process_count': processes,
            }
            stream.write(json.dumps(sample, sort_keys=True) + '\n')
            stream.flush()
            time.sleep(max(0.1, args.interval))


if __name__ == '__main__':
    main()
