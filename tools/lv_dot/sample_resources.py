#!/usr/bin/env python3
"""Sample host scene and isolated LV-DOT container resource usage."""

import argparse
import json
from pathlib import Path
import re
import subprocess
import time


PROCESS_PATTERN = re.compile(
    r'(gz sim|gz_pointcloud_bridge|mid360_preprocessor|gz_sensor_bridge|'
    r'uav_camera_|boat_nav2_interface|tf_topic_relay|static_transform_publisher|'
    r'lv_dot_|ground_truth_adapter|perception_fusion|perception_source_mux|'
    r'onboardDetector|roscore|roslaunch)'
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument(
        '--docker-bin',
        default='/home/dji/.local/lib/docker-static/docker',
    )
    parser.add_argument('--container', default='uav_usv_lv_dot')
    return parser.parse_args()


def host_usage():
    output = subprocess.check_output(
        ['ps', '-eo', 'pcpu=,rss=,args='], text=True
    )
    cpu = 0.0
    rss_kib = 0
    for line in output.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) != 3 or not PROCESS_PATTERN.search(fields[2]):
            continue
        if 'sample_resources.py' in fields[2]:
            continue
        cpu += float(fields[0])
        rss_kib += int(fields[1])
    return cpu, rss_kib / 1024.0


def size_megabytes(value):
    match = re.match(r'([0-9.]+)([KMG]i?B)', value.strip())
    if match is None:
        return 0.0
    number = float(match.group(1))
    unit = match.group(2)
    if unit.startswith('K'):
        return number / 1024.0
    if unit.startswith('G'):
        return number * 1024.0
    return number


def container_usage(docker_bin, container):
    try:
        output = subprocess.check_output(
            [docker_bin, 'stats', '--no-stream', '--format',
             '{{.CPUPerc}}|{{.MemUsage}}', container],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        ).strip()
        cpu_text, memory_text = output.split('|', 1)
        return (
            float(cpu_text.rstrip('%')),
            size_megabytes(memory_text.split('/', 1)[0]),
        )
    except (OSError, subprocess.SubprocessError, ValueError):
        return None, None


def main():
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + max(0.0, args.duration)
    with args.output.open('w') as stream:
        while time.monotonic() < deadline:
            host_cpu, host_rss = host_usage()
            lv_dot_cpu, lv_dot_memory = container_usage(
                args.docker_bin, args.container
            )
            sample = {
                'time': time.time(),
                'host_cpu_percent': host_cpu,
                'host_rss_mb': host_rss,
            }
            if lv_dot_cpu is not None:
                sample['lv_dot_cpu_percent'] = lv_dot_cpu
                sample['lv_dot_memory_mb'] = lv_dot_memory
            stream.write(json.dumps(sample, sort_keys=True) + '\n')
            stream.flush()
            time.sleep(max(0.1, args.interval))


if __name__ == '__main__':
    main()
