#!/usr/bin/env python3
"""Publish SAN-60 sweep traces as portable JSON on a ROS1 String topic."""

import json
import os
import subprocess

import rospy
from std_msgs.msg import String


def default_helper_path():
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "bin", "harogic_capture"
    )


def helper_command():
    return [
        rospy.get_param("~capture_helper", default_helper_path()),
        "--start-hz",
        str(rospy.get_param("~start_hz", 2.4e9)),
        "--stop-hz",
        str(rospy.get_param("~stop_hz", 2.5e9)),
        "--rbw-hz",
        str(rospy.get_param("~rbw_hz", 100e3)),
        "--ref-level-dbm",
        str(rospy.get_param("~ref_level_dbm", 0.0)),
        "--fps",
        str(rospy.get_param("~fps", 100.0)),
        "--frames",
        "0",
    ]


def stop_process(process):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main():
    rospy.init_node("san60_spectrum_publisher")
    topic = rospy.get_param("~topic", "/san60/spectrum")
    publisher = rospy.Publisher(topic, String, queue_size=10)
    command = helper_command()
    helper = command[0]
    if not os.path.isfile(helper) or not os.access(helper, os.X_OK):
        raise SystemExit("capture helper is missing or not executable: {}".format(helper))

    rospy.loginfo("Opening SAN-60 with: %s", " ".join(command))
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=None,
        text=True,
        bufsize=1,
    )
    sequence = 0
    try:
        for line in process.stdout:
            if rospy.is_shutdown():
                break
            try:
                message = json.loads(line)
            except json.JSONDecodeError as exc:
                rospy.logerr("Invalid capture JSON: %s", exc)
                continue
            if message.get("type") != "spectrum":
                rospy.logwarn("Ignoring unexpected capture message")
                continue
            message["sequence"] = sequence
            publisher.publish(
                String(
                    data=json.dumps(
                        message,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                )
            )
            if sequence == 0 or sequence % 500 == 0:
                rospy.loginfo(
                    "Published frame=%d points=%d peak=%.3fMHz/%.1fdBm",
                    sequence,
                    len(message.get("powers_dbm", [])),
                    float(message.get("peak_hz", 0.0)) / 1e6,
                    float(message.get("peak_dbm", 0.0)),
                )
            sequence += 1
    finally:
        stop_process(process)

    if not rospy.is_shutdown() and process.returncode not in (0, -15):
        raise SystemExit("capture helper exited with {}".format(process.returncode))


if __name__ == "__main__":
    main()
