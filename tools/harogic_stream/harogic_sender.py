#!/usr/bin/env python3
"""Run the native HAROGIC collector and stream traces to the RX relay."""

import argparse
import json
import os
import subprocess
import time

from protocol import connect, send_frame


def default_helper_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "bin", "harogic_capture")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--entry-host",
        "--receiver-host",
        "--relay-host",
        dest="entry_host",
        default="192.168.10.31",
    )
    parser.add_argument(
        "--entry-port",
        "--receiver-port",
        "--relay-port",
        dest="entry_port",
        type=int,
        default=5000,
    )
    parser.add_argument("--start-hz", type=float, default=2.4e9)
    parser.add_argument("--stop-hz", type=float, default=2.5e9)
    parser.add_argument("--rbw-hz", type=float, default=100e3)
    parser.add_argument("--ref-level-dbm", type=float, default=0.0)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--frames", type=int, default=0, help="0 streams until interrupted")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--local-preview-host", default="127.0.0.1")
    parser.add_argument(
        "--local-preview-port",
        type=int,
        default=0,
        help="local preview port; 0 disables the duplicate stream",
    )
    parser.add_argument("--capture-helper", default=default_helper_path())
    return parser.parse_args()


def helper_command(args):
    return [
        args.capture_helper,
        "--start-hz", str(args.start_hz),
        "--stop-hz", str(args.stop_hz),
        "--rbw-hz", str(args.rbw_hz),
        "--ref-level-dbm", str(args.ref_level_dbm),
        "--fps", str(args.fps),
        "--frames", str(args.frames),
    ]


def close_socket(sock):
    if sock is not None:
        try:
            sock.close()
        except OSError:
            pass


def send_required(message, sock, host, port, args, label):
    while True:
        if sock is None:
            try:
                sock = connect(host, port, args.connect_timeout)
                print("connected to {}".format(label), flush=True)
            except OSError as exc:
                print("{} unavailable: {}; retrying".format(label, exc), flush=True)
                time.sleep(args.reconnect_delay)
                continue
        try:
            return sock, send_frame(sock, message)
        except OSError as exc:
            print("{} connection lost: {}".format(label, exc), flush=True)
            close_socket(sock)
            sock = None


def send_optional(message, sock, host, port, args, label):
    if sock is None:
        try:
            sock = connect(host, port, min(args.connect_timeout, 1.0))
            print("connected to {}".format(label), flush=True)
        except OSError:
            return None
    try:
        send_frame(sock, message)
        return sock
    except OSError as exc:
        print("{} connection lost: {}".format(label, exc), flush=True)
        close_socket(sock)
        return None


def main():
    args = parse_args()
    if not (0 < args.start_hz < args.stop_hz):
        raise SystemExit("start-hz must be positive and lower than stop-hz")
    if args.rbw_hz <= 0 or args.fps <= 0 or args.frames < 0:
        raise SystemExit("rbw-hz/fps must be positive and frames must not be negative")
    if not os.path.isfile(args.capture_helper) or not os.access(args.capture_helper, os.X_OK):
        raise SystemExit(
            "native capture helper is missing; run tools/harogic_stream/build_capture.sh"
        )

    process = subprocess.Popen(
        helper_command(args),
        stdout=subprocess.PIPE,
        stderr=None,
        text=True,
        bufsize=1,
    )
    sequence = 0
    sock = None
    preview_sock = None
    try:
        for line in process.stdout:
            try:
                message = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError("invalid native capture output: {}".format(exc)) from exc
            if message.get("type") != "spectrum":
                raise RuntimeError("unexpected native capture message")
            message["sequence"] = sequence
            sock, byte_count = send_required(
                message,
                sock,
                args.entry_host,
                args.entry_port,
                args,
                "TX relay",
            )
            if args.local_preview_port > 0:
                preview_sock = send_optional(
                    message,
                    preview_sock,
                    args.local_preview_host,
                    args.local_preview_port,
                    args,
                    "local preview",
                )
            if sequence == 0 or sequence % max(1, int(args.fps * 5)) == 0:
                print(
                    "frame={} peak={:.3f}MHz/{:.1f}dBm temp={:.1f}C bytes={}".format(
                        sequence,
                        message["peak_hz"] / 1e6,
                        message["peak_dbm"],
                        message["temperature_c"],
                        byte_count,
                    ),
                    flush=True,
                )
            sequence += 1
    except KeyboardInterrupt:
        print("stopped", flush=True)
    finally:
        close_socket(sock)
        close_socket(preview_sock)
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    if process.returncode not in (0, -15):
        raise SystemExit("native capture helper exited with {}".format(process.returncode))


if __name__ == "__main__":
    main()
