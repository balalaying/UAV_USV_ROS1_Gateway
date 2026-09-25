#!/usr/bin/env python3
"""Receive compressed spectrum frames and draw them with Tkinter."""

import argparse
import datetime
import json
import os
import queue
import socket
import threading
import time
import tkinter as tk

import rosgraph
import rospy
from std_msgs.msg import String

from protocol import ProtocolError, recv_frame


class SpectrumServer(threading.Thread):
    def __init__(
        self,
        host,
        port,
        allow_source,
        output_queue,
        status_queue,
        stop_event,
        ros_publisher,
    ):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.allow_source = allow_source
        self.output_queue = output_queue
        self.status_queue = status_queue
        self.stop_event = stop_event
        self.ros_publisher = ros_publisher
        self.server_socket = None

    @staticmethod
    def _replace_queue(target_queue, item):
        try:
            target_queue.put_nowait(item)
        except queue.Full:
            try:
                target_queue.get_nowait()
            except queue.Empty:
                pass
            target_queue.put_nowait(item)

    def status(self, text):
        self._replace_queue(self.status_queue, text)
        print(text, flush=True)

    def run(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(2)
            self.server_socket.settimeout(1.0)
        except OSError as exc:
            self.status("监听失败: {}".format(exc))
            return
        self.status("等待 RX 数据: {}:{}".format(self.host, self.port))
        while not self.stop_event.is_set():
            try:
                client, address = self.server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            source_ip = address[0]
            if self.allow_source and source_ip != self.allow_source:
                self.status("拒绝未授权来源: {}".format(source_ip))
                client.close()
                continue
            self.status("RX 已连接: {}".format(source_ip))
            client.settimeout(2.0)
            try:
                while not self.stop_event.is_set():
                    try:
                        message, byte_count = recv_frame(client)
                    except socket.timeout:
                        continue
                    try:
                        payload = json.dumps(message, separators=(",", ":"))
                        self.ros_publisher.publish(String(data=payload))
                    except (TypeError, ValueError, rospy.ROSException) as exc:
                        self.status("ROS 发布失败: {}".format(exc))
                    message["_wire_bytes"] = byte_count
                    message["_received_at"] = time.time()
                    self._replace_queue(self.output_queue, message)
            except (EOFError, OSError, ProtocolError) as exc:
                self.status("RX 连接断开: {}".format(exc))
            finally:
                client.close()

    def close(self):
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass


class SpectrumWindow:
    BACKGROUND = "#0b1220"
    GRID = "#334155"
    TEXT = "#dbeafe"
    TRACE = "#22d3ee"
    PEAK = "#fb7185"

    def __init__(self, args, ros_publisher):
        self.args = args
        self.root = tk.Tk()
        self.root.title(args.window_title)
        self.root.geometry("1200x720")
        self.root.configure(bg=self.BACKGROUND)
        self.status_var = tk.StringVar(value="正在启动接收服务…")
        self.detail_var = tk.StringVar(value="尚未收到频谱帧")
        tk.Label(
            self.root,
            text=args.window_title,
            bg=self.BACKGROUND,
            fg=self.TEXT,
            font=("Sans", 18, "bold"),
        ).pack(pady=(10, 0))
        tk.Label(
            self.root,
            textvariable=self.detail_var,
            bg=self.BACKGROUND,
            fg="#93c5fd",
            font=("Sans", 11),
        ).pack(pady=4)
        self.canvas = tk.Canvas(
            self.root, bg=self.BACKGROUND, highlightthickness=0, height=580
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=12, pady=4)
        tk.Label(
            self.root,
            textvariable=self.status_var,
            bg=self.BACKGROUND,
            fg="#a7f3d0",
            anchor="w",
        ).pack(fill=tk.X, padx=16, pady=(0, 10))
        self.frames = queue.Queue(maxsize=2)
        self.statuses = queue.Queue(maxsize=4)
        self.stop_event = threading.Event()
        self.server = SpectrumServer(
            args.listen_host,
            args.listen_port,
            args.allow_source,
            self.frames,
            self.statuses,
            self.stop_event,
            ros_publisher,
        )
        self.last_frame_time = None
        self.smoothed_fps = 0.0
        self.latest_frame = None
        self.server.start()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self.poll)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def poll(self):
        try:
            while True:
                self.status_var.set(self.statuses.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self.latest_frame = self.frames.get_nowait()
                now = time.monotonic()
                if self.last_frame_time is not None and now > self.last_frame_time:
                    instant_fps = 1.0 / (now - self.last_frame_time)
                    self.smoothed_fps = (
                        instant_fps
                        if self.smoothed_fps == 0
                        else 0.8 * self.smoothed_fps + 0.2 * instant_fps
                    )
                self.last_frame_time = now
        except queue.Empty:
            pass
        if self.latest_frame is not None:
            self.redraw()
        if not self.stop_event.is_set():
            self.root.after(100, self.poll)

    @staticmethod
    def format_frequency(hz):
        if abs(hz) >= 1e9:
            return "{:.3f} GHz".format(hz / 1e9)
        if abs(hz) >= 1e6:
            return "{:.3f} MHz".format(hz / 1e6)
        return "{:.3f} kHz".format(hz / 1e3)

    def redraw(self):
        frame = self.latest_frame
        if frame is None:
            return
        powers = frame.get("powers_dbm")
        if not isinstance(powers, list) or len(powers) < 2:
            self.status_var.set("收到无效频谱帧")
            return
        width = max(600, self.canvas.winfo_width())
        height = max(360, self.canvas.winfo_height())
        left, right, top, bottom = 72, 24, 22, 55
        plot_w = width - left - right
        plot_h = height - top - bottom
        min_dbm = self.args.min_dbm
        max_dbm = self.args.max_dbm
        self.canvas.delete("all")

        for index in range(7):
            x = left + plot_w * index / 6
            self.canvas.create_line(x, top, x, top + plot_h, fill=self.GRID)
            freq = frame["start_hz"] + (frame["stop_hz"] - frame["start_hz"]) * index / 6
            self.canvas.create_text(
                x,
                top + plot_h + 20,
                text=self.format_frequency(freq),
                fill=self.TEXT,
                font=("Sans", 9),
            )
        for index in range(7):
            y = top + plot_h * index / 6
            self.canvas.create_line(left, y, left + plot_w, y, fill=self.GRID)
            dbm = max_dbm - (max_dbm - min_dbm) * index / 6
            self.canvas.create_text(
                left - 10,
                y,
                text="{:.0f}".format(dbm),
                fill=self.TEXT,
                anchor="e",
                font=("Sans", 9),
            )
        self.canvas.create_text(
            16,
            top + plot_h / 2,
            text="功率 (dBm)",
            fill=self.TEXT,
            angle=90,
            font=("Sans", 10),
        )

        stride = max(1, len(powers) // max(1, int(plot_w)))
        sampled = powers[::stride]
        coords = []
        denominator = max(1, len(sampled) - 1)
        for index, power in enumerate(sampled):
            clamped = min(max_dbm, max(min_dbm, float(power)))
            x = left + plot_w * index / denominator
            y = top + plot_h * (max_dbm - clamped) / (max_dbm - min_dbm)
            coords.extend((x, y))
        if len(coords) >= 4:
            self.canvas.create_line(*coords, fill=self.TRACE, width=2)

        peak_hz = float(frame.get("peak_hz", 0.0))
        peak_dbm = float(frame.get("peak_dbm", min_dbm))
        if frame["stop_hz"] > frame["start_hz"]:
            peak_x = left + plot_w * (peak_hz - frame["start_hz"]) / (
                frame["stop_hz"] - frame["start_hz"]
            )
            peak_y = top + plot_h * (max_dbm - min(max_dbm, max(min_dbm, peak_dbm))) / (
                max_dbm - min_dbm
            )
            self.canvas.create_oval(
                peak_x - 4,
                peak_y - 4,
                peak_x + 4,
                peak_y + 4,
                fill=self.PEAK,
                outline=self.PEAK,
            )

        timestamp = datetime.datetime.fromtimestamp(
            float(frame.get("captured_at", time.time()))
        ).strftime("%H:%M:%S.%f")[:-3]
        self.detail_var.set(
            "帧 #{seq}  |  {points} 点  |  峰值 {freq} / {power:.1f} dBm  |  "
            "温度 {temp:.1f} °C  |  {fps:.1f} FPS  |  {timestamp}".format(
                seq=frame.get("sequence", "?"),
                points=len(powers),
                freq=self.format_frequency(peak_hz),
                power=peak_dbm,
                temp=float(frame.get("temperature_c", 0.0)),
                fps=self.smoothed_fps,
                timestamp=timestamp,
            )
        )
        self.status_var.set(
            "数据正常：来源 {}，最近一帧 {} 字节".format(
                self.args.allow_source, frame.get("_wire_bytes", 0)
            )
        )

    def close(self):
        self.stop_event.set()
        self.server.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--listen-host", default="192.168.10.10")
    parser.add_argument("--listen-port", type=int, default=5000)
    parser.add_argument("--allow-source", default="192.168.10.32")
    parser.add_argument("--window-title", default="SAN-60 远程实时频谱")
    parser.add_argument("--min-dbm", type=float, default=-140.0)
    parser.add_argument("--max-dbm", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.min_dbm >= args.max_dbm:
        raise SystemExit("min-dbm must be lower than max-dbm")
    old_timeout = socket.getdefaulttimeout()
    socket.setdefaulttimeout(2.0)
    try:
        master_online = rosgraph.is_master_online()
    finally:
        socket.setdefaulttimeout(old_timeout)
    if not master_online:
        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        raise SystemExit(
            "ROS master 不可用: {}；请先启动 roscore".format(master_uri)
        )
    try:
        rospy.init_node("san60_spectrum_receiver")
        ros_publisher = rospy.Publisher(
            "/san60/spectrum", String, queue_size=1
        )
    except Exception as exc:
        raise SystemExit("ROS 初始化失败: {}".format(exc)) from exc
    try:
        SpectrumWindow(args, ros_publisher).run()
    finally:
        rospy.signal_shutdown("spectrum receiver stopped")


if __name__ == "__main__":
    main()
