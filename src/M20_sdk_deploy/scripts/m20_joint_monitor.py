import argparse
import math
import os
import sys
import time

import rclpy
from drdds.msg import JointsData
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


TOPIC = "/JOINTS_DATA"
JOINT_NAMES = tuple(
    f"{leg}_{joint}"
    for leg in ("fl", "fr", "hl", "hr")
    for joint in ("hipx", "hipy", "knee", "wheel")
)
STALE_AFTER = 0.5


def format_number(value):
    return f"{value: .4f}" if math.isfinite(value) else "--"


class JointMonitor(Node):
    def __init__(self):
        super().__init__("m20_joint_monitor")
        self.sample = None
        self.received_at = None
        self.stamp_advanced_at = None
        self.stamp = None
        self.frame_count = 0
        self.invalid = 0
        self.bad_channels = 0
        self.subscription = self.create_subscription(
            JointsData, TOPIC, self.read_joints, qos_profile_sensor_data,
        )

    def read_joints(self, message):
        joints = message.data.joints_data
        if len(joints) != len(JOINT_NAMES):
            self.invalid += 1
            return
        now = time.monotonic()
        stamp = message.header.stamp
        timestamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.stamp is not None:
            if timestamp_ns > self.stamp:
                self.stamp_advanced_at = now
            elif timestamp_ns < self.stamp:
                self.stamp_advanced_at = None
        self.stamp = timestamp_ns
        self.received_at = now
        self.frame_count += 1
        self.sample = tuple((joint.position, joint.velocity) for joint in joints)
        self.bad_channels = sum(not (math.isfinite(position) and math.isfinite(velocity))
                                for position, velocity in self.sample)
        if self.bad_channels:
            self.invalid += 1

    def render(self, now, receive_rate):
        age = None if self.received_at is None else now - self.received_at
        if age is None:
            state = "WAITING FOR FEEDBACK"
        elif age > STALE_AFTER:
            state = "STALE (showing last sample)"
        elif self.stamp_advanced_at is None or now - self.stamp_advanced_at > STALE_AFTER:
            state = "STAMP NOT ADVANCING"
        elif self.bad_channels:
            state = f"INVALID VALUES ({self.bad_channels} channels)"
        else:
            state = "LIVE"
        age_text = "--" if age is None else f"{age * 1000:.0f} ms"
        wall_time = time.strftime("%H:%M:%S")
        lines = [
            f"{wall_time}  M20 joint monitor  {TOPIC}  [{state}]",
            f"RX ~{receive_rate:.0f} Hz | Last frame age: {age_text} | Invalid frames: {self.invalid} | Ctrl+C to quit",
            "Firmware feedback; labels follow SDK channel order. No offset or sign conversion.",
            f"{'idx':>3}  {'motor':<12} {'position (rad)':>16} {'position (deg)':>16} {'speed (rad/s)':>16}",
        ]
        for index, name in enumerate(JOINT_NAMES):
            if self.sample is None:
                radians = degrees = speed = "--"
            else:
                position, velocity = self.sample[index]
                radians = format_number(position)
                degrees = format_number(math.degrees(position))
                speed = format_number(velocity)
            lines.append(f"{index:3}  {name:<12} {radians:>16} {degrees:>16} {speed:>16}")
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Continuously display M20 motor feedback (read-only).")
    parser.add_argument("--hz", type=float, default=5.0, help="terminal refresh rate, 0.5 to 20 Hz (default: 5)")
    parser.add_argument("--plain", action="store_true", help="print snapshots without redrawing the terminal")
    options = parser.parse_args()
    if not 0.5 <= options.hz <= 20.0:
        parser.error("--hz must be between 0.5 and 20")

    redraw = sys.stdout.isatty() and os.environ.get("TERM") != "dumb" and not options.plain
    interval = 1.0 / options.hz
    rclpy.init(args=[])
    monitor = None
    try:
        monitor = JointMonitor()
        rendered_at = time.monotonic()
        next_render = rendered_at
        rendered_count = 0
        while rclpy.ok():
            now = time.monotonic()
            if now >= next_render:
                rate = (monitor.frame_count - rendered_count) / max(now - rendered_at, 1e-6)
                prefix = "\033[H\033[2J" if redraw else ""
                print(prefix + monitor.render(now, rate), flush=True)
                rendered_at = now
                rendered_count = monitor.frame_count
                next_render = now + interval
            timeout = max(0.0, min(0.1, next_render - time.monotonic()))
            rclpy.spin_once(monitor, timeout_sec=timeout)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if monitor is not None:
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
