import argparse
import math
import sys
import time

import rclpy
from drdds.msg import ImuData, JointsData
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


JOINT_NAMES = tuple(
    "{}_{}".format(leg, joint)
    for leg in ("fl", "fr", "hl", "hr")
    for joint in ("hipx", "hipy", "knee", "wheel")
)
MAX_REPORT_AGE = 0.5
STANDARD_IMU_TOPIC = "/IMU"
LEGACY_IMU_TOPIC = "/IMU_DATA"


class Stream:
    def __init__(self):
        self.frames = 0
        self.invalid = 0
        self.stamp = None
        self.stamp_advances = 0
        self.last_advance_at = None
        self.last_at = None
        self.latest = None

    def update(self, message, valid):
        self.frames += 1
        self.last_at = time.monotonic()
        if not valid:
            self.invalid += 1
            self.latest = None
            return
        stamp = message.header.stamp
        value = stamp.sec * 1000000000 + stamp.nanosec
        if self.stamp is not None and value > self.stamp:
            self.stamp_advances += 1
            self.last_advance_at = self.last_at
        self.stamp = value
        self.latest = message

    def report(self, name, now):
        age = None if self.last_at is None else now - self.last_at
        fresh = (
            self.latest is not None and age <= MAX_REPORT_AGE and
            self.last_advance_at is not None and now - self.last_advance_at <= MAX_REPORT_AGE
        )
        age_text = "n/a" if age is None else "{:.3f}s".format(age)
        print("{}: frames={}, invalid={}, stamp_advances={}, age={}, {}".format(
            name, self.frames, self.invalid, self.stamp_advances, age_text,
            "live" if fresh else "missing, stale or invalid",
        ))
        return fresh and not self.invalid


class Preflight(Node):
    def __init__(self, imu_topic):
        super().__init__("m20_preflight")
        self.joints = Stream()
        self.imu = Stream()
        self.imu_topic = imu_topic
        self.joint_subscription = self.create_subscription(
            JointsData, "/JOINTS_DATA", self.read_joints, qos_profile_sensor_data,
        )
        imu_type, callback = (
            (ImuData, self.read_legacy_imu) if imu_topic == LEGACY_IMU_TOPIC
            else (Imu, self.read_standard_imu)
        )
        self.imu_subscription = self.create_subscription(
            imu_type, imu_topic, callback, qos_profile_sensor_data,
        )

    def read_joints(self, message):
        joints = message.data.joints_data
        valid = len(joints) == len(JOINT_NAMES) and all(
            math.isfinite(value) for joint in joints for value in (
                joint.position, joint.velocity, joint.torque, joint.motion_temp, joint.driver_temp,
            )
        )
        self.joints.update(message, valid)

    def read_legacy_imu(self, message):
        data = message.data
        valid = all(math.isfinite(value) for value in (
            data.roll, data.pitch, data.yaw, data.omega_x, data.omega_y, data.omega_z,
            data.acc_x, data.acc_y, data.acc_z,
        ))
        self.imu.update(message, valid)

    def read_standard_imu(self, message):
        valid = all(math.isfinite(value) for vector, covariance in (
            (message.angular_velocity, message.angular_velocity_covariance),
            (message.linear_acceleration, message.linear_acceleration_covariance),
        ) if covariance[0] != -1.0 for value in (vector.x, vector.y, vector.z))
        self.imu.update(message, valid)

    def report_standard_imu(self, message):
        quat = message.orientation
        norm = math.sqrt(sum(value * value for value in (quat.x, quat.y, quat.z, quat.w)))
        if message.orientation_covariance[0] == -1.0:
            orientation = "unavailable (covariance[0] = -1)"
        elif not math.isclose(norm, 1.0, rel_tol=0.01):
            orientation = "invalid quaternion (norm={:.3f})".format(norm)
        else:
            orientation = "available (quaternion norm={:.3f})".format(norm)
        print("IMU frame_id={!r}, orientation={}".format(message.header.frame_id, orientation))
        for name, vector, covariance, unit in (
            ("angular_velocity", message.angular_velocity, message.angular_velocity_covariance, "rad/s"),
            ("linear_acceleration", message.linear_acceleration, message.linear_acceleration_covariance, "m/s^2"),
        ):
            reading = "unavailable" if covariance[0] == -1.0 else "({:.4f}, {:.4f}, {:.4f}) {}".format(
                vector.x, vector.y, vector.z, unit,
            )
            print("IMU {}={}".format(name, reading))

    def report(self):
        now = time.monotonic()
        joint_ok = self.joints.report("/JOINTS_DATA", now)
        imu_ok = self.imu.report(self.imu_topic, now)
        if self.imu_topic != LEGACY_IMU_TOPIC and self.imu.latest is not None:
            self.report_standard_imu(self.imu.latest)
        print("/JOINTS_CMD discoverable publishers: {} (not a control authority check)".format(
            self.count_publishers("/JOINTS_CMD"),
        ))
        if self.joints.latest is not None:
            print("Raw feedback; channel labels follow SDK ordering and are not verified by this probe.")
            print(" idx  SDK label       position     velocity       torque   motor_temp  driver_temp      status")
            for index, joint in enumerate(self.joints.latest.data.joints_data):
                print("{:4d}  {:<12}  {:10.4f}   {:10.4f}   {:10.4f}   {:10.1f}   {:11.1f}      0x{:04x}".format(
                    index, JOINT_NAMES[index], joint.position, joint.velocity, joint.torque,
                    joint.motion_temp, joint.driver_temp, joint.status_word,
                ))
        ok = joint_ok and imu_ok
        print("{}: telemetry only; this does not authorize actuation.".format(
            "PASS" if ok else "CHECK FAILED",
        ))
        return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description="Read-only M20 telemetry check. Never sends motor commands.")
    parser.add_argument("--seconds", type=float, default=5.0, help="sampling duration (1 to 60 s)")
    parser.add_argument("--imu-topic", choices=(STANDARD_IMU_TOPIC, LEGACY_IMU_TOPIC),
                        default=STANDARD_IMU_TOPIC, help="use /IMU for M20 Pro or /IMU_DATA for the legacy SDK topic")
    options = parser.parse_args()
    if not 1.0 <= options.seconds <= 60.0:
        parser.error("--seconds must be between 1 and 60")

    rclpy.init(args=[])
    node = None
    try:
        node = Preflight(options.imu_topic)
        deadline = time.monotonic() + options.seconds
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return node.report()
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        return 130
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
