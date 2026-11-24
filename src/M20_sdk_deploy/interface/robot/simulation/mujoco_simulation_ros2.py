import os
import sys
import time
import socket
import struct
import threading
from pathlib import Path
from xml.etree import ElementTree
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
from sensor_msgs.msg import Imu
from m20_pro_urdf import read_motor_joints, revolute_limits
from m20_sim_config import (
    BENCH_TOPIC_PREFIX, DEMO_COMMAND_TIMEOUT, DEMO_LEG_KD, DEMO_LEG_KP, DEMO_TOPIC_PREFIX,
    DEMO_WHEEL_KD, JOINT_DIR as SIM_JOINT_DIR, JOINT_NAMES, MJCF_DIR, POS_OFFSET_DEG as SIM_POS_OFFSET_DEG,
    WHEEL_INDICES, configure_demo_discovery,
)



MODEL_NAME = "M20"
# 当前脚本所在的目录
CURRENT_DIR = Path(__file__).resolve().parent

# 从脚本目录找到普通仿真用的楼梯场景。
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "M20_description" / "m20_mjcf" / "mjcf" / "M20_stair.xml"

# 加载模型时传入绝对路径字符串。
XML_PATH = str(XML_PATH.resolve())
USE_VIEWER = os.environ.get("SDK_SIM_HEADLESS", "0") != "1"  # 无窗口模式
DT = 0.001  # 仿真步长（秒）
RENDER_INTERVAL = 50  # 每隔多少步刷新一次窗口

# 关节标定参数，用于在 MuJoCo 和 SDK 的关节坐标之间换算。
JOINT_DIR = np.array(SIM_JOINT_DIR, dtype=np.float32)  # 同向为 1，反向为 -1
POS_OFFSET_DEG = np.array(SIM_POS_OFFSET_DEG, dtype=np.float32)  # SDK 零位对应的模型角度（度）
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi  # 换算用的零位偏移（rad）
PRO_MODEL_DIR = MJCF_DIR.parents[1] / "M20_Pro"

# 初始关节角度（rad）
JOINT_INIT = {
    "M20": np.array([-0.438, -1.16, 2.76, 0,
                     0.438, -1.16, 2.76, 0,
                     -0.438, 1.16, -2.76, 0,
                     0.438, 1.16, -2.76, 0], dtype=np.float32),
}


def load_pro_bench_model():
    model_path = PRO_MODEL_DIR / 'mjcf' / 'M20.xml'
    root = ElementTree.parse(model_path).getroot()
    root.find('compiler').set('meshdir', str(model_path.parent / 'meshes'))
    motor_joints = read_motor_joints(PRO_MODEL_DIR / 'urdf' / 'M20.urdf')
    model_joints = {joint.attrib['name']: joint for joint in root.findall('./worldbody//joint')}
    for name in JOINT_NAMES:
        joint_name = f'{name}_joint'
        urdf_joint = motor_joints[joint_name]
        model_joint = model_joints[joint_name]
        axis_text = urdf_joint.find('axis').attrib['xyz']
        axis = np.fromstring(axis_text, sep=' ')
        rpy = np.fromstring(urdf_joint.find('origin').attrib['rpy'], sep=' ')
        zero_rotation = np.dot(rpy, axis)
        if not np.allclose(rpy, zero_rotation * axis):
            raise ValueError(f'{joint_name} zero offset must rotate about its URDF axis')
        # ref 是 MJCF 原始姿态对应的 URDF 读数，关节旋转量为 qpos - ref。
        model_joint.set('ref', str(-zero_rotation))
        model_joint.set('axis', axis_text)
        if urdf_joint.attrib['type'] == 'revolute':
            lower, upper = revolute_limits(urdf_joint)
            model_joint.set('range', f'{lower} {upper}')

    fixture = ElementTree.SubElement(root, 'equality')
    ElementTree.SubElement(fixture, 'weld', name='bench_mount', body1='base_link')
    return mujoco.MjModel.from_xml_string(ElementTree.tostring(root, encoding='unicode'))


# 接收 ROS 2 关节指令，推进 M20 仿真，发布 IMU 和关节反馈。
class MuJoCoSimulationNode(Node):
    # model_key 选初始站姿，xml_path 指定普通模式的 MJCF 文件。
    # demo 和 bench 都用固定机身模型，bench 按实机接口发布反馈。
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH,
                 demo: bool = False,
                 bench: bool = False):

        self.bench = bench
        self.demo = demo or bench
        topic_prefix = BENCH_TOPIC_PREFIX if bench else DEMO_TOPIC_PREFIX if demo else ""
        super().__init__(topic_prefix[1:] or 'mujoco_simulation')
        self.last_demo_command = None  # 最近一条有效演示指令的接收时间（单调时钟，秒）
        if bench:
            self.model = load_pro_bench_model()
            self.joint_direction = np.ones_like(JOINT_DIR)
            self.position_offset = np.zeros_like(POS_OFFSET_RAD)
        else:
            if demo:
                xml_path = str(MJCF_DIR / "M20_joint_demo.xml")
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.joint_direction = JOINT_DIR
            self.position_offset = POS_OFFSET_RAD
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)  # 关节、传感器和执行器的运行数据

        self.actuator_ids = [a for a in range(self.model.nu)]  # 执行器编号 0～15
        self.dof_num = len(self.actuator_ids)  # 受控关节数，含轮电机
        assert self.dof_num == 16, "Expected 16 DOF for M20"

        # 普通模式用预设站姿，演示模式从模型初始姿态开始。
        if not self.demo:
            self._set_initial_pose(model_key)

        # 每个控制量都用一列保存，每行对应一个关节。
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)   # Kp
        self.kd_cmd = np.zeros_like(self.kp_cmd)                # Kd
        self.pos_cmd = np.zeros_like(self.kp_cmd)               # MuJoCo 坐标下的目标角度（rad）
        self.vel_cmd = np.zeros_like(self.kp_cmd)               # MuJoCo 坐标下的目标角速度（rad/s）
        self.tau_ff = np.zeros_like(self.kp_cmd)                # 指令给出的前馈力矩（N·m）
        self.input_tq = np.zeros_like(self.kp_cmd)              # 本步算出的控制力矩（N·m）
        if self.demo:
            self._hold_demo_pose()

        self.last_base_linvel = np.zeros((3, 1), np.float64)  # 预留的机身线速度缓存（m/s）
        self.timestamp = 0.0  # 仿真时间（秒），用来给反馈消息打时间戳

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        if bench:
            self.imu_pub = self.create_publisher(Imu, f'{topic_prefix}/IMU', qos_profile_sensor_data)
        else:
            self.imu_pub = self.create_publisher(ImuData, f'{topic_prefix}/IMU_DATA', qos_profile_sensor_data)
        # 发布关节反馈，队列深度在演示模式下为 1，普通模式下为 200。
        self.joints_pub = self.create_publisher(JointsData, f'{topic_prefix}/JOINTS_DATA', 1 if self.demo else 200)

        # 订阅关节指令，队列深度在演示模式下为 1，普通模式下为 50。
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            f'{topic_prefix}/JOINTS_CMD',
            self._cmd_callback,
            1 if self.demo else 50
        )

        self.viewer = None  # 无窗口模式下保持为 None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def _set_initial_pose(self, key: str):
        """用与 PyBullet 脚本一致的关节角度设置初始站姿。"""
        qpos0 = self.data.qpos.copy()
        # qpos 前三项是机身位置，接着四项是机身四元数，其余是关节角度。
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
        qpos0[:3] = np.array([0, 0, 0.2])  # 机身初始高度为 0.2 m
        qpos0[3:7] = np.array([1, 0, 0, 0])  # 初始姿态用单位四元数
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)  # 刷新初始姿态对应的运动学和传感器数据

    # 保持当前腿关节角度，把目标速度和前馈力矩清零。
    def _hold_demo_pose(self):
        self.pos_cmd.flat = self.data.qpos[7:7 + self.dof_num]
        self.vel_cmd.fill(0)
        self.tau_ff.fill(0)
        self.kp_cmd.fill(DEMO_LEG_KP)
        self.kd_cmd.fill(DEMO_LEG_KD)
        # 轮电机只用速度控制，位置增益置零。
        self.kp_cmd[list(WHEEL_INDICES)] = 0
        self.kd_cmd[list(WHEEL_INDICES)] = DEMO_WHEEL_KD

    def _cmd_callback(self, msg: JointsDataCmd):
        """接收关节指令，把目标角度和速度换算到 MuJoCo 坐标。
        msg 是一帧 JointsDataCmd，按关节顺序给出各关节的目标和增益。
        """
        if len(msg.data.joints_data) != 16:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return
        # 演示模式下，指令各字段都是有限值才记下接收时间。
        if self.demo:
            if not all(np.isfinite((joint.position, joint.velocity, joint.kp, joint.kd, joint.torque)).all()
                       for joint in msg.data.joints_data):
                return
            self.last_demo_command = time.monotonic()

        # 存放 SDK 坐标下的目标角度（rad）和角速度（rad/s）。
        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque  # 前馈力矩直接沿用消息里的值

        # 先换方向再加零位偏移。
        self.pos_cmd.flat = pub_pos * self.joint_direction + self.position_offset
        self.vel_cmd.flat = pub_vel * self.joint_direction

    # 主循环负责仿真步进和 ROS 2 回调。
    def start(self):
        step = 0  # 已推进的仿真步数
        last_time = time.time()  # 上次推进时的系统时间（秒）
        while rclpy.ok() and (not self.demo or self.viewer is None or self.viewer.is_running()):
            # 按系统时间控制步进节奏，每轮最多推进一步。
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                # 演示指令超时后，保持当前腿关节角度，把轮速目标归零。
                if (self.last_demo_command is not None and
                        time.monotonic() - self.last_demo_command > DEMO_COMMAND_TIMEOUT):
                    self._hold_demo_pose()
                    self.last_demo_command = None
                    self.get_logger().info("Demo command timeout: holding legs, stopping wheels")
                # 先写入控制力矩，再让 MuJoCo 推进一步。
                self._apply_joint_torque()
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT  # 时间戳按仿真步数累计

                # 每 5 步发布一次反馈，按仿真时间算是 200 Hz。
                if step % 5 == 0:
                    self._publish_robot_state()

                # 窗口按 RENDER_INTERVAL 的步数间隔刷新。
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

            # 处理就绪的 ROS 2 回调。
            rclpy.spin_once(self, timeout_sec=0.0)

    # 用位置误差、速度误差和前馈项算关节控制力矩。
    def _apply_joint_torque(self):
        # qpos 前 7 项、qvel 前 6 项属于浮动机身，从后面取各关节的状态。
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
                self.kp_cmd * (self.pos_cmd - q) +
                self.kd_cmd * (self.vel_cmd - dq) +
                self.tau_ff
        )

        # 展平成一维数组，按执行器顺序写入 MuJoCo 的控制缓冲区。
        self.data.ctrl[:] = self.input_tq.flatten()

    def quaternion_to_euler(self, q):
        """把四元数换成横滚、俯仰、偏航角，返回值以弧度计。
        q 按 MuJoCo 的顺序传入，依次为 (w, x, y, z)。
        """
        w, x, y, z = q

        # 横滚角，绕 X 轴转动。
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # 俯仰角，绕 Y 轴转动。
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止舍入误差让 arcsin 的输入越界
        pitch = np.arcsin(t2)

        # 偏航角，绕 Z 轴转动。
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_imu(self, stamp):
        # 传感器数组前四项是机身在世界坐标系中的四元数 (w, x, y, z)。
        q_world = self.data.sensordata[:4]
        body_acc = self.data.sensordata[4:7]  # IMU 局部坐标系下的加速度（m/s²）
        angvel_b = self.data.sensordata[7:10]  # IMU 局部坐标系下的角速度（rad/s）

        if self.bench:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'imu_site'
            w, x, y, z = map(float, q_world)
            imu_msg.orientation.w = w
            imu_msg.orientation.x = x
            imu_msg.orientation.y = y
            imu_msg.orientation.z = z
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = map(float, angvel_b)
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = map(float, body_acc)
            self.imu_pub.publish(imu_msg)
            return

        rpy_rad = self.quaternion_to_euler(q_world)

        # 姿态角在 IMU 消息里用度表示。
        rpy_deg = [angle * (180.0 / 3.141592653589793) for angle in rpy_rad]

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0])
        imu_msg.data.omega_y = float(angvel_b[1])
        imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0])
        imu_msg.data.acc_y = float(body_acc[1])
        imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)

    def _publish_robot_state(self):
        # 关节反馈和 IMU 用同一步的仿真时间戳。
        sec = int(self.timestamp)
        stamp = Time(sec=sec, nanosec=int((self.timestamp - sec) * 1e9))
        self._publish_imu(stamp)

        # 关节反馈
        q = self.data.qpos[7:7 + self.dof_num]  # 当前关节角度（rad）
        dq = self.data.qvel[6:6 + self.dof_num]  # 当前关节角速度（rad/s）
        tau = self.input_tq.flatten()  # 力矩字段用本步算出的控制力矩（N·m）

        # 转回 SDK 坐标。先减零位偏移再换方向。
        pub_pos = (q - self.position_offset) * self.joint_direction
        pub_vel = dq * self.joint_direction
        pub_tau = tau * self.joint_direction

        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # 用四个空格占位
            joint.data_id = 0  # 数据编号用 0 占位
            joint.status_word = 1  # 固定填正常状态
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # 电机温度占位值
            joint.driver_temp = 45.0  # 驱动器温度占位值
        self.joints_pub.publish(joints_msg)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)  # 数组按定点格式显示，精度为 4 位小数
    # 关节演示在 ROS 2 初始化前设置本机发现，并用空参数列表启动。
    bench = sys.argv[1:] == ["--bench"]
    demo = os.environ.get("M20_SIM_JOINT_DEMO") == "1"
    if demo or bench:
        configure_demo_discovery()
    rclpy.init(args=[] if demo or bench else None)
    sim_node = MuJoCoSimulationNode(demo=demo, bench=bench)
    try:
        sim_node.start()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # ROS 可能在步进循环检查上下文后、收发消息前完成关闭。
        if rclpy.ok():
            raise
    finally:
        # 退出时先关窗口，再销毁节点和清理 ROS 2 上下文。
        if sim_node.viewer is not None:
            sim_node.viewer.close()
        sim_node.destroy_node()
        rclpy.try_shutdown()
