# M20 固定机身仿真的终端演示，可单独设置腿关节角度和轮速。
import math
from queue import Empty, Queue
import sys
from threading import Thread
import time

import rclpy
from drdds.msg import JointDataCmd, JointsData, JointsDataCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from m20_sim_config import (
    DEMO_FEEDBACK_TIMEOUT, DEMO_LEG_KD, DEMO_LEG_KP, DEMO_TOPIC_PREFIX,
    DEMO_WHEEL_KD, JOINT_NAMES, WHEEL_INDICES, configure_demo_discovery,
    demo_position_limits,
)

PERIOD = 0.02  # 指令发送周期（秒），对应 50 Hz
LEG_STEP = math.radians(30.0) * PERIOD  # 每周期的最大角度变化（rad），按 30°/s 计算
WHEEL_STEP = 2.0 * PERIOD  # 每周期的最大轮速变化（rad/s），按 2 rad/s² 计算
WHEEL_MAX_SPEED = 2.0  # 轮速目标的绝对值上限（rad/s）
WHEEL_RUN_TIME = 2.0  # 每次轮速目标的有效时间（秒）

HELP = """status | limits | arm | stop | disarm | quit
pos <joint> <deg>       absolute leg angle (e.g. pos fl_hipx 24deg)
jog <joint> <deg>       change leg target (e.g. jog fl_hipx +2deg)
wheel <leg> <rad/s>     wheel speed for 2 s (e.g. wheel fr 0.5rad/s)
leg: fl, fr, hl, hr; joints per leg: hipx, hipy, knee (e.g. fr_knee)
"""


# 接收仿真反馈，维护各关节目标，启用后定时下发指令。
class JointDemo(Node):
    def __init__(self):
        super().__init__("m20_joint_demo_cli")
        self.limits = demo_position_limits()  # 从模型换算到 SDK 坐标的腿关节限位（rad）
        self.armed = False  # 是否启用指令发送

        # 反馈、目标和发送值都按 JOINT_NAMES 排列。腿关节用 rad，轮电机用 rad/s。
        self.feedback = None  # 最近一帧有效反馈
        self.feedback_at = -math.inf  # 有效反馈的本地接收时间（秒）
        self.target = [0.0] * len(JOINT_NAMES)  # 各关节要达到的目标
        self.sent = self.target.copy()  # 限制每周期变化量后实际用来发指令的值
        self.wheel_expires = [0.0] * len(JOINT_NAMES)  # 各轮速目标的到期时间（秒）

        # 发送使用话题/JOINTS_CMD，接收使用话题/JOINTS_DATA，队列深度为 1。
        self.publisher = self.create_publisher(JointsDataCmd, f"{DEMO_TOPIC_PREFIX}/JOINTS_CMD", 1)
        self.subscription = self.create_subscription(
            JointsData, f"{DEMO_TOPIC_PREFIX}/JOINTS_DATA", self.read_feedback, 1,
        )
        self.timer = self.create_timer(PERIOD, self.send_targets)  # 每 20 ms 检查并发送目标

    # message 是一帧 JointsData，按顺序取腿关节角度和轮速。
    def read_feedback(self, message):
        values = [joint.velocity if index in WHEEL_INDICES else joint.position
                  for index, joint in enumerate(message.data.joints_data)]
        # 每项都是有限值，才更新反馈和接收时间。
        if len(values) == len(JOINT_NAMES) and all(map(math.isfinite, values)):
            self.feedback = values
            self.feedback_at = time.monotonic()

    # 距上次收到有效反馈的时间未超过 FEEDBACK_TIMEOUT 时返回 True。
    def feedback_fresh(self):
        return time.monotonic() - self.feedback_at <= DEMO_FEEDBACK_TIMEOUT

    # 保持最近反馈的腿关节角度，轮速置零。没有及时收到有效反馈时抛出 ValueError。
    def hold_feedback(self):
        if not self.feedback_fresh():
            raise ValueError("No fresh demo feedback; check that demo-mode MuJoCo is running.")
        self.target = self.feedback.copy()
        for index in WHEEL_INDICES:
            self.target[index] = 0.0
        self.sent = self.target.copy()
        self.wheel_expires = [0.0] * len(JOINT_NAMES)

    # 定时发布整帧指令。反馈超时就解除启用，停止发送。
    def send_targets(self):
        if not self.armed:
            return
        if not self.feedback_fresh():
            self.armed = False
            self.get_logger().warning("Demo feedback timed out; disarmed. Arm again after feedback returns.")
            return
        now = time.monotonic()
        message = JointsDataCmd()
        message.header.stamp = self.get_clock().now().to_msg()  # 消息时间戳用 ROS 时钟
        joints = []
        for index, target in enumerate(self.target):
            joint = JointDataCmd()
            wheel = index in WHEEL_INDICES
            # 轮速到期先把目标置零，再逐步把发送的轮速调回零。
            if wheel and now >= self.wheel_expires[index]:
                self.target[index] = target = 0.0
            step = WHEEL_STEP if wheel else LEG_STEP
            self.sent[index] += max(-step, min(step, target - self.sent[index]))
            # 轮电机发目标速度和 kd，腿关节发目标角度、kp 和 kd。
            if wheel:
                joint.velocity = self.sent[index]
                joint.kd = DEMO_WHEEL_KD
            else:
                joint.position = self.sent[index]
                joint.kp = DEMO_LEG_KP
                joint.kd = DEMO_LEG_KD
            joints.append(joint)
        message.data.joints_data = joints
        self.publisher.publish(message)

    # 打印启用状态和各关节的目标、反馈。腿关节显示为度，轮速显示为 rad/s。
    def status(self):
        state = "armed" if self.armed else "disarmed"
        print(f"{state}, feedback: {'fresh' if self.feedback_fresh() else 'unavailable/stale'}")
        print(f"{'joint':<12} {'target':>11} {'feedback':>11}  units")
        for index, name in enumerate(JOINT_NAMES):
            wheel = index in WHEEL_INDICES
            scale = 1.0 if wheel else 180.0 / math.pi
            # 未启用时目标栏显示 --，反馈缺失或过期时反馈栏也显示 --。
            target = f"{self.target[index] * scale:>11.2f}" if self.armed else f"{'--':>11}"
            actual = f"{self.feedback[index] * scale:>11.2f}" if self.feedback_fresh() else f"{'--':>11}"
            print(f"{name:<12} {target} {actual}  {'rad/s' if wheel else 'deg'}")

    # line 是终端读到的一行命令。quit 返回 False，命令或参数有误时抛出 ValueError。
    def execute(self, line):
        words = line.lower().split()
        if not words:
            return True
        command, *args = words
        # 查询、启停和退出命令都无需参数。
        if command in {"help", "status", "limits", "arm", "stop", "disarm", "quit"} and not args:
            if command == "help":
                print(HELP, end="")
            elif command == "status":
                self.status()
            elif command == "limits":
                # 模型限位以弧度保存，终端换算成度。
                for name, (lower, upper) in self.limits.items():
                    print(f"{name:<12} {math.degrees(lower):7.2f} .. {math.degrees(upper):7.2f} deg")
                print(f"wheels: ±{WHEEL_MAX_SPEED:g} rad/s")
            elif command == "arm":
                # 每次启用都从当前反馈角度开始，轮速目标归零。
                self.hold_feedback()
                self.armed = True
                print("Armed: holding the current leg angles, wheel targets zero.")
            elif command == "stop":
                if not self.armed:
                    raise ValueError("Demo is disarmed.")
                # 切换到保持姿态，继续发送指令。
                self.hold_feedback()
                print("Holding current leg angles; wheel targets zero.")
            elif command == "disarm":
                # 停止发送后，仿真端在指令超时时保持腿关节并停轮。
                self.armed = False
                print("Disarmed; the simulator watchdog will hold legs and stop wheels.")
            elif command == "quit":
                return False
            return True

        # 运动命令需要名称和数值，只在已启用且反馈未超时时接受。
        if command not in {"pos", "jog", "wheel"} or len(args) != 2:
            raise ValueError("Unknown or incomplete command. Type help.")
        if not self.armed or not self.feedback_fresh():
            raise ValueError("Wait for fresh feedback and enter arm first.")
        name, value = args
        if command == "wheel":
            # 腿名补上 _wheel 后查关节。轮速按 rad/s 解析，后缀可省略。
            speed = float(value.removesuffix("rad/s"))
            name = f"{name}_wheel"
            if name not in JOINT_NAMES or not math.isfinite(speed) or abs(speed) > WHEEL_MAX_SPEED:
                raise ValueError(f"Use fl, fr, hl or hr and a speed within ±{WHEEL_MAX_SPEED:g} rad/s.")
            index = JOINT_NAMES.index(name)
            self.target[index] = speed
            # 每次设置轮速都从头计时。
            self.wheel_expires[index] = time.monotonic() + WHEEL_RUN_TIME
            print(f"{name} -> {speed:.2f} rad/s for {WHEEL_RUN_TIME:g} s")
        else:
            if name not in self.limits:
                raise ValueError("Use a leg joint name such as fl_hipx; type limits for all names.")
            index = JOINT_NAMES.index(name)
            # 角度输入按度解析。jog 的增量加在当前目标上。
            angle = math.radians(float(value.removesuffix("deg")))
            if command == "jog":
                angle += self.target[index]
            # 绝对目标或叠加后的目标都要落在模型给出的限位内。
            lower, upper = self.limits[name]
            if not math.isfinite(angle) or not lower <= angle <= upper:
                raise ValueError(f"{name} must stay within {math.degrees(lower):.2f} .. {math.degrees(upper):.2f} deg.")
            self.target[index] = angle
            print(f"{name} -> {math.degrees(angle):.2f} deg")
        return True


# 后台线程读终端。commands 是给主循环用的命令队列，读到 EOF 时放入 quit。
def read_console(commands):
    for line in sys.stdin:
        commands.put(line)
    commands.put("quit")


# 启动演示节点并处理终端命令。正常退出和查看帮助返回 0，传入其他参数返回 2。
def main():
    if sys.argv[1:]:
        print(HELP, end="")
        return 0 if sys.argv[1:] in (["-h"], ["--help"]) else 2
    # 默认用 ROS 域 52，节点发现范围限制在本机。
    configure_demo_discovery()
    rclpy.init(args=[])
    node = JointDemo()
    # 后台线程等终端输入，ROS 回调和命令执行都留在主线程。
    commands = Queue()
    Thread(target=read_console, args=(commands,), daemon=True).start()
    interactive = sys.stdin.isatty()  # 交互终端才显示提示符
    print("M20 MuJoCo joint demo: disarmed. Type status, then arm when feedback is fresh.", flush=True)
    if interactive:
        print("m20> ", end="", flush=True)
    try:
        while rclpy.ok():
            # 每轮先处理一次 ROS 回调，再试着取一条命令。
            rclpy.spin_once(node, timeout_sec=PERIOD)
            try:
                line = commands.get_nowait()
            except Empty:
                continue
            try:
                if not node.execute(line):
                    break
            except ValueError as error:
                print(error)
            print("m20> " if interactive else "", end="", flush=True)
    # Ctrl+C 或 ROS 关闭后，统一销毁节点并清理 ROS 上下文。
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
