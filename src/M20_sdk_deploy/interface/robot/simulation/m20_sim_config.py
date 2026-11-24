# MuJoCo 仿真和终端关节演示共用的配置。
import math
import os
from pathlib import Path
from xml.etree import ElementTree

# M20 的 MJCF 模型目录，从当前文件定位到 SDK 包内。
MJCF_DIR = Path(__file__).resolve().parents[3] / "M20_description" / "m20_mjcf" / "mjcf"

# 方向和偏移都按 JOINT_NAMES 排列。MuJoCo 与 SDK 同向取 1，反向取 -1。
JOINT_DIR = (1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1)
# SDK 零位在 MuJoCo 坐标里对应的角度（度）。
POS_OFFSET_DEG = (-25, 229, 160, 0, 25, -131, -200, 0, -25, -229, -160, 0, 25, 131, 200, 0)

# 关节名称
JOINT_NAMES = tuple(f"{leg}_{joint}" for leg in ("fl", "fr", "hl", "hr")
                    for joint in ("hipx", "hipy", "knee", "wheel"))
# 轮电机在 JOINT_NAMES 里的下标。
WHEEL_INDICES = tuple(index for index, name in enumerate(JOINT_NAMES) if name.endswith("_wheel"))

# 话题和超时设置
DEMO_TOPIC_PREFIX = "/m20_joint_demo_sim"  # 控制指令和仿真反馈的话题前缀
BENCH_TOPIC_PREFIX = "/m20_joint_bench_sim"  # 与 m20_joint_bench 的 --sim 话题一致
DEMO_FEEDBACK_TIMEOUT = 0.5  # 终端判定反馈过期的时限（秒），超时后停止发指令
DEMO_COMMAND_TIMEOUT = 0.5  # 仿真端的指令超时（秒），超时后保持当前腿关节角度，轮速目标归零

# 关节演示的控制增益
DEMO_LEG_KP = 400.0  # 腿关节的位置增益
DEMO_LEG_KD = 5.0   # 腿关节的速度增益
DEMO_WHEEL_KD = 1.0  # 轮电机的速度增益，轮电机的位置增益为 0


# 初始化 ROS 2 前设置演示用的域和本机发现范围。
def configure_demo_discovery():
    os.environ.setdefault("ROS_DOMAIN_ID", "52")  # 未指定 ROS 域时用 52
    # 按 ROS_DISTRO 选择限制发现范围的环境变量。
    if os.environ.get("ROS_DISTRO") in {"foxy", "galactic", "humble", "iron"}:
        os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
    else:
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)


# 返回以腿关节名为键的字典，值为 SDK 坐标下的（下限，上限），单位是 rad。
def demo_position_limits():
    model = ElementTree.parse(MJCF_DIR / "M20.xml")
    # 按模型里的关节名收集 range，端点用弧度。
    ranges = {joint.attrib["name"]: joint.attrib["range"]
              for joint in model.findall("./worldbody//joint") if "range" in joint.attrib}
    result = {}
    # 角度限位只取腿关节。
    for index, name in enumerate(JOINT_NAMES):
        if index not in WHEEL_INDICES:
            offset = math.radians(POS_OFFSET_DEG[index])  # 零位偏移先转成弧度
            # 模型关节名带 _joint 后缀，限位端点减去零位偏移再乘方向系数。
            endpoints = [(float(bound) - offset) * JOINT_DIR[index]
                         for bound in ranges[f"{name}_joint"].split()]
            # 方向系数为 -1 时两端会对调，重新取上下限。
            result[name] = min(endpoints), max(endpoints)
    return result
