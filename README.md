# 云深处山猫 M20 SDK Deploy

本工作区仅保留四轮足山猫 M20 的 ROS 2 部署包及其依赖。实机请从 [M20 部署说明：Sim-to-Real](src/M20_sdk_deploy/README.md#4-sim-to-real) 开始；在电脑上验证时，请参阅 [MuJoCo 仿真说明](src/M20_sdk_deploy/README.md#3-sim-to-sim)。

> [!NOTE]
> 上游 SDK 提示：因使用 SDK 造成的设备损坏不在保修范围内。

## 目录与入口

| 路径 | 用途 |
| --- | --- |
| [`src/M20_sdk_deploy/main.cpp`](src/M20_sdk_deploy/main.cpp) | `m20_sdk_deploy` 包的 `rl_deploy` 入口；默认键盘，也可切换到手柄或 ROS 2 指令。 |
| [`src/M20_sdk_deploy/interface/`](src/M20_sdk_deploy/interface/) | ROS 2/DDS 实机通信、用户输入接口和 MuJoCo 仿真入口。 |
| [`src/M20_sdk_deploy/state_machine/`](src/M20_sdk_deploy/state_machine/) | M20 控制参数及站立、趴下、阻尼、策略控制等状态。 |
| [`run_policy/`](src/M20_sdk_deploy/run_policy/)、[`policy/`](src/M20_sdk_deploy/policy/)、[`third_party/`](src/M20_sdk_deploy/third_party/) | 策略推理代码、ONNX 模型，以及 ARM 和 x86 构建所需的第三方依赖。 |
| [`src/M20_sdk_deploy/M20_description/`](src/M20_sdk_deploy/M20_description/) | M20 的 MuJoCo 机器人模型和场景。 |
| [`src/drdds/`](src/drdds/) | 部署包依赖的 ROS 2 消息和服务定义；实机部署时与 M20 包一同复制和构建。 |

键盘、手柄键位和 ROS 2 话题示例见 [M20 控制接口](src/M20_sdk_deploy/README.md#5-control-interfaces)。如果本地已有 `docs/`，其中的 M20 系列手册可作设备参考；手册不代表不同型号间的部署兼容性。

原项目的[贡献者](Contributors.md)和[许可证](LICENSE)保留。
