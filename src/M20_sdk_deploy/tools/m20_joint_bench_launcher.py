#!/usr/bin/env python3
"""Start the joint bench, with a local MuJoCo process in simulation mode."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def stop_child(process, stop_signal):
    if process is None or process.poll() is not None:
        return
    process.send_signal(stop_signal)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def run_simulation(controller_path, simulation_path, headless):
    sys.path.insert(0, str(simulation_path.parent))
    from m20_sim_config import configure_demo_discovery

    configure_demo_discovery()
    if headless:
        os.environ["SDK_SIM_HEADLESS"] = "1"

    stopping = False

    def on_signal(_signum, _frame):
        nonlocal stopping
        stopping = True

    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(stop_signal, on_signal)

    simulator = controller = None
    try:
        # Keep MuJoCo alive until the controller has finished its shutdown.
        simulator = subprocess.Popen(
            [sys.executable, str(simulation_path), "--bench"],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        controller = subprocess.Popen([str(controller_path), "--sim"])
        while not stopping and controller.poll() is None:
            simulator_status = simulator.poll()
            if simulator_status is not None:
                print(f"MuJoCo exited ({simulator_status}); closing the bench.", file=sys.stderr)
                return simulator_status if simulator_status >= 0 else 128 - simulator_status
            time.sleep(0.05)
    finally:
        stop_child(controller, signal.SIGINT)
        stop_child(simulator, signal.SIGTERM)
    return controller.returncode


def main():
    args = sys.argv[1:]
    entry = Path(__file__).absolute()
    controller = entry.with_name("m20_joint_bench_node")
    try:
        if "--sim" not in args:
            os.execv(str(controller), [str(controller), *args])
        if args not in (["--sim"], ["--sim", "--headless"]):
            print("Usage: m20_joint_bench --sim [--headless]", file=sys.stderr)
            return 2

        simulation = (entry.parents[2] / "share" / entry.parent.name /
                      "interface" / "robot" / "simulation" / "mujoco_simulation_ros2.py")
        return run_simulation(controller, simulation, "--headless" in args)
    except (OSError, ImportError) as error:
        print(f"Cannot start M20 joint bench: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
