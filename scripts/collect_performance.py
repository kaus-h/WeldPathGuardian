#!/usr/bin/env python3
"""Collect per-message WeldPath Guardian latency and recovery metrics."""

import argparse
import json
import math
import os
import platform
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from weld_interfaces.msg import FilteredSeam, SeamObservation, SystemStatus, WeldPlan


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEED = 42


SCENARIOS = [
    {
        "name": "clean",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": [
            "scenario:=clean",
            "noise_stddev:=0.0",
            "dropout_ratio:=0.0",
            f"seed:={DEFAULT_SEED}",
        ],
    },
    {
        "name": "gaussian_noise_0.002",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": [
            "scenario:=gaussian_noise",
            "noise_stddev:=0.002",
            "dropout_ratio:=0.02",
            f"seed:={DEFAULT_SEED}",
        ],
    },
    {
        "name": "gaussian_noise_0.006",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": [
            "scenario:=gaussian_noise",
            "noise_stddev:=0.006",
            "dropout_ratio:=0.06",
            f"seed:={DEFAULT_SEED}",
        ],
    },
    {
        "name": "gaussian_noise_0.010",
        "launch": "demo.launch.py",
        "duration": 8.0,
        "args": [
            "scenario:=gaussian_noise",
            "noise_stddev:=0.010",
            "dropout_ratio:=0.10",
            f"seed:={DEFAULT_SEED}",
        ],
    },
    {
        "name": "missing_segment",
        "launch": "fault_demo.launch.py",
        "duration": 7.0,
        "args": ["scenario:=missing_segment", f"seed:={DEFAULT_SEED}"],
    },
    {
        "name": "low_confidence_recovery",
        "launch": "demo.launch.py",
        "duration": 10.0,
        "args": ["scenario:=low_confidence_recovery", f"seed:={DEFAULT_SEED}"],
    },
]


def percentile(values, percentile_value):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil((percentile_value / 100.0) * len(ordered)) - 1)
    return ordered[index]


def stats(values):
    if not values:
        return {"count": 0, "median": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0, "stddev": 0.0}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
        "stddev": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def stamp_age_ms(node, stamp):
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    return (node.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0


def clean_reference_point(x_value):
    t = min(max(x_value / 1.2, 0.0), 1.0)
    point = Point()
    point.x = x_value
    point.y = 0.08 * math.sin(t * 2.2 * math.pi)
    point.z = 0.03 * math.cos(t * math.pi)
    return point


def distance(lhs, rhs):
    return math.sqrt((lhs.x - rhs.x) ** 2 + (lhs.y - rhs.y) ** 2 + (lhs.z - rhs.z) ** 2)


def mean_reference_error(plan):
    if not plan.waypoints:
        return 0.0
    total = 0.0
    for waypoint in plan.waypoints:
        point = waypoint.pose.position
        total += distance(point, clean_reference_point(point.x))
    return total / len(plan.waypoints)


def max_sustained_hz(receive_times):
    if not receive_times:
        return 0.0
    best = 0
    left = 0
    for right, timestamp in enumerate(receive_times):
        while timestamp - receive_times[left] > 1.0:
            left += 1
        best = max(best, right - left + 1)
    return float(best)


def interarrival_jitter_ms(receive_times):
    if len(receive_times) < 3:
        return 0.0
    intervals_ms = [
        (later - earlier) * 1000.0 for earlier, later in zip(receive_times, receive_times[1:])
    ]
    return statistics.pstdev(intervals_ms)


def parse_launch_args(args):
    parsed = {}
    for arg in args:
        if ":=" in arg:
            key, value = arg.split(":=", 1)
            parsed[key] = value
    return parsed


def command_text(command):
    try:
        return subprocess.check_output(command, cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def git_dirty():
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"
    return bool(output.strip())


def total_memory_mb():
    if hasattr(os, "sysconf"):
        try:
            pages = os.sysconf("SC_PHYS_PAGES")
            page_size = os.sysconf("SC_PAGE_SIZE")
            return round((pages * page_size) / (1024.0 * 1024.0))
        except (OSError, ValueError):
            pass
    return "unknown"


def metadata(args):
    compiler = command_text(["c++", "--version"]).splitlines()[0]
    return {
        "git_sha": os.environ.get("WELDPATH_GIT_SHA") or command_text(["git", "rev-parse", "HEAD"]),
        "git_dirty": git_dirty(),
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "compiler": compiler,
        "build_type": args.build_type,
        "platform": platform.platform(),
        "cpu": platform.processor() or platform.machine(),
        "cpu_count": os.cpu_count(),
        "memory_total_mb": total_memory_mb(),
        "python": platform.python_version(),
        "seed": DEFAULT_SEED,
    }


def stop_process(process):
    if process.poll() is not None:
        return
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGTERM)
    else:
        process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()


def start_launch(config, env):
    launch_path = ROOT / "launch" / config["launch"]
    command = ["ros2", "launch", str(launch_path), *config["args"]]
    popen_kwargs = {
        "env": env,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "text": True,
    }
    if os.name == "posix":
        popen_kwargs["preexec_fn"] = os.setsid
    return command, subprocess.Popen(command, **popen_kwargs)


class ScenarioCollector:
    def __init__(self, node):
        self.node = node
        self.raw_receive_times = []
        self.filtered_processing_ms = []
        self.filtered_age_ms = []
        self.plan_processing_ms = []
        self.plan_age_ms = []
        self.path_errors = []
        self.plan_faults = []
        self.statuses = []

        node.create_subscription(SeamObservation, "/seam/raw", self.on_raw, qos_profile_sensor_data)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(FilteredSeam, "/seam/filtered", self.on_filtered, reliable_qos)
        node.create_subscription(WeldPlan, "/weld/plan", self.on_plan, reliable_qos)
        node.create_subscription(SystemStatus, "/weld/status", self.on_status, reliable_qos)

    def on_raw(self, _msg):
        self.raw_receive_times.append(time.monotonic())

    def on_filtered(self, msg):
        self.filtered_processing_ms.append(float(msg.processing_latency_ms))
        self.filtered_age_ms.append(stamp_age_ms(self.node, msg.header.stamp))

    def on_plan(self, msg):
        self.plan_processing_ms.append(float(msg.processing_latency_ms))
        self.plan_age_ms.append(stamp_age_ms(self.node, msg.header.stamp))
        if msg.valid:
            self.path_errors.append(mean_reference_error(msg))
        else:
            self.plan_faults.append(msg.fault_code)

    def on_status(self, msg):
        self.statuses.append(
            {
                "state": msg.state,
                "fault": msg.latest_fault,
                "planning_failures": int(msg.planning_failures),
                "execution_faults": int(msg.execution_faults),
                "execution_pauses": int(msg.execution_pauses),
                "recovery_time_ms": float(msg.recovery_time_ms),
            }
        )

    def summary(self):
        final_status = self.statuses[-1] if self.statuses else {}
        return {
            "raw_message_count": len(self.raw_receive_times),
            "filtered_message_count": len(self.filtered_processing_ms),
            "plan_message_count": len(self.plan_processing_ms),
            "perception_processing_ms": stats(self.filtered_processing_ms),
            "filtered_age_ms": stats(self.filtered_age_ms),
            "planning_processing_ms": stats(self.plan_processing_ms),
            "end_to_end_plan_age_ms": stats(self.plan_age_ms),
            "path_error_m": stats(self.path_errors),
            "max_sustained_input_hz": max_sustained_hz(self.raw_receive_times),
            "raw_interarrival_jitter_ms": interarrival_jitter_ms(self.raw_receive_times),
            "recovery_time_ms": max(
                [status["recovery_time_ms"] for status in self.statuses] or [0.0]
            ),
            "final_state": final_status.get("state", "UNKNOWN"),
            "final_fault": final_status.get("fault", "UNKNOWN"),
            "planning_failures": final_status.get("planning_failures", 0),
            "execution_faults": final_status.get("execution_faults", 0),
            "execution_pauses": final_status.get("execution_pauses", 0),
            "plan_faults": sorted(set(self.plan_faults)),
        }


def run_scenario(config, domain_id):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    command, process = start_launch(config, env)
    rclpy.init(args=None)
    node = rclpy.create_node(f"performance_collector_{domain_id}")
    collector = ScenarioCollector(node)
    deadline = time.monotonic() + config["duration"]
    try:
        while time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        stop_process(process)

    output, _ = process.communicate(timeout=5)
    return {
        "name": config["name"],
        "launch": config["launch"],
        "duration_sec": config["duration"],
        "parameters": parse_launch_args(config["args"]),
        "command": command,
        "summary": collector.summary(),
        "samples": {
            "perception_processing_ms": collector.filtered_processing_ms,
            "filtered_age_ms": collector.filtered_age_ms,
            "planning_processing_ms": collector.plan_processing_ms,
            "end_to_end_plan_age_ms": collector.plan_age_ms,
            "path_error_m": collector.path_errors,
            "statuses": collector.statuses,
        },
        "log": output.splitlines(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="docs/performance-latest.json")
    parser.add_argument("--domain-base", type=int, default=120)
    parser.add_argument("--build-type", default="RelWithDebInfo")
    args = parser.parse_args()

    results = {"metadata": metadata(args), "scenarios": []}
    for offset, config in enumerate(SCENARIOS):
        print(f"running {config['name']}...", flush=True)
        result = run_scenario(config, args.domain_base + offset)
        if result["summary"]["plan_message_count"] == 0:
            print(f"no plan samples collected for {config['name']}", file=sys.stderr)
            return 1
        results["scenarios"].append(result)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    print(json.dumps([{ "name": r["name"], **r["summary"] } for r in results["scenarios"]], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
