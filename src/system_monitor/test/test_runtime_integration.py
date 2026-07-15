import os
import re
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def run_command(args, env, timeout=12):
    return subprocess.run(args, env=env, timeout=timeout, check=True, text=True, capture_output=True)


def wait_for_topic(topic_line, env, timeout=12):
    deadline = time.monotonic() + timeout
    latest_topics = ""
    while time.monotonic() < deadline:
        latest_topics = run_command(["ros2", "topic", "list", "-t"], env).stdout
        if topic_line in latest_topics:
            return latest_topics
        time.sleep(0.25)
    raise AssertionError(f"{topic_line} not found in topics:\n{latest_topics}")


def launch_demo(env, *extra_args):
    return subprocess.Popen(
        ["ros2", "launch", str(ROOT / "launch" / "demo.launch.py"), *extra_args],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def launch_fault(env, *extra_args):
    return subprocess.Popen(
        ["ros2", "launch", str(ROOT / "launch" / "fault_demo.launch.py"), *extra_args],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_process(process):
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()


def test_clean_demo_produces_executable_plan():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "81"
    process = launch_demo(env)
    try:
        wait_for_topic("/seam/raw [weld_interfaces/msg/SeamObservation]", env)

        plan = run_command(
            ["ros2", "topic", "echo", "--once", "/weld/plan", "weld_interfaces/msg/WeldPlan"],
            env,
        ).stdout
        assert "valid: true" in plan
        assert "fault_code: None" in plan
        assert re.search(r"path_length: (?!0\.0)", plan)
    finally:
        stop_process(process)


def test_missing_segment_faults_with_excessive_gap():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "82"
    process = launch_fault(env, "scenario:=missing_segment")
    try:
        time.sleep(5)
        plan = run_command(
            ["ros2", "topic", "echo", "--once", "/weld/plan", "weld_interfaces/msg/WeldPlan"],
            env,
        ).stdout
        assert "waypoints: []" in plan
        assert "valid: false" in plan
        assert "fault_code: ExcessiveGap" in plan

        status = run_command(
            ["ros2", "topic", "echo", "--once", "/weld/status", "weld_interfaces/msg/SystemStatus"],
            env,
        ).stdout
        assert "state: FAULTED" in status
        assert "latest_fault: ExcessiveGap" in status
    finally:
        stop_process(process)


def test_low_confidence_pauses_and_recovers():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "83"
    process = launch_demo(env, "scenario:=low_confidence_recovery")
    try:
        time.sleep(2.5)
        paused = run_command(
            ["ros2", "topic", "echo", "--once", "/weld/status", "weld_interfaces/msg/SystemStatus"],
            env,
        ).stdout
        assert "state: PAUSED" in paused
        assert "latest_fault: LowConfidence" in paused

        time.sleep(4.5)
        recovered = run_command(
            ["ros2", "topic", "echo", "--once", "/weld/status", "weld_interfaces/msg/SystemStatus"],
            env,
        ).stdout
        assert "latest_fault: None" in recovered
        assert re.search(r"execution_pauses: [1-9]", recovered)
        assert re.search(r"recovery_time_ms: (?!0\.0)", recovered)
    finally:
        stop_process(process)
