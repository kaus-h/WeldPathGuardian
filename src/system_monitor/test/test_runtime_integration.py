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


def wait_for_topic_message(topic, msg_type, env, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    latest_message = ""
    while time.monotonic() < deadline:
        try:
            latest_message = run_command(
                ["ros2", "topic", "echo", "--once", topic, msg_type], env, timeout=3
            ).stdout
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            time.sleep(0.25)
            continue
        if predicate(latest_message):
            return latest_message
        time.sleep(0.25)
    raise AssertionError(f"expected message not observed on {topic}:\n{latest_message}")


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
        plan = wait_for_topic_message(
            "/weld/plan",
            "weld_interfaces/msg/WeldPlan",
            env,
            lambda message: "fault_code: ExcessiveGap" in message,
        )
        assert "waypoints: []" in plan
        assert "valid: false" in plan
        assert "fault_code: ExcessiveGap" in plan

        status = wait_for_topic_message(
            "/weld/status",
            "weld_interfaces/msg/SystemStatus",
            env,
            lambda message: "state: FAULTED" in message and "latest_fault: ExcessiveGap" in message,
        )
        assert "state: FAULTED" in status
        assert "latest_fault: ExcessiveGap" in status
    finally:
        stop_process(process)


def test_low_confidence_pauses_and_recovers():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "83"
    process = launch_demo(env, "scenario:=low_confidence_recovery")
    try:
        paused = wait_for_topic_message(
            "/weld/status",
            "weld_interfaces/msg/SystemStatus",
            env,
            lambda message: "state: PAUSED" in message and "latest_fault: LowConfidence" in message,
        )
        assert "state: PAUSED" in paused
        assert "latest_fault: LowConfidence" in paused

        recovered = wait_for_topic_message(
            "/weld/status",
            "weld_interfaces/msg/SystemStatus",
            env,
            lambda message: (
                "latest_fault: None" in message
                and re.search(r"execution_pauses: [1-9]", message)
                and re.search(r"recovery_time_ms: (?!0\.0)", message)
            ),
            timeout=14,
        )
        assert "latest_fault: None" in recovered
        assert re.search(r"execution_pauses: [1-9]", recovered)
        assert re.search(r"recovery_time_ms: (?!0\.0)", recovered)
    finally:
        stop_process(process)
