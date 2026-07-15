import os
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from weld_interfaces.action import ExecuteWeld
from weld_interfaces.msg import WeldPlan, WeldWaypoint


def make_plan(count=18):
    plan = WeldPlan()
    plan.header.frame_id = "weld_cell"
    plan.valid = True
    plan.fault_code = "None"
    plan.path_length = 0.9
    for index in range(count):
        waypoint = WeldWaypoint()
        waypoint.pose = Pose()
        waypoint.pose.position.x = 0.05 * index
        waypoint.pose.orientation.w = 1.0
        waypoint.speed = 0.04
        plan.waypoints.append(waypoint)
    return plan


def spin_until(node, future, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done()
    return future.result()


def start_executor(domain_id):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "weld_executor",
            "weld_executor_node",
            "--ros-args",
            "-p",
            "auto_execute:=false",
            "-p",
            "waypoint_delay_ms:=200",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return env, process


def test_concurrent_action_goal_is_rejected_and_active_goal_cancels():
    env, process = start_executor(71)
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    rclpy.init()
    node = rclpy.create_node("executor_action_concurrency_test")
    client = ActionClient(node, ExecuteWeld, "execute_weld")
    try:
        assert client.wait_for_server(timeout_sec=8.0)

        first_goal = spin_until(node, client.send_goal_async(ExecuteWeld.Goal(plan=make_plan())))
        assert first_goal.accepted
        time.sleep(0.5)

        second_goal = spin_until(node, client.send_goal_async(ExecuteWeld.Goal(plan=make_plan())))
        assert not second_goal.accepted

        cancel_response = spin_until(node, first_goal.cancel_goal_async())
        assert len(cancel_response.goals_canceling) == 1
        result = spin_until(node, first_goal.get_result_async(), timeout_sec=8.0)
        assert result.result.success is False
        assert result.result.fault_code == "Cancelled"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


def test_active_action_aborts_when_plan_fault_arrives():
    env, process = start_executor(72)
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    rclpy.init()
    node = rclpy.create_node("executor_action_fault_test")
    client = ActionClient(node, ExecuteWeld, "execute_weld")
    fault_pub = node.create_publisher(WeldPlan, "/weld/plan", 10)
    try:
        assert client.wait_for_server(timeout_sec=8.0)

        goal = spin_until(node, client.send_goal_async(ExecuteWeld.Goal(plan=make_plan(count=30))))
        assert goal.accepted
        time.sleep(0.5)

        fault = WeldPlan()
        fault.header.frame_id = "weld_cell"
        fault.valid = False
        fault.fault_code = "ExcessiveGap"
        for _ in range(5):
            fault_pub.publish(fault)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        result = spin_until(node, goal.get_result_async(), timeout_sec=8.0)
        assert result.result.success is False
        assert result.result.fault_code == "ExcessiveGap"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


def test_completed_action_allows_next_action_through_terminal_reset():
    env, process = start_executor(73)
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]
    rclpy.init()
    node = rclpy.create_node("executor_action_terminal_reset_test")
    client = ActionClient(node, ExecuteWeld, "execute_weld")
    try:
        assert client.wait_for_server(timeout_sec=8.0)

        first_goal = spin_until(node, client.send_goal_async(ExecuteWeld.Goal(plan=make_plan(count=2))))
        assert first_goal.accepted
        first_result = spin_until(node, first_goal.get_result_async(), timeout_sec=8.0)
        assert first_result.result.success is True
        assert first_result.result.fault_code == "None"

        second_goal = spin_until(node, client.send_goal_async(ExecuteWeld.Goal(plan=make_plan(count=2))))
        assert second_goal.accepted
        second_result = spin_until(node, second_goal.get_result_async(), timeout_sec=8.0)
        assert second_result.result.success is True
        assert second_result.result.fault_code == "None"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
