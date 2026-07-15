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


def test_concurrent_action_goal_is_rejected_and_active_goal_cancels():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "71"
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
