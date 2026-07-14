from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="gaussian_noise"),
            Node(
                package="seam_sensor",
                executable="seam_sensor_node",
                name="seam_sensor",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "rate_hz": 20.0,
                        "point_count": 64,
                        "noise_stddev": 0.004,
                        "dropout_ratio": 0.06,
                    }
                ],
            ),
            Node(
                package="seam_perception",
                executable="seam_perception_node",
                name="seam_perception",
                output="screen",
                parameters=[
                    {
                        "min_confidence": 0.55,
                        "stale_threshold_ms": 350,
                        "smoothing_window": 1,
                        "min_points": 4,
                    }
                ],
            ),
            Node(
                package="weld_planner",
                executable="weld_planner_node",
                name="weld_planner",
                output="screen",
                parameters=[{"waypoint_spacing": 0.05, "max_gap": 0.18, "tool_speed": 0.04}],
            ),
            Node(
                package="weld_executor",
                executable="weld_executor_node",
                name="weld_executor",
                output="screen",
                parameters=[{"auto_execute": True, "waypoint_delay_ms": 35}],
            ),
            Node(
                package="system_monitor",
                executable="system_monitor_node",
                name="system_monitor",
                output="screen",
            ),
            Node(
                package="weld_visualization",
                executable="weld_visualization_node",
                name="weld_visualization",
                output="screen",
            ),
        ]
    )

