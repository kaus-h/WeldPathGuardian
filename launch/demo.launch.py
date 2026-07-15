from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    rate_hz = LaunchConfiguration("rate_hz")
    point_count = LaunchConfiguration("point_count")
    noise_stddev = LaunchConfiguration("noise_stddev")
    dropout_ratio = LaunchConfiguration("dropout_ratio")
    max_gap = LaunchConfiguration("max_gap")
    max_curvature = LaunchConfiguration("max_curvature")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="gaussian_noise"),
            DeclareLaunchArgument("rate_hz", default_value="20.0"),
            DeclareLaunchArgument("point_count", default_value="64"),
            DeclareLaunchArgument("noise_stddev", default_value="0.004"),
            DeclareLaunchArgument("dropout_ratio", default_value="0.06"),
            DeclareLaunchArgument("max_gap", default_value="0.18"),
            DeclareLaunchArgument("max_curvature", default_value="45.0"),
            Node(
                package="seam_sensor",
                executable="seam_sensor_node",
                name="seam_sensor",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "rate_hz": rate_hz,
                        "point_count": point_count,
                        "noise_stddev": noise_stddev,
                        "dropout_ratio": dropout_ratio,
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
                        "max_neighbor_distance": 0.2,
                    }
                ],
            ),
            Node(
                package="weld_planner",
                executable="weld_planner_node",
                name="weld_planner",
                output="screen",
                parameters=[
                    {
                        "waypoint_spacing": 0.05,
                        "max_gap": max_gap,
                        "max_curvature": max_curvature,
                        "tool_speed": 0.04,
                    }
                ],
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
