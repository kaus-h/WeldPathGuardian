from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    noise_stddev = LaunchConfiguration("noise_stddev")
    dropout_ratio = LaunchConfiguration("dropout_ratio")
    max_gap = LaunchConfiguration("max_gap")
    max_curvature_rad_per_meter = LaunchConfiguration("max_curvature_rad_per_meter")
    seed = LaunchConfiguration("seed")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="missing_segment"),
            DeclareLaunchArgument("noise_stddev", default_value="0.006"),
            DeclareLaunchArgument("dropout_ratio", default_value="0.2"),
            DeclareLaunchArgument("max_gap", default_value="0.12"),
            DeclareLaunchArgument("max_curvature_rad_per_meter", default_value="45.0"),
            DeclareLaunchArgument("seed", default_value="42"),
            Node(
                package="seam_sensor",
                executable="seam_sensor_node",
                name="seam_sensor",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "rate_hz": 12.0,
                        "point_count": 64,
                        "noise_stddev": noise_stddev,
                        "dropout_ratio": dropout_ratio,
                        "stale_offset_ms": 900,
                        "seed": seed,
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
                        "max_neighbor_distance": 0.12,
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
                        "max_curvature_rad_per_meter": max_curvature_rad_per_meter,
                        "tool_speed": 0.04,
                    }
                ],
            ),
            Node(
                package="weld_executor",
                executable="weld_executor_node",
                name="weld_executor",
                output="screen",
                parameters=[{"auto_execute": True, "waypoint_delay_ms": 50}],
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
