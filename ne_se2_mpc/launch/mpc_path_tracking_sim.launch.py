from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    planner_launch = os.path.join(
        get_package_share_directory("ne_global_planner"),
        "launch",
        "esdf_map_planning.launch.py",
    )
    rog_map_rviz_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "ne_rog_map",
            "rviz",
            "default.rviz",
        )
    )

    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    optimized_path_topic = LaunchConfiguration("optimized_path_topic")
    grid_topic = LaunchConfiguration("grid_topic")
    map_frame = LaunchConfiguration("map_frame")
    safe_distance = LaunchConfiguration("safe_distance")
    use_rviz = LaunchConfiguration("use_rviz")

    rog_map_node = Node(
        package="rog_map",
        executable="rog_map_node",
        name="rog_map_node",
        output="screen",
        parameters=[
            {
                "static_tf.parent": "camera_init",
                "static_tf.child": map_frame,
                "static_tf.translation": [0.0, 0.0, 0.0],
                "static_tf.rotation": [1.0, 0.0, 0.0, 0.0],
            }
        ],
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            "input_grid_topic": grid_topic,
            "esdf_topic": "/esdf_map",
            "esdf_marker_topic": "/esdf_markers",
            "use_visualizer": "true",
            "use_planner": "true",
            "jps_path_topic": "/jps_path",
            "optimized_path_topic": optimized_path_topic,
            "path_marker_topic": "/planner_path_markers",
            "start_x": start_x,
            "start_y": start_y,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "safe_distance": safe_distance,
            "optimize_trajectory": "true",
            "plan_on_map_update": "true",
            "traj_total_time": "10.0",
            "traj_piece_len": "2.0",
            "traj_sample_dt": "0.05",
            "path_z_offset": "0.08",
            "marker_line_width": "0.06",
            "max_expand": "-1",
        }.items(),
    )

    sim_node = Node(
        package="ne_se2_mpc",
        executable="mpc_tracking_sim_node",
        name="mpc_tracking_sim_node",
        output="screen",
        parameters=[
            {
                "map_frame": map_frame,
                "base_frame": "mpc_base_link",
                "optimized_path_topic": optimized_path_topic,
                "odom_topic": "/mpc_sim/odom",
                "actual_path_topic": "/mpc_sim/actual_path",
                "reference_path_topic": "/mpc_sim/reference_path",
                "preview_path_topic": "/mpc_sim/preview_path",
                "marker_topic": "/mpc_sim/markers",
                "spline_speed": 1.2,
                "sim_dt": 0.05,
                "status_rate_hz": 5.0,
                "reference_sample_dt": 0.05,
                "reset_on_new_path": False,
                "duplicate_path_position_tol": 0.05,
                "stop_position_tolerance": 0.15,
                "stop_velocity_tolerance": 0.12,
                "mpc.dt": 0.05,
                "mpc.horizon": 40,
                "mpc.mass": 20.0,
                "mpc.iz": 5.0,
                "mpc.gravity": 9.81,
                "mpc.max_preview_time": 2.0,
                "mpc.projection_sample_dt": 0.02,
                "mpc.q_diag": [50.0, 50.0, 20.0, 5.0, 5.0, 2.0],
                "mpc.r_diag": [0.2, 0.2, 0.1],
                "mpc.u_min": [-30.0, -30.0, -10.0],
                "mpc.u_max": [30.0, 30.0, 10.0],
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rog_map_rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_frame", default_value="world"),
            DeclareLaunchArgument("grid_topic", default_value="/terrain_analysis/grid"),
            DeclareLaunchArgument("optimized_path_topic", default_value="/optimized_trajectory"),
            DeclareLaunchArgument("safe_distance", default_value="0.45"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_x", default_value="0.0"),
            DeclareLaunchArgument("start_y", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            rog_map_node,
            planner,
            sim_node,
            rviz_node,
        ]
    )
