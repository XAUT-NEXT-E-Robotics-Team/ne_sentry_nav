"""
Launch ESDF map generation and visualization for global planning.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_grid_topic = LaunchConfiguration("input_grid_topic")
    esdf_topic = LaunchConfiguration("esdf_topic")
    esdf_marker_topic = LaunchConfiguration("esdf_marker_topic")
    use_visualizer = LaunchConfiguration("use_visualizer")
    obs_thresh = LaunchConfiguration("obs_thresh")
    treat_unknown_as_obstacle = LaunchConfiguration("treat_unknown_as_obstacle")
    danger_distance = LaunchConfiguration("danger_distance")
    warn_distance = LaunchConfiguration("warn_distance")
    marker_alpha = LaunchConfiguration("marker_alpha")
    marker_height_scale = LaunchConfiguration("marker_height_scale")
    rainbow_max_distance = LaunchConfiguration("rainbow_max_distance")
    use_planner = LaunchConfiguration("use_planner")
    jps_path_topic = LaunchConfiguration("jps_path_topic")
    optimized_path_topic = LaunchConfiguration("optimized_path_topic")
    path_marker_topic = LaunchConfiguration("path_marker_topic")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    safe_distance = LaunchConfiguration("safe_distance")
    optimize_trajectory = LaunchConfiguration("optimize_trajectory")
    traj_total_time = LaunchConfiguration("traj_total_time")
    traj_piece_len = LaunchConfiguration("traj_piece_len")
    traj_sample_dt = LaunchConfiguration("traj_sample_dt")
    path_z_offset = LaunchConfiguration("path_z_offset")
    marker_line_width = LaunchConfiguration("marker_line_width")
    max_expand = LaunchConfiguration("max_expand")

    esdf_builder_node = Node(
        package="ne_esdf_builder",
        executable="esdf_builder_node",
        name="esdf_builder_node",
        output="screen",
        parameters=[
            {
                "input_topic": input_grid_topic,
                "output_topic": esdf_topic,
                "obs_thresh": obs_thresh,
                "treat_unknown_as_obstacle": treat_unknown_as_obstacle,
            }
        ],
    )

    esdf_visualizer_node = Node(
        package="ne_esdf_builder",
        executable="esdf_visualizer_node",
        name="esdf_visualizer_node",
        output="screen",
        condition=IfCondition(use_visualizer),
        parameters=[
            {
                "input_topic": esdf_topic,
                "output_topic": esdf_marker_topic,
                "danger_distance": danger_distance,
                "warn_distance": warn_distance,
                "marker_alpha": marker_alpha,
                "marker_height_scale": marker_height_scale,
                "rainbow_max_distance": rainbow_max_distance,
            }
        ],
    )

    jps_planner_node = Node(
        package="ne_global_planner",
        executable="jps_planner_node",
        name="jps_planner_node",
        output="screen",
        condition=IfCondition(use_planner),
        parameters=[
            {
                "esdf_topic": esdf_topic,
                "jps_path_topic": jps_path_topic,
                "optimized_path_topic": optimized_path_topic,
                "path_marker_topic": path_marker_topic,
                "start_x": start_x,
                "start_y": start_y,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "safe_distance": safe_distance,
                "optimize_trajectory": optimize_trajectory,
                "traj_total_time": traj_total_time,
                "traj_piece_len": traj_piece_len,
                "traj_sample_dt": traj_sample_dt,
                "path_z_offset": path_z_offset,
                "marker_line_width": marker_line_width,
                "max_expand": max_expand,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_grid_topic",
                default_value="/terrain_analysis/grid",
                description="Input nav_msgs/OccupancyGrid topic used to build the ESDF map.",
            ),
            DeclareLaunchArgument(
                "esdf_topic",
                default_value="/esdf_map",
                description="Output nav_interfaces/Esdf topic.",
            ),
            DeclareLaunchArgument(
                "esdf_marker_topic",
                default_value="/esdf_markers",
                description="Output visualization_msgs/MarkerArray topic for RViz.",
            ),
            DeclareLaunchArgument(
                "use_visualizer",
                default_value="true",
                description="Start the ESDF RViz marker visualizer.",
            ),
            DeclareLaunchArgument(
                "obs_thresh",
                default_value="50",
                description="OccupancyGrid value greater than or equal to this is treated as occupied.",
            ),
            DeclareLaunchArgument(
                "treat_unknown_as_obstacle",
                default_value="true",
                description="Treat unknown OccupancyGrid cells as obstacles.",
            ),
            DeclareLaunchArgument(
                "danger_distance",
                default_value="0.2",
                description="Distance threshold used by the ESDF visualizer color scale.",
            ),
            DeclareLaunchArgument(
                "warn_distance",
                default_value="0.6",
                description="Warning distance used by the ESDF visualizer.",
            ),
            DeclareLaunchArgument(
                "marker_alpha",
                default_value="0.85",
                description="ESDF marker alpha.",
            ),
            DeclareLaunchArgument(
                "marker_height_scale",
                default_value="0.2",
                description="ESDF marker height scale relative to map resolution.",
            ),
            DeclareLaunchArgument(
                "rainbow_max_distance",
                default_value="0.6",
                description="Distance mapped to the end of the visualizer color range.",
            ),
            DeclareLaunchArgument(
                "use_planner",
                default_value="true",
                description="Start JPS planner and optimized trajectory publisher.",
            ),
            DeclareLaunchArgument(
                "jps_path_topic",
                default_value="/jps_path",
                description="Output nav_msgs/Path topic for raw JPS path.",
            ),
            DeclareLaunchArgument(
                "optimized_path_topic",
                default_value="/optimized_trajectory",
                description="Output nav_msgs/Path topic for optimized trajectory samples.",
            ),
            DeclareLaunchArgument(
                "path_marker_topic",
                default_value="/planner_path_markers",
                description="Output visualization_msgs/MarkerArray topic for planner path overlays.",
            ),
            DeclareLaunchArgument(
                "start_x",
                default_value="0.0",
                description="Planner start x position in the ESDF map frame.",
            ),
            DeclareLaunchArgument(
                "start_y",
                default_value="0.0",
                description="Planner start y position in the ESDF map frame.",
            ),
            DeclareLaunchArgument(
                "goal_x",
                default_value="5.0",
                description="Planner goal x position in the ESDF map frame.",
            ),
            DeclareLaunchArgument(
                "goal_y",
                default_value="0.0",
                description="Planner goal y position in the ESDF map frame.",
            ),
            DeclareLaunchArgument(
                "safe_distance",
                default_value="0.3",
                description="Safety distance used by JPS and trajectory optimization.",
            ),
            DeclareLaunchArgument(
                "optimize_trajectory",
                default_value="true",
                description="Run continuous trajectory optimization after JPS.",
            ),
            DeclareLaunchArgument(
                "traj_total_time",
                default_value="10.0",
                description="Initial total trajectory time for optimization.",
            ),
            DeclareLaunchArgument(
                "traj_piece_len",
                default_value="10.0",
                description="Target path length per spline segment.",
            ),
            DeclareLaunchArgument(
                "traj_sample_dt",
                default_value="0.1",
                description="Sampling interval for optimized trajectory Path output.",
            ),
            DeclareLaunchArgument(
                "path_z_offset",
                default_value="0.08",
                description="Z offset used to lift the Path and Marker overlays above the ESDF layer.",
            ),
            DeclareLaunchArgument(
                "marker_line_width",
                default_value="0.06",
                description="Line width used by the planner path LINE_STRIP markers.",
            ),
            DeclareLaunchArgument(
                "max_expand",
                default_value="-1",
                description="Maximum JPS expansions, or -1 for unlimited.",
            ),
            esdf_builder_node,
            esdf_visualizer_node,
            jps_planner_node,
        ]
    )
