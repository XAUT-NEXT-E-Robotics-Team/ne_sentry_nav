"""
Author: ei_code_bash && 3080152159@qq.com
Date: 2026-04-16 15:52:32
LastEditors: ei_code_bash && 3080152159@qq.com
LastEditTime: 2026-04-16 15:54:48
FilePath: /ne_nav/src/ne_esdf_builder/launch/esdf_visual.launch.py
Description: 我永远喜欢雪之下雪乃

Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    esdf_builder_node = Node(
        package="ne_esdf_builder",
        executable="esdf_builder_node",
        name="esdf_builder_node",
        output="screen",
    )

    esdf_visualizer_node = Node(
        package="ne_esdf_builder",
        executable="esdf_visualizer_node",
        name="esdf_visualizer_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/esdf_map",
                "output_topic": "/esdf_markers",
                "danger_distance": 0.2,
                "warn_distance": 0.6,
                "marker_alpha": 0.85,
                "marker_height_scale": 0.2,
                "rainbow_max_distance": 0.6,
            }
        ],
    )

    return LaunchDescription([esdf_builder_node, esdf_visualizer_node])
