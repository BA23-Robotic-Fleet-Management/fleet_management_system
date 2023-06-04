# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    map_file = LaunchConfiguration("map", default="default_ns")
    namespace = LaunchConfiguration("namespace", default="default_ns")
    params_file = LaunchConfiguration("params_file", default="default_ns")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=map_file,
        description="Full path to map file to load",
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=namespace,
        description="Full path to map file to load",
    )

    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Full path to param file to load",
    )

    declare_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    ld = LaunchDescription()

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            Node(
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": "True"}],
                arguments=["--ros-args", "--log-level", "info"],
                remappings=remappings,
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("rmf_ff_tb3"), "launch", "localization_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "True",
                    "params_file": params_file,
                    "use_respawn": "False",
                    "container_name": "nav2_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": "True",
                    "params_file": params_file,
                    "use_respawn": "False",
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    ld.add_action(declare_map_arg)
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_sim_time_arg)
    ld.add_action(bringup_cmd_group)

    return ld
