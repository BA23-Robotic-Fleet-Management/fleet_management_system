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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sdf_path = os.path.join(
        get_package_share_directory("ff_tb3_gz"),
        "model",
        "burger.sdf",
    )

    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "urdf",
        "turtlebot3_burger.urdf",
    )

    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    # Launch configuration variables specific to simulation
    robot_name = LaunchConfiguration("name", default="burger")
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    x_pos = LaunchConfiguration("x_pos", default="0.0")
    y_pos = LaunchConfiguration("y_pos", default="0.0")

    # Declare the launch arguments
    declare_robot_name_param = DeclareLaunchArgument(
        "robot_name", default_value="burger"
    )
    # Declare the launch arguments
    declare_use_sim_time_param = DeclareLaunchArgument(
        "use_sim_time", default_value="False"
    )
    # Declare the launch arguments
    declare_x_pos_param = DeclareLaunchArgument("x_pos", default_value="0.0")
    declare_y_pos_param = DeclareLaunchArgument("y_pos", default_value="0.0")

    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            robot_name,
            "-robot_namespace",
            robot_name,
            "-file",
            sdf_path,
            "-x",
            x_pos,
            "-y",
            y_pos,
            "-z",
            "0.01",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen"
    )
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
            }
        ],
        remappings=remappings
    )

    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_robot_name_param)
    ld.add_action(declare_use_sim_time_param)
    ld.add_action(declare_x_pos_param)
    ld.add_action(declare_y_pos_param)
    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
