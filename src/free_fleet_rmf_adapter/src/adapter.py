# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import sys
import argparse
import yaml
import time
import rclpy
import rclpy.node
from rclpy.parameter import Parameter
from rmf_fleet_msgs.msg import FleetState, RobotState, Location, PathRequest
import rmf_adapter as adpt
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSReliabilityPolicy as Reliability

from .configuration import get_configuration

from functools import partial

# This should be the fleet manager name
ADAPTER_NAME = "free_fleet_rmf_adapter"


class Robot:
    def __init__(self, state: RobotState = None):
        self.state = state
        self.last_path_request = None
        self.last_completed_request = None

    def is_expected_task_id(self, task_id) -> bool:
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True

    def to_easy_control_robot_state(self) -> RobotState:
        return adpt.easy_full_control.RobotState(
            self.state.name,
            # FIXME: From where can we get this information
            # it is not published in the fleet_state / free_fleet_states
            "test_node",
            # FIXME: This is probably wrong
            'L1',
            [self.state.location.x, self.state.location.y, self.state.location.yaw],
            self.state.battery_percent,
        )


class RMFAdapter(rclpy.node.Node):
    def __init__(self, config, nav_graph_path, use_sim_time):
        super().__init__(ADAPTER_NAME)
        # Task ID counter
        self.next_id = 0
        # Dict that contains the current task ID that is being processed
        self.cmd_ids = {}
        # Fleet name
        self.fleet_name = "gaga"
        # RMF control object
        self.easy_full_control = None
        self.init_rmf_adapter(config, nav_graph_path, use_sim_time)
        # Initialize robot states and fleet name
        self.create_subscription(
            FleetState,
            'free_fleet_states',
            self.update_robot_state,
            QoSProfile(
                reliability=Reliability.RELIABLE,
                history=History.KEEP_LAST,
                depth=10,
            )
        )
        self.robots = {}
        # Publisher that is used to publish path requests
        self.path_publisher = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default
        )

    def update_robot_state(self, rmf_message: FleetState):
        """Add or update a robot state

        Args:
            rmf_message (FleetState): Message that is sent via ROS2
        """

        def __goal_completed(robot_name: str) -> bool:
            return self.get_robot_state(robot_name).last_request_completed()

        def __get_robot_state(robot_name: str) -> RobotState:
            robot = self.robots.get(robot_name)
            if robot is None:
                return None

            return robot.to_easy_control_robot_state()

        def __navigate(robot_name: str, map_name: str, goal: Location, update_handle) -> partial:
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id

            robot = self.self.robots.get(robot_name)
            if robot is None:
                return None

            target_x = goal["x"]
            target_y = goal["y"]
            target_yaw = goal["yaw"]

            time_ = self.get_clock().now().to_msg()

            path_request = PathRequest()
            # Add current location to the path request a the first location
            path_request.path.append(robot.state.location)

            disp = self.disp(
                [target_x, target_y], [robot.state.location.x, robot.state.location.y]
            )
            duration = int(disp / self.vehicle_traits.linear.nominal_velocity) + int(
                abs(abs(robot.state.location.yaw) - abs(target_yaw))
                / self.vehicle_traits.rotational.nominal_velocity
            )
            time_.sec = time_.sec + duration
            target_loc = Location()
            target_loc.t = time_
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = map_name

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            self.path_publisher.publish(path_request)

            robot.last_path_request = path_request

            self.get_logger().info(f"Navigating robot {robot_name}")
            return partial(__goal_completed, robot_name)

        def __stop(robot_name: str) -> bool:
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id

            robot = self.self.robots.get(robot_name)
            if robot is None:
                return None

            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)
            path_request.task_id = str(cmd_id)

            self.path_publisher.publish(path_request)

            self.get_logger().info(
                f"Sending stop request for {robot_name}: {cmd_id}"
            )
            robot.last_path_request = path_request

            return True

        def __dock(robot_name: str, dock_name: str, update_handle):
            pass

        def __action_executor(
            robot_name: str,
            category: str,
            description: dict,
            execution: adpt.robot_update_handle.ActionExecution,
        ):
            pass

        if self.fleet_name != rmf_message.name:
            self.get_logger().info(f"Found fleet {rmf_message.name}")
            self.fleet_name = rmf_message.name

        for robot in rmf_message.robots:
            if robot.name not in self.robots:
                self.get_logger().info(f"Found robot {robot.name}")
                self.robots[robot.name] = Robot(robot)
                # Add robot to fleet
                # FIXME:
                # [adapter-2] [ERROR] [1680470020.884664428] [easy_fleet_adapter]:
                # Unable to compute a StartSet for robot [ros2_tb3_0]
                # using level_name [turtlebot_world] and location
                # [-1.895, -0.420, 0.000] specified in the RobotState param.
                # This can happen if the level_name in RobotState does not
                # match any of the map names in the navigation graph supplied
                # or if the location reported in the RobotState is far way from
                # the navigation graph. This robot will not be added to the
                #  fleet.

                self.easy_full_control.add_robot(
                    self.robots[robot.name].to_easy_control_robot_state(),
                    partial(__get_robot_state, robot.name),
                    partial(__navigate, robot.name),
                    partial(__stop, robot.name),
                    partial(__dock, robot.name),
                    partial(__action_executor, robot.name),
                )

    def init_rmf_adapter(self, config, nav_graph_path, use_sim_time):
        configuration = get_configuration(self.fleet_name, config, nav_graph_path, self)
        # Make the easy full control
        self.easy_full_control = adpt.EasyFullControl.make(configuration)

        if use_sim_time:
            self.easy_full_control.node.use_sim_time()


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(prog="free_fleet_rmf_adapter")
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    parser.add_argument(
        "-n",
        "--nav_graph",
        type=str,
        required=True,
        help="Path to the nav_graph for this fleet adapter",
    )
    parser.add_argument(
        "-sim",
        "--use_sim_time",
        action="store_true",
        help="Use sim time, default: false",
    )
    args = parser.parse_args(args_without_ros[1:])

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r", encoding="utf-8") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    node = rclpy.node.Node(ADAPTER_NAME)
    node.get_logger().info("Starting free fleet rmf adapter")

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = RMFAdapter(
        config_yaml,
        nav_graph_path,
        args.use_sim_time
    )

    rclpy.spin(adapter)

    # Shutdown
    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
