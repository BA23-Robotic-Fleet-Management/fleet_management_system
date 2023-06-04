# ROS2 Fleet Manager

This repository contains an experimental fleet manager built on top of:

- [ROS2](https://docs.ros.org/en/humble/index.html)
- [Open-RMF](https://www.open-rmf.org/)
- [Free Fleet](https://github.com/open-rmf/free_fleet)

## Setup

To be able to test the fleet manager the following steps are required:

1. Install ROS2 humble (See https://docs.ros.org/en/humble/Installation.html)
2. Install Open-RMF with the `easy_fleet_adapter` feature

At the time of writing, the `easy_fleet_adapter` feature,
which the fleet manager depends on, still lives in its own
branch and has not been merged into the main branch. For this reason, the
[rmf repository](https://github.com/open-rmf/rmf), which is a repository that contains
all the Open-RMF components, has been forked and patched to use the `easy_fleet_adapter` feature.
Clone the forked repository and follow the instructions in the
[README](https://github.com/BA23-Robotic-Fleet-Management/rmf#installation-instructions).

3. Clone this repository and build the fleet manager
```bash
git clone --recurse-submodules https://github.com/BA23-Robotic-Fleet-Management/fleet_management_system.git
cd fleet_management_system
source /opt/ros/humble/setup.bash
source ~/rmf_ws/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## Start Gazebo Simulation

The next steps start all components that are required to test the fleet manager
in a simulation. [Gazebo](https://gazebosim.org/) is used as the simulation software.

Start the simulation with the free fleet client and the whole nav2 stack:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch ff_tb3_gz icclab.launch.xml
```

Start the free fleet server:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch ff_tb3_gz ff_server.launch.xml
```

Start adapter that connects both Open-RMF and Free Fleet:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch free_fleet_rmf_adapter adapter.launch.xml
```

### Test fleet manager from the command line

Give a dispatch order:

```bash
cd fleet_management_system
source install/setup.bash
ros2 run rmf_demos_tasks dispatch_patrol --places middle_top2 --use_sim_time
```
