# ROS2 Fleet Management System

This repository contains an experimental fleet management system built on top of:

- [ROS2](https://docs.ros.org/en/humble/index.html)
- [Open-RMF](https://www.open-rmf.org/)
- [Free Fleet](https://github.com/open-rmf/free_fleet)

## Setup

To be able to test the fleet management system the following steps are required:

1. Installing ROS2 humble (See the [official documentation](https://docs.ros.org/en/humble/Installation.html)).
2. Installing Open-RMF with the `easy_fleet_adapter` feature:

    * At the time of writing, the `easy_fleet_adapter` feature, still lives in its own branch and has not yet been merged into the main branch.
      For this reason, the [rmf repository](https://github.com/open-rmf/rmf), which is a repository containing all the Open-RMF components, has been forked and patched to use the `easy_fleet_adapter` feature.
      Clone the forked repository and follow the [official instructions for building from source](https://github.com/open-rmf/rmf#building-from-sources-recommended).

3. Cloning this repository and building the fleet management system:
```bash
git clone --recurse-submodules https://github.com/BA23-Robotic-Fleet-Management/fleet_management_system.git
cd fleet_management_system
source /opt/ros/humble/setup.bash
source ~/rmf_ws/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## Starting the Gazebo Simulation

The next steps launch all components that are required to test the fleet management system in a [Gazebo](https://gazebosim.org/) simulation.

Starting the simulation with the Free Fleet client and the entire Nav2 stack:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch rmf_ff_tb3 icclab_single_robot_gz.launch.xml
```

Starting the Free Fleet server:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch rmf_ff_tb3 ff_server.launch.xml
```

Starting the fleet adapter that connects both Open-RMF and Free Fleet:

```bash
cd fleet_management_system
source install/setup.bash
ros2 launch free_fleet_rmf_adapter adapter.launch.xml
```

### Testing the Fleet Management System from the Command Line

Sending a dispatch order to Open-RMF:

```bash
cd fleet_management_system
source install/setup.bash
ros2 run rmf_demos_tasks dispatch_patrol --places middle_top2 --use_sim_time
```
