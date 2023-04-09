# Installation

```bash
git clone --recurse-submodules https://github.com/BA23-Robotic-Fleet-Management/fleet_manager.git
cd fleet_manager

source /opt/ros/humble/setup.bash
# we need rmf_building_map_tools. TODO: document how to install rmf for our use case
source ~/rmf_ws/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

# Start the Free Fleet Turtlebot3 Gazebo Simulation

Start the Robot and the Gazebo simulation:
```bash
cd fleet_manager
source install/setup.bash

ros2 launch ff_tb3_gz icclab.launch.xml
```

Start the Free Fleet server:
```bash
cd fleet_manager
source install/setup.bash

ros2 launch ff_tb3_gz ff_server.launch.xml
```

Start the RMF Adapter for the Free Fleet server:
```bash
cd fleet_manager
source install/setup.bash

ros2 launch free_fleet_rmf_adapter adapter.launch.xml
```
