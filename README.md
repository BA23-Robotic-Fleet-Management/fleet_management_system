# Installation

```bash
git clone --recurse-submodules https://github.com/BA23-Robotic-Fleet-Management/fleet_manager.git
cd fleet_manager

source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

# Start the Free Fleet Turtlebot3 Gazebo Simulation

Start the Robot and the Gazebo simulation:
```bash
cd fleet_manager
source install/setup.bash

ros2 launch ff_tb3_gz icclab.launch.xml
```

Start the Free Fleet manager:
```bash
cd fleet_manager
source install/setup.bash

ros2 launch TODO
```
