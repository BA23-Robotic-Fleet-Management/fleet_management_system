# Installation
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import src < rmf.repos
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
# ERROR: the following rosdeps failed to install
#   apt: command [sudo -H apt-get install -y ros-humble-gazebo-plugins] failed

cd src/rmf/rmf_ros2/
git checkout yadu/easy_full_control

sudo apt install clang clang-tools lldb lld libstdc++-12-dev python3-colcon-mixin libccd-dev
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

cd ~/rmf_ws
export CXX=clang++
export CC=clang
colcon build --mixin release lld

cd ~
git clone --recurse-submodules https://github.com/BA23-Robotic-Fleet-Management/fleet_manager.git
cd ~/fleet_manager
source ~/rmf_ws/install/setup.bash

# for the hardware
colcon build --parallel-workers 4 --symlink-install --packages-select free_fleet
colcon build --parallel-workers 4 --symlink-install --packages-select free_fleet_client_ros2

# for simluation
colcon build --parallel-workers 4 --symlink-install
```

# start on hardware
on the robot:
```bash
# reduce load
sudo systemctl disable --now snapd

ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch nav2_bringup bringup_launch.py map:=icclab_latest_map.yaml use_composition:=False
```

on the control node in the cloud:
```bash
# to test the basic connectivity, and move the robot without free_fleet / rmf, just using nav2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz

# free_fleet & rmf
TODO
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

Dispatch via RMF:
```bash
cd fleet_manager
source install/setup.bash

ros2 run rmf_demos_tasks dispatch_patrol --places middle_top2 --use_sim_time
```
