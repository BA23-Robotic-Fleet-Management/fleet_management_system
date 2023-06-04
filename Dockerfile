FROM ghcr.io/ba23-robotic-fleet-management/fleet-manager-ci-base:main AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=1

WORKDIR /usr/src/app
COPY . .
# Build packages
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /rmf_demos_ws/install/setup.bash && colcon build"

FROM base AS adapter
# Launch adapter
RUN echo "source /rmf_demos_ws/install/setup.bash" >> /docker-entrypoint.sh
RUN echo "exec ros2 launch free_fleet_rmf_adapter adapter.launch.xml" >> /docker-entrypoint.sh

FROM base AS free-fleet-server
# Launch server
RUN echo "exec ros2 launch ff_tb3_gz ff_server.launch.xml" >> /docker-entrypoint.sh

FROM base AS free-fleet-client
# Set robot name with via ENV
ENV ROBOT_NAME=robot1
RUN echo "envsubst < src/ff_tb3_gz/launch/generic_robot_launch.xml.template > install/ff_tb3_gz/share/ff_tb3_gz/launch/generic_robot.launch.xml" >> /docker-entrypoint.sh
# Launch client
RUN echo "exec ros2 launch ff_tb3_gz generic_robot.launch.xml" >> /docker-entrypoint.sh
