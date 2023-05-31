#!/bin/bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///PATH_TO_CYCLONE_DDS_XML

echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION**"
