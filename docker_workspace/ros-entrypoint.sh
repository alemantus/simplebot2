#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/alexander/simplebot2/ros2_workspace/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/alexander/simplebot2/ros2_workspace/src/sllidar_ros2/scripts/create_udev_rules.sh
exec "$@"
