#!/bin/bash

source /opt/ros/humble/install/setup.bash
source /opt/ros/humble/setup.bash
source /home/alexander/simplebot2/ros2_workspace/install/setup.bash
source /home/alexander/simplebot2/ros2_workspace/src/sllidar_ros2/scripts/create_udev_rules.sh
exec "$@"
