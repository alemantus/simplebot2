#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/alexander/simplebot2/ros2_workspace/install/setup.bash
export PYTHONPATH=/home/alexander/venv/lib/python3.10/site-packages:$PYTHONPATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROARM_MODEL=roarm_m2
source /home/alexander/simplebot2/ros2_workspace/src/sllidar_ros2/scripts/create_udev_rules.sh
# Activate venv
source /home/alexander/venv/bin/activate
exec "$@"
