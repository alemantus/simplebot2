#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  rplidar"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyUSB1"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp /home/alexander/simplebot2/ros2_workspace/src/sllidar_ros2/scripts/rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "