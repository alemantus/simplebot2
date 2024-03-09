#!/bin/bash

# Prune Docker containers
yes | docker container prune

# Function to run docker command with error handling
docker run --network host \
    --name ros2 \
    --user 1001:alexander \
    --group-add=dialout \
    --group-add=messagebus \
    --volume /home/alexander/simplebot2/ros2_workspace/:/home/alexander/simplebot2/ros2_workspace/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    --privileged \
    --env="UDEV=1" \
    --volume=/dev:/dev \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    -d --restart unless-stopped \
    -it \
    ros2 bash -c "ros2 launch /home/alexander/simplebot2/ros2_workspace/launch/bot_launch.py; bash"
