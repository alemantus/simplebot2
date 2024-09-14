#!/bin/bash

# Prune Docker containers
yes | docker container prune

# Function to run docker command with error handling
docker run --network host \
    --name ros2 \
    --user 1001:alexander \
    --group-add $(getent group dialout | cut -d: -f3) \
    --group-add=messagebus \
    --group-add $(getent group i2c | cut -d: -f3) \
    --group-add $(getent group gpio | cut -d: -f3) \
    --volume /home/alexander/simplebot2/ros2_workspace/:/home/alexander/simplebot2/ros2_workspace/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    --env="UDEV=1" \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    --restart unless-stopped \
    -it \
    ros2 bash
