#!/bin/bash

# Prune Docker containers
# yes | docker container prune
if [ "$(docker ps -a | grep ros2)" ]; then
    # Remove the container
    docker rm ros2
fi

# Function to run docker command with error handling
docker run --network host \
    --name ros2 \
    --user 1000:1000 \
    --group-add $(getent group dialout | cut -d: -f3) \
    --group-add $(getent group video | cut -d: -f3) \
    --group-add $(getent group alexander | cut -d: -f3) \
    --group-add=messagebus \
    --volume /home/alexander/simplebot2/ros2_workspace/:/home/alexander/simplebot2/ros2_workspace/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --privileged \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    --volume=/dev:/dev \
    --env="UDEV=1" \
    --env="BLINKA_MCP2221"=1 \
    -v /var/run/dbus:/var/run/dbus \
    -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
    -d \
    ros2:v0.2 bash -c "ros2 launch /home/alexander/simplebot2/ros2_workspace/launch/bot_launch.py; bash"
