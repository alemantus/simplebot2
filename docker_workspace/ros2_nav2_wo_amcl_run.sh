#!/bin/bash

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
    ros2:v0.2 bash -c "RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG ros2 launch simplebot2 main_nav2_wo_amcl_launch.py" 