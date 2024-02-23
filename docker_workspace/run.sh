#!/bin/bash

# Prune Docker containers
docker container prune

# Function to run docker command with error handling
run_docker_command() {
    # Run the Docker command and catch errors
    "$@" || return 1
}

# Try the first Docker command
run_docker_command docker run --network host \
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
    --device=/dev/input/event2 \
    --device=/dev/ttyACM0 \
    -it \
    ros2 bash 

# If the first Docker command fails, run the second Docker command
if [ $? -ne 0 ]; then
    docker container prune
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
        -it \
        ros2 bash 
fi
