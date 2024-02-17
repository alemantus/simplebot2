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
    --device=/dev/input/event2 \
    --device=/dev/ttyACM0 \
    -it \
    ros2 bash 

#