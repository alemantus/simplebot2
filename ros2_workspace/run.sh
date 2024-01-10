docker container prune

docker run --network host \
    --name ros2 \
    --user 1000:1000 \
    --volume /home/alexander/Documents/simplebot2/ros2_workspace/:/home/alexander/Documents/simplebot2/ros2_workspace/ \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/etc/localtime:/etc/localtime:ro" \
    --volume="/etc/timezone:/etc/timezone:ro" \
    -it \
    ros2 bash


