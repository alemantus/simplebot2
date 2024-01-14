docker run -it \
        -v "/dev:/dev" \
        --privileged \
        --name joy \
        --net=host \
        naomiz/ds4_driver:foxy \
        bash
