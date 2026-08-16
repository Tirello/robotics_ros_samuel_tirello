#!/bin/bash
CONFIGFILE=docker/config/tools.sh
source $CONFIGFILE

xhost +local:docker

# Update to not use -v for devices
docker run -it  --rm \
    -e QT_X11_NO_MITSHM=1 \
    --network=host \
    --ipc=host \
    -v /dev:/dev \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -v $PWD/ros_ws:/home/$USERNAME/ros_ws \
    --privileged \
    --name $CONTAINER_NAME \
    $IMAGE_NAME:$IMAGE_TAG
