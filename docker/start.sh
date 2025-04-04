#!/bin/bash

xhost +local:docker
docker run -it --rm \
    --privileged \
    --network host \
    --user $(id -u):$(id -g) \
    --group-add dialout \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.ssh:/home/devuser/.ssh \
    -v $HOME/DIYRobotChallengeSim:/home/devuser/ros2_ws \
    -v /dev:/dev \
    diy-robot-challenge-sim:latest
xhost -local:docker