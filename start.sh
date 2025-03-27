#!/bin/bash
#coding=utf-8
#这个是使用我自己的docker pm_v2x01
echo 9 | sudo -S sysctl -w net.core.rmem_max=2147483647
echo 9 | sudo -S sysctl -w net.ipv4.ipfrag_time=3
echo 9 | sudo -S sysctl -w net.ipv4.ipfrag_high_thresh=134217728
echo 9 | sudo -S ip link set lo multicast on

XSOCK=/tmp/.X11-unix
FONTS=/usr/share/fonts # 字体问题

VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=/dev:/dev:rw
         --volume=$HOME/shared_dir:/home/erlang/shared_dir:rw
         --volume=$FONTS:$FONTS:rw" 

# --gpus all \
docker run \
    -it --rm \
    --name="xiaoche" \
    --gpus all \
    --workdir="/home/erlang/shared_dir" \
    $VOLUMES \
    --env="DISPLAY=${DISPLAY}" \
    --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --privileged \
    --net=host \
    --ipc=host \
    pm_v2x01 \
    /bin/bash


