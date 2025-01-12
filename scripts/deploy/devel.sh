#!/bin/bash

# Get base.sh funcs
source "$(dirname "$0")/base.sh"

stop_docker

mode="gpu"

while getopts 'ch' opt; do
    case "$opt" in
        c)
            mode="cpu"
            ;;
        ?|h)
            echo "Usage: $(basename $0) [-c]"
            exit 1
            ;;
    esac
done
shift "$(($OPTIND -1))"

if [ "$mode" == "gpu" ]; then
    run_docker --runtime=nvidia \
    -v $absolute_path/workspace/:/root/workspace/src \
    mp_ros2:latest bash
else
    run_docker \
    -v $absolute_path/workspace/:/root/workspace/src \
    mp_ros2:latest bash
fi
