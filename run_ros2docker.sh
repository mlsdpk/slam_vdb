#!/bin/bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

export ROS_DEV_CONTAINER_NAME="slam_vdb_ros_jazzy"
export ROS_PROJECT_PATH="$SCRIPTPATH"

echo "Creating ros2 docker container named $ROS_DEV_CONTAINER_NAME..."
echo "Mounting the project directory $ROS_PROJECT_PATH to /slam_vdb_ws in the container..."

docker pull ghanta1996/slam_vdb:ros_jazzy

docker run -it -v $ROS_PROJECT_PATH:/slam_vdb_ws/slam_vdb -e DISPLAY=docker.for.mac.host.internal:0 ros:jazzy bash
