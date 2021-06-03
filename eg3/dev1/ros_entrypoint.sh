#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /home/user1/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/user1/cyclonedds.xml

exec "$@"