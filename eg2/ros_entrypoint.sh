#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"