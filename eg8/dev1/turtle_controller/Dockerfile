FROM osrf/ros:galactic-desktop

SHELL ["/bin/bash", "-c"]

# build custom ROS 2 nodes
COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/galactic/setup.bash && \
    colcon build