FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Install ROS 2 deppendencies (Cyclone DDS)
RUN sudo apt update && \
    sudo apt install -y \
    ros-foxy-rmw-cyclonedds-cpp 
RUN sudo rm -rf /var/lib/apt/lists/*

WORKDIR /app

# build custom ROS 2 nodes
COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY eg4/dev1/cyclonedds.xml .
COPY eg4/dev1/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]   
CMD ["bash"]