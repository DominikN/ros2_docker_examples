FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Fix ros key
# NOTE: Needed because of this bug https://github.com/osrf/docker_images/issues/535
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS 2 deppendencies (Cyclone DDS)
RUN sudo apt update && \
    sudo apt install -y \
    ros-foxy-rmw-cyclonedds-cpp 
RUN sudo rm -rf /var/lib/apt/lists/*

# build custom ROS 2 nodes
COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY eg4/dev1/ros_entrypoint.sh /
COPY eg4/dev1/cyclonedds.xml /

ENTRYPOINT ["/ros_entrypoint.sh"]   
CMD ["bash"]