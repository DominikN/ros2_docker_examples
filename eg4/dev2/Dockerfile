FROM osrf/ros:foxy-desktop

# Install ROS 2 deppendencies (Cyclone DDS)
RUN sudo apt update && \
    sudo apt install -y \
    ros-foxy-rmw-cyclonedds-cpp 
RUN sudo rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY cyclonedds.xml .
COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]   
CMD ["bash"]