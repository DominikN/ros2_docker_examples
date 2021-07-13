FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Install Husarnet Client
RUN apt update -y
RUN apt install -y curl gnupg2 systemd
RUN curl https://install.husarnet.com/install.sh | bash
RUN update-alternatives --set ip6tables /usr/sbin/ip6tables-legacy

# Install ROS 2 deppendencies (Cyclone DDS)
RUN sudo apt update && \
    sudo apt install -y \
    ros-foxy-rmw-cyclonedds-cpp 
RUN sudo rm -rf /var/lib/apt/lists/*

# Find your JOINCODE at https://app.husarnet.com
ENV JOINCODE=""
ENV HOSTNAME=my-container-1


WORKDIR /app

COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY eg3/dev1/ros_entrypoint.sh /
COPY eg3/dev1/cyclonedds.xml .

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]