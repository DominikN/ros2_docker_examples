FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Fix ros key
# NOTE: Needed because of this bug https://github.com/osrf/docker_images/issues/535
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

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

# Create a new user "user1"
RUN useradd -ms /bin/bash user1 && usermod -aG sudo user1 && echo 'user1     ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /home/user1

COPY eg3/dev2/ros_entrypoint.sh /
COPY eg3/dev2/init-container.sh /opt
COPY eg3/dev2/cyclonedds.xml .

USER user1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD /opt/init-container.sh