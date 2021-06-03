FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Create a new user "user1"
RUN useradd -ms /bin/bash user1 && usermod -aG sudo user1 && echo 'user1     ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /home/user1

COPY ros2_ws ros2_ws/
RUN cd ros2_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

COPY eg1/ros_entrypoint.sh /
COPY eg1/init-container.sh /opt

USER user1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD /opt/init-container.sh