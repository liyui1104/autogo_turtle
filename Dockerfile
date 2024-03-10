# Use osrf/ros:humble-desktop-full as base image
FROM osrf/ros:humble-desktop-full

# Set author information
LABEL maintainer="Guanyu Lai <liyui1104@gmail.com>"

# Set ROS2 workspace
ENV WORKSPACE=ros2_ws

# Copy autogo_turtle package
COPY autogo_turtle /opt/ros/${WORKSPACE}/src/autogo_turtle

# Set working directory to ROS2 workspace
WORKDIR /opt/ros/${WORKSPACE}

# Compile autogo_turtle package and update /ros_entrypoint.sh
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
    && sed --in-place --expression \
        '$isource "/opt/ros/${WORKSPACE}/install/setup.bash" --' \
        /ros_entrypoint.sh

