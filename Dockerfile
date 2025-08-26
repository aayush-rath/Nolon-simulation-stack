# Use the official ROS2 Jazzy base image with Ubuntu 24.04
FROM osrf/ros:jazzy-desktop-full-ubuntu24.04

# Set environment variables
ENV ROS_DISTRO=jazzy
ENV GAZEBO_VERSION=ionic
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Basic tools
    curl \
    wget \
    git \
    vim \
    nano \
    software-properties-common \
    lsb-release \
    gnupg2 \
    # Build tools
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # ROS2 additional packages
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-msgs \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-simple-controller-manager \
    # Gazebo and simulation
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    # Additional utilities
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rqt-robot-steering \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    ros-jazzy-slam-toolbox \
    # GUI and X11
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Ionic (if not already included)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-ionic && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /opt/ros_ws

# Copy the source code
COPY nolon_bot_description /opt/ros_ws/src/nolon_bot_description/

# Initialize rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 setup\n\
source /opt/ros/jazzy/setup.bash\n\
source /opt/ros_ws/install/setup.bash\n\
\n\
# Export display for GUI applications\n\
export DISPLAY=${DISPLAY:-:0}\n\
\n\
# Set Gazebo environment\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros_ws/src/nolon_bot_description\n\
export GZ_VERSION=7\n\
\n\
# Execute the command\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set up user for GUI applications (optional, for development)
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g $GROUP_ID rosuser && \
    useradd -m -u $USER_ID -g $GROUP_ID -s /bin/bash rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create directories for X11 forwarding
RUN mkdir -p /tmp/.X11-unix && \
    chmod 1777 /tmp/.X11-unix

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["bash"]

# Expose common ROS ports
EXPOSE 11311 11345

# Set ownership of workspace
RUN chown -R rosuser:rosuser /opt/ros_ws

# Switch to rosuser for GUI applications
USER rosuser
WORKDIR /opt/ros_ws

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD ros2 node list || exit 1