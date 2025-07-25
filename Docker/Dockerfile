# Use ROS 2 Jazzy as the base image
FROM ros:jazzy

# Set up the environment
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    python3-pyqt6 \
    python3-sympy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Copy the workspace
WORKDIR /root/multi_robots_ws
COPY ../. /root/multi_robots_ws
