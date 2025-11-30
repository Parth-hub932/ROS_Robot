# Use the official ARM64 ROS 2 Humble base image
FROM arm64v8/ros:humble-ros-base

# Set environment variable to prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies needed for your project
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    ros-humble-rosbridge-server \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS 2 entrypoint to automatically source the environment
COPY --from=ros:humble-ros-base /ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Launch a bash shell by default when the container starts
CMD ["bash"]