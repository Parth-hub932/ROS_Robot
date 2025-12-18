# Stage 1: Define the base image and install dependencies
# Use the official ARM64 ROS 2 Humble base image
FROM arm64v8/ros:humble-ros-base

# Set environment variable to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
# Added: build-essential, libserial-dev (for C++ UART), and ROS control/localization packages
# NEW ADDITIONS: tf-transformations and math libraries for PID
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    ros-humble-rosbridge-server \
    git \
    build-essential \
    libserial-dev \
    ros-humble-ros2-control \
    ros-humble-hardware-interface \
    ros-humble-ros2-controllers \
    ros-humble-robot-localization \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-tf-transformations \
    python3-dev \
    libblas-dev \
    liblapack-dev \
    gfortran \
    && rm -rf /var/lib/apt/lists/*

# Install Python transforms3d library (Required for PID math)
RUN pip3 install transforms3d

COPY --from=ros:humble-ros-base /ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]