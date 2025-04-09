# Use ROS2 Foxy as the base image
FROM ros:foxy

# Set non-interactive mode to prevent tzdata prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

# Add ROS2 apt repository
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - && \
    sh -c 'echo "deb [arch=amd64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update and install ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-setuptools \
    build-essential \
    python3-rosdep \
    python3-colcon-ros \
    python3-rosinstall \
    python3-vcstool \
    ros-foxy-rclpy \
    ros-foxy-geometry-msgs \
    ros-foxy-launch \
    ros-foxy-launch-ros \
    ros-foxy-ament-cmake-python \
    libpython3-dev \
    python3-serial

# Upgrade pip and install Python dependencies
RUN pip3 install --upgrade pip pyserial

# Create workspace directory
WORKDIR /ros_ws


# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# Set environment variables
ENV ROS_DISTRO=foxy
ENV ROS_WS=/ros_ws
ENV PATH="$PATH:/ros_ws/install/bin"

# Default command to keep the container running
CMD ["/bin/bash"]

