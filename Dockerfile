# Use ROS 2 Jazzy desktop as base image
FROM osrf/ros:jazzy-desktop

# Labels for maintainer info
LABEL description="Development environment for armBot 6-DOF robotic arm"
LABEL version="1.0"

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install graphics and rendering dependencies
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libegl1-mesa \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    vim \
    python3-transforms3d \
    python3-numpy \
    # ROS 2 packages
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-control-msgs \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-moveit \
    ros-jazzy-moveit-common \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
ENV WORKSPACE=/armBot
WORKDIR $WORKSPACE

# Create workspace structure
RUN mkdir -p $WORKSPACE/src

# Copy the armBot project into the container
COPY /src $WORKSPACE/src/

# Setup rosdep
RUN rosdep update --rosdistro=jazzy

# Install dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c '. /opt/ros/jazzy/setup.bash && \
    cd $WORKSPACE && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Environment setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=$WORKSPACE/install/armbot_description/share/armbot_description/meshes:\$GAZEBO_MODEL_PATH" >> ~/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=$WORKSPACE/install/armbot_description/share/armbot_description:\$GAZEBO_RESOURCE_PATH" >> ~/.bashrc && \
    echo "export DISPLAY=:0" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc

# Setup display for both Linux and Windows
RUN echo '#!/bin/bash\n\
if [ -z "$DISPLAY" ]; then\n\
    export DISPLAY=host.docker.internal:0.0\n\
fi\n\
export LIBGL_ALWAYS_INDIRECT=1\n\
source /opt/ros/jazzy/setup.bash\n\
source $WORKSPACE/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]