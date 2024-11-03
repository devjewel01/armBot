# Use the ROS 2 Humble desktop image as the base
FROM osrf/ros:humble-desktop

# Arguments to create a user
ARG USERNAME=jewel
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Remove any existing user with the same UID and create a new user
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Change to the user to run rosdep commands
USER jewel

# Initialize and update rosdep as a non-root user
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        sudo rosdep init; \
    fi && \
    rosdep update

# Switch back to root user for further installations
USER root

# Install missing dependencies manually
RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-control-interface \
    ros-humble-moveit-configs-utils \ 
    ros-humble-tf-transformations \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit \
    libserial-dev \
    python3-serial \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# Create the workspace directory
RUN mkdir -p /home/armBot/src
WORKDIR /home/armBot

# Copy project files into the container
COPY src/ /home/armBot/src/

# Install ROS dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Set the default user and command
USER $USERNAME
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]
