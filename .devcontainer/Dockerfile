FROM ghcr.io/smart-wheelchair-rrc/humble_harmonic:v3.0

# Add the following labels
LABEL org.opencontainers.image.description="ROS2 Humble + ROS2 Harmonic + ROS2 Control"
LABEL org.opencontainers.image.title="humble-harmonic-control"
LABEL org.opencontainers.image.vendor="Tarun Ramakrishnan"
LABEL org.opencontainers.image.source="https://github.com/rtarun1/HepoBot"
LABEL maintainer="tarun.ramak@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

# handle default shell
SHELL ["/bin/bash", "-c"]

# Install Basic deps
RUN sudo apt-get update \
    && sudo apt-get -y install --no-install-recommends\
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    wget \
    cmake \
    build-essential \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    v4l-utils \
    xterm \
    tmux \
    vim \
    nano \
    tree \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*


RUN sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list' \
    && rosdep update \
    && rosdep resolve gz-harmonic 

COPY rosPkgs.list /tmp/rosPkgs.list
RUN sudo apt-get update \
    && sudo apt-get -y install --no-install-recommends $(cat /tmp/rosPkgs.list) \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/wheelchair2/gz_ros2_control_ws/src
RUN git clone https://github.com/ros-controls/gz_ros2_control -b humble . \
    && export GZ_VERSION=harmonic \
    && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys="ros_gz_bridge ros_gz_sim" 

WORKDIR /home/wheelchair2/gz_ros2_control_ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build \
    && echo "source ~/gz_ros2_control_ws/install/setup.bash" | sudo tee -a ~/.bashrc > /dev/null  
