# Copyright (c) 2023 Hiroyuki Okada
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

LABEL maintainer="Hiroyuki Okada <hiroyuki.okada@okadanet.org>"
LABEL org.okadanet.vendor="Hiroyuki Okada" \
      org.okadanet.dept="TRCP" \
      org.okadanet.version="1.0.0" \
      org.okadanet.released="July 11, 2023"

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ARG ROS_DISTRO=noetic

# Locale + fonts
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales language-pack-ja-base language-pack-ja tzdata \
    fonts-ipafont fonts-ipaexfont fonts-takao \
 && locale-gen ja_JP.UTF-8 \
 && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=ja_JP.UTF-8 TZ=Asia/Tokyo

# Install basic packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    git \
    build-essential \
    cmake \
    g++ \
    iproute2 \
    gnupg \
    gnupg2 \
    libcanberra-gtk* \
    python3-pip \
    python3-tk \
    wget \
    curl \
    sudo \
    python-is-python3 \
    mesa-utils \
    x11-utils \
    x11-apps \
    terminator \
    xterm \
    xauth \
    nano \
    vim \
    htop \
    software-properties-common \
    gdb \
    valgrind \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Install ROS1 Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt-get update && apt-get install -y --allow-downgrades --allow-remove-essential --allow-change-held-packages \
    libpcap-dev \
    libopenblas-dev \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-noetic-socketcan-bridge \
    ros-noetic-geodesy \
    python3-catkin-tools \
    ros-noetic-gmapping \
    ros-noetic-navigation \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip install --no-cache-dir \
    rosnumpy \
    transformers \
    stitching

# Install tmux 3.2 from source
RUN apt-get update && apt-get install -y --no-install-recommends \
    automake \
    autoconf \
    pkg-config \
    libevent-dev \
    libncurses5-dev \
    bison \
 && apt-get clean && rm -rf /var/lib/apt/lists/* \
 && git clone --depth 1 --branch 3.2 https://github.com/tmux/tmux.git /tmp/tmux \
 && cd /tmp/tmux \
 && sh autogen.sh \
 && ./configure \
 && make -j$(nproc) \
 && make install \
 && rm -rf /tmp/tmux

# Install HSR ROS packages
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list' \
 && sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list' \
 && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget -qO - https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key | apt-key add - \
 && wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
 && mkdir -p /etc/apt/auth.conf.d \
 && echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" > /etc/apt/auth.conf.d/auth.conf \
 && echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\n\
Package: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences \
 && apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-tmc-desktop-full \
 && rm -rf /var/lib/apt/lists/*

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME
ARG PASSWORD
ARG NETWORK_IF
ARG WORKSPACE_DIR
ARG ROBOT_NAME
RUN groupadd -g $GID $GROUP_NAME \
 && useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USER_NAME \
 && echo $USER_NAME:$PASSWORD | chpasswd \
 && echo "$USER_NAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Copy config files (as root)
COPY assets/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to user
USER ${USER_NAME}

# Git config
RUN git config --global user.email "you@example.com" \
 && git config --global user.name "Your Name"

# Terminator config
RUN mkdir -p ~/.config/terminator/
COPY --chown=${USER_NAME}:${GROUP_NAME} assets/terminator_config /home/$USER_NAME/.config/terminator/config

# Setup bashrc with ROS configuration
RUN cat >> ~/.bashrc <<'BASHRC_EOF'

# Network interface for ROS
network_if=NETWORK_IF_PLACEHOLDER

# Source ROS setup
if [ -e /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "ROS packages are not installed."
fi

# Set ROS_IP from network interface
export TARGET_IP=$(LANG=C /sbin/ip address show $network_if 2>/dev/null | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*')
if [ -z "$TARGET_IP" ]; then
    echo "ROS_IP is not set."
else
    export ROS_IP=$TARGET_IP
fi

export ROS_HOME=~/.ros
export ROBOT_NAME=ROBOT_NAME_PLACEHOLDER
export ROS_MASTER_URI=http://${ROBOT_NAME}.local:11311
export PS1="\[\033[44;1;37m\]<hsrc>\[\033[0m\]\w$ "

# Simulator mode alias
alias sim_mode='export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311; export PS1="\[\033[44;1;37m\]<local>\[\033[0m\]\w$ "'

# Source ROS and catkin workspace
source /opt/ros/noetic/setup.bash
if [ -f ~/catkin_ros/devel/setup.bash ]; then
    source ~/catkin_ros/devel/setup.bash
fi

export MESA_LOADER_DRIVER_OVERRIDE=i965
BASHRC_EOF

# Replace placeholders with actual values
RUN sed -i "s/NETWORK_IF_PLACEHOLDER/${NETWORK_IF}/g" ~/.bashrc \
 && sed -i "s/ROBOT_NAME_PLACEHOLDER/${ROBOT_NAME}/g" ~/.bashrc

# Setup catkin workspace
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p ~/catkin_ros/src \
 && cd ~/catkin_ros/src \
 && catkin_init_workspace

# Copy and build sample packages
COPY --chown=${USER_NAME}:${GROUP_NAME} ./sample/hsrb_samples /home/$USER_NAME/catkin_ros/src
RUN source /opt/ros/noetic/setup.bash \
 && cd ~/catkin_ros \
 && catkin_make

# Copy scripts
COPY --chown=${USER_NAME}:${GROUP_NAME} ./scripts /home/$USER_NAME/scripts

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /home/$USER_NAME
CMD ["terminator"]
