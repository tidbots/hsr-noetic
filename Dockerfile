# Copyright (c) 2023 Hiroyuki Okada
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php
#FROM ubuntu:20.04
FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

LABEL maintainer="Hiroyuki Okada <hiroyuki.okada@okadanet.org>"
LABEL org.okadanet.vendor="Hiroyuki Okada" \
      org.okadanet.dept="TRCP" \
      org.okadanet.version="1.0.0" \
      org.okadanet.released="July 11, 2023"

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
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


# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  dirmngr \
  git build-essential  \
  cmake \
  g++ \
  iproute2 gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip \
  python3-tk \
  git wget curl sudo python-is-python3 \
  mesa-utils x11-utils x11-apps terminator xterm xauth \
  terminator xterm nano vim htop \
  software-properties-common gdb valgrind sudo \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Install ROS1 noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --allow-downgrades --allow-remove-essential --allow-change-held-packages \
libpcap-dev \
libopenblas-dev \
gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
ros-noetic-desktop-full \
python3-rosdep python3-rosinstall-generator python3-vcstool build-essential \
ros-noetic-socketcan-bridge \
ros-noetic-geodesy \
python3-catkin-tools ros-noetic-gmapping ros-noetic-navigation && \
apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip install rosnumpy transformers stitching

# Install tmux 3.2(if you wish)
RUN apt-get update && apt-get install -y automake autoconf pkg-config libevent-dev libncurses5-dev bison && \
apt-get clean && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/tmux/tmux.git && \
cd tmux && git checkout tags/3.2 && ls -la && sh autogen.sh && ./configure && make -j8 && make install

# Install HSR ROS packages
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add -
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN sh -c 'mkdir -p /etc/apt/auth.conf.d'
RUN sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
RUN sh -c '/bin/echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences'
RUN apt-get update
RUN apt-get install -y --no-install-recommends ros-noetic-tmc-desktop-full \
    && rm -rf /var/lib/apt/lists/*


# COPY assets/avahi-daemon.conf /etc/avahi/avahi-daemon.conf

#RUN apt update \
#  && apt install -y iputils-ping net-tools chrony 
ARG ROBOT_NAME
#RUN echo "server ${ROBOT_NAME}.local" >> /etc/chrony/chrony.conf
#RUN echo "driftfile /var/lib/chrony/chrony.drift" >> /etc/chrony/chrony.conf
#RUN echo "keyfile /etc/chrony/chrony.keys" >> /etc/chrony/chrony.conf
#RUN echo "generatecommandkey" >> /etc/chrony/chrony.conf
#RUN echo "log tracking measurements statistics" >> /etc/chrony/chrony.conf
#RUN echo "logdir /var/log/chrony" >> /etc/chrony/chrony.conf
#RUN echo "local stratum 10" >> /etc/chrony/chrony.conf
#RUN echo "allow ${ROBOT_NAME}.local" >> /etc/chrony/chrony.conf
#RUN echo "logchange 0.5" >> /etc/chrony/chrony.conf
#RUN echo "initstepslew 20 ${ROBOT_NAME}.local" >> /etc/chrony/chrony.conf

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME
ARG PASSWORD
ARG NETWORK_IF
ARG WORKSPACE_DIR
RUN groupadd -g $GID $GROUP_NAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USER_NAME && \
    echo $USER_NAME:$PASSWORD | chpasswd && \
    echo "$USER_NAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USER_NAME}
#RUN sudo rosdep init && rosdep update

RUN cd ~/ &&  \
    git config --global user.email "you@example.con" &&  \
    git config --global user.name "Your Name"

# Config
RUN mkdir -p ~/.config/terminator/
COPY assets/terminator_config /home/$USER_NAME/.config/terminator/config 

COPY assets/entrypoint.sh /entrypoint.sh

RUN echo "# please set network-interface" >> ~/.bashrc
RUN echo "network_if=${NETWORK_IF}" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "if [ -e /opt/ros/noetic/setup.bash ] ; then" >> ~/.bashrc
RUN echo "    source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "else" >> ~/.bashrc
RUN echo '    echo "ROS packages are not installed."' >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "export TARGET_IP=\$(LANG=C /sbin/ip address show  \$network_if | grep -Eo  'inet (addr:)?([0-9]*\.){3}[0-9]*'| grep -Eo '([0-9]*\.){3}[0-9]*') " >> ~/.bashrc

RUN echo 'if [ -z "$TARGET_IP" ] ; then' >> ~/.bashrc
RUN echo '    echo "ROS_IP is not set."' >> ~/.bashrc
RUN echo "else" >> ~/.bashrc
RUN echo '    export ROS_IP=$TARGET_IP' >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "export ROS_HOME=~/.ros" >> ~/.bashrc
RUN echo "export ROBOT_NAME=hsrc" >> ~/.bashrc
RUN echo "alias sim_mode='export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11311 export PS1=\"\[\033[44;1;37m\]<local>\[\033[0m\]\w$ \"'" >> ~/.bashrc
#RUN echo "export ROS_MASTER_URI=http://hsrc28.local:11311" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://${ROBOT_NAME}.local:11311" >> ~/.bashrc
RUN echo "export PS1=\"\[\033[44;1;37m\]<hsrc>\[\033[0m\]\w$ \"" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ros/devel/setup.bash" >> ~/.bashrc
RUN echo "export MESA_LOADER_DRIVER_OVERRIDE=i965" >> ~/.bashrc

RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p ~/catkin_ros/src && cd ~/catkin_ros/src && \
    catkin_init_workspace && \
    cd ~/catkin_ros && \
    catkin_make

# entrypoint
COPY ./assets/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
#CMD ["bash"]
CMD ["terminator"]
