FROM osrf/ros:foxy-desktop

ENV VNC_PORT=5901 \
    VNC_RESOLUTION=1024x640 \
    DISPLAY=:1 \
    TERM=xterm \
    DEBIAN_FRONTEND=noninteractive \
    HOME=/home/user \
    PATH=/opt/TurboVNC/bin:$PATH \
    SSH_PORT=22 \
    ROS_WS=/home/user/ROS2_ws 

# Install shared utils
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        apt-utils \
        ca-certificates \
        locales \
        net-tools \
        sudo \
        supervisor \
        wget \
        openssh-server \
        build-essential \
        cmake \
        git \
        libssl-dev \
        libusb-1.0-0-dev \
        pkg-config \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \    
        curl \	
        libusb-1.0-0 \
        udev \
        apt-transport-https \
        curl \
        swig \
        software-properties-common \
        python3-pip 

# Install XFCE and terminal
RUN apt-get install -y --no-install-recommends \
        dbus-x11 \
        libexo-1-0 \
        x11-apps \
        x11-xserver-utils \
        xauth \
        xfce4 \
        xfce4-terminal \
        xterm
ENV TVNC_WM=xfce4-session

# Install extra ros2 packages
RUN apt-get install -y --no-install-recommends \
        ros-foxy-cartographer \
        ros-foxy-rmw-fastrtps* \
        ros-foxy-navigation2 \
        ros-foxy-nav2-bringup \
        ros-foxy-nav2-amcl \
        ros-foxy-joint-state-publisher-gui \
        ros-foxy-gazebo-ros-pkgs \
        ros-foxy-xacro

# Install TurboVNC
ENV TVNC_VERSION=2.2.2
RUN export TVNC_DOWNLOAD_FILE="turbovnc_${TVNC_VERSION}_amd64.deb" && \
    wget -q -O $TVNC_DOWNLOAD_FILE "https://sourceforge.net/projects/turbovnc/files/2.2.2/${TVNC_DOWNLOAD_FILE}/download" && \
    dpkg -i $TVNC_DOWNLOAD_FILE && \
    rm -f $TVNC_DOWNLOAD_FILE

# Configure SSH server
RUN mkdir -p /var/run/sshd
RUN sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#AllowTcpForwarding\s+.*/AllowTcpForwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11Forwarding\s+.*/X11Forwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11UseLocalhost\s+.*/X11UseLocalhost no/g' /etc/ssh/sshd_config

# Setup another user
RUN useradd -ms /bin/bash user && \
    adduser user sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR $ROS_WS

# Configure X server
RUN touch ~/.Xauthority && \
    mkdir ~/.vnc

# Copy in the init script
COPY start.sh /startup/start.sh
RUN sudo chmod +x /startup/start.sh
RUN echo "export ROS_DOMAIN_ID=228" >> ~/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/startup/start.sh" ]
CMD [ "--wait" ]