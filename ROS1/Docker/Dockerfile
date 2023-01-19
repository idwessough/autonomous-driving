FROM ros:melodic-ros-base-bionic

ENV VNC_PORT=5901 \
    VNC_RESOLUTION=1024x640 \
    DISPLAY=:1 \
    TERM=xterm \
    DEBIAN_FRONTEND=noninteractive \
    HOME=/home/user \
    PATH=/opt/TurboVNC/bin:$PATH \
    SSH_PORT=22 \
    ROS_WS=/home/user/ROS1_ws 

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
        python3-pip \
        python-matplotlib \
        evince \
        python-tk

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
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
        ros-melodic-desktop-full \
        ros-melodic-navigation \
        ros-melodic-moveit \
        ros-melodic-gazebo-ros-control \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-robot-pose-ekf \
        ros-melodic-video-stream-opencv \
        ros-melodic-tf2-tools \
        && apt-get clean 

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
RUN mkdir -p $ROS_WS
WORKDIR $ROS_WS

# Configure X server
RUN touch ~/.Xauthority && \
    mkdir ~/.vnc

# Copy in the init script
COPY start.sh /startup/start.sh
RUN sudo chmod +x /startup/start.sh
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://192.168.1.193:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=192.168.1.145" >> ~/.bashrc

ENTRYPOINT [ "/startup/start.sh" ]
CMD [ "--wait" ]