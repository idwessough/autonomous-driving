FROM osrf/ros:foxy-desktop
# set enviroment    

ENV VNC_PORT=5901 \
    VNC_RESOLUTION=1024x640 \
    DISPLAY=:1 \
    TERM=xterm \
    HOME=/home/user \
    DEBIAN_FRONTEND=noninteractive \
    PATH=/opt/TurboVNC/bin:$PATH \
    SSH_PORT=22 \
    ROS_WS=/home/user/ROS2_ws

EXPOSE $VNC_PORT
EXPOSE $SSH_PORT

RUN mkdir -p $ROS_WS/src

# install ros dependencies packages
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
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
    locales \
    ca-certificates \
    curl \
    swig \
    sudo \
    supervisor \
    wget \
    software-properties-common \
    python3-pip \
    dbus-x11 \
    libexo-1-0 \
    x11-apps \
    x11-xserver-utils \
    xauth \
    xfce4 \
    xfce4-terminal \
    xterm \
    net-tools \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

ENV TVNC_WM=xfce4-session

ENV TVNC_VERSION=2.2.2
RUN export TVNC_DOWNLOAD_FILE="turbovnc_${TVNC_VERSION}_amd64.deb" && \
    wget -q -O $TVNC_DOWNLOAD_FILE "https://sourceforge.net/projects/turbovnc/files/2.2.2/${TVNC_DOWNLOAD_FILE}/download" && \
    dpkg -i $TVNC_DOWNLOAD_FILE && \
    rm -f $TVNC_DOWNLOAD_FILE

RUN mkdir -p /var/run/sshd
RUN sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#AllowTcpForwarding\s+.*/AllowTcpForwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11Forwarding\s+.*/X11Forwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11UseLocalhost\s+.*/X11UseLocalhost no/g' /etc/ssh/sshd_config

RUN useradd -ms /bin/bash user && \
    adduser user sudo && \
    usermod -aG sudo user && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
#USER user
WORKDIR $ROS_WS

RUN sudo touch ~/.Xauthority && \
    sudo mkdir ~/.vnc

COPY start.sh /startup/start.sh
RUN sudo chmod +x /startup/start.sh
RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT [ "/startup/start.sh" ]
CMD [ "--wait" ]



#RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
#    && sed --in-place --expression \ 
#    '$isource "$ROS_WS/devel/setup.bash"' \
#    /ros_entrypoint.sh

# run ros package launch file
#CMD [ "roslaunch", "<your package>", "<your launch file>" ]