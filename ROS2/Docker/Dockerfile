FROM osrf/ros:foxy-desktop
# set enviroment    

ENV ROS_WS=/home/ROS2_ws

ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p $ROS_WS/src

#COPY /bash /home
WORKDIR $ROS_WS

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
    ca-certificates \
    curl \
    swig \
    software-properties-common \
    python3-pip \
    x11-apps \
    x11-xserver-utils \
    xauth \
    net-tools \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/sshd
RUN echo 'root:root' | chpasswd
RUN sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#AllowTcpForwarding\s+.*/AllowTcpForwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11Forwarding\s+.*/X11Forwarding yes/g' /etc/ssh/sshd_config && \
    sed -ri 's/^#X11UseLocalhost\s+.*/X11UseLocalhost no/g' /etc/ssh/sshd_config

CMD ["/usr/sbin/sshd", "-D"]

RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

#RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
#    && sed --in-place --expression \ 
#    '$isource "$ROS_WS/devel/setup.bash"' \
#    /ros_entrypoint.sh

# run ros package launch file
#CMD [ "roslaunch", "<your package>", "<your launch file>" ]