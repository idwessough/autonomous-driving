FROM ros:melodic-ros-base
# set enviroment    
ENV ROS_MASTER_URI=http://192.168.1.193:11311
#ENV ROS_HOSTNAME=172.18.0.2
ENV ROS_WS=/home/ROS1_ws

RUN mkdir -p $ROS_WS/src

#COPY /bash /home
WORKDIR $ROS_WS

# install ros dependencies packages
RUN apt update && apt-get install -y \
    ros-melodic-desktop-full \
    ros-melodic-navigation \
    ros-melodic-moveit \
    ros-melodic-gazebo-ros-control \
    ros-melodic-teleop-twist-keyboard \
    apt-get install ros-melodic-robot-pose-ekf \
    && apt-get clean 

#RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
#    && sed --in-place --expression \ 
#    '$isource "$ROS_WS/devel/setup.bash"' \
#    /ros_entrypoint.sh

# run ros package launch file
#CMD [ "roslaunch", "<your package>", "<your launch file>" ]