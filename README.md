# Darknet ROS


## Dans le .bashrc : 

export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

ROS_MASTER_URI=HTTP://192.168.1.166:11311
ROS_HOSTNAME=192.168.1.150 


## Sur le LIMO : 

Terminal 1 -> roscore

Terminal 2 -> roslaunch astra_camera dabai_u3.launch


## Sur le PC : 

Aller dans vision_ws

Terminal -> roslaunch darknet_ros darknet_ros.launch

