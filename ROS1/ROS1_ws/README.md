# Lancement du robot avec suivi de ligne:

### demarrage robot (appuie long sur bouton jusqu'au signal sonore)

### Se connecter au robot en ssh

> ssh agilex@ip

mdp:
> agx

Ouvrir 3 fenetre ssh, et lancer ces 3 commandes:
> roscore

> roslaunch astra_camera dabai_u3.launch

> roslaunch limo_bringup limo_start.launch

### Sur PC avec carte graphique Nvidia, en local

*Modifier les var ENV ROS_MASTER_URI et ROS_HOSTNAME bans ./bashrc pour correspondre avec votre systeme.*

Ne pas oublier de source bashrc et devel/setup.bash

### Lancement ROS-darknet

> 

### Apres le lancement du [docker](../Docker/README.md):

> cd ROS1_ws

> source devel/setup.bash

### Lancement line_follower:

> roslaunch line_follower line_follower.launch

### Lancement machine a etat:

> roslaunch nav_limo_projet nav.launch

### Lancement identification panneaux

> roslaunch test_rosdarknet rosdarknet.launch







## Lien utiles suivi de lignes:

- https://github.com/nsa31/Line-Lane-Follower-Robot_ROS/blob/master/white_yellow_lane_follower_sim.py
- https://github.com/MHWK-Git/ROS_Systems_Engineering_Project/blob/main/limo_pov/limo_linetrack_node.py
- https://automaticaddison.com/