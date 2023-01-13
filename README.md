# Navigation de véhicules en milieu urbain

auteurs : Idwes Sough, Arthur Saunier, Younes Abouchi

## Objectifs
Faire naviguer des véhicules (Turtlebot + [Limo](https://global.agilex.ai/products/limo)) dans un environnement urbain (route, feu, piétons, signalisation, ...) en respectant sa réglementation (laisser) 

##

Informations pour tag vIntermediate:

Informations de compilation et demo ds le dossier neural network.

## Listes des fonctionnalités :

1. [ ] Un conteneur docker contenant :
    1. [ ] Un réseau de neurone YOLO/Darknet
        1. [ ] Reconnaissance panneaux
        2. [ ] Reconnaissance personnage
        3. [ ] Reconnaissance signalisation lumineuse
    2. [ ] ROS node
        1. [ ] Flux vidéo en entrée
        2. [ ] Post-traitement
        3. [ ] Stack navigation
        

# Demo avec camera pc

./darknet detector demo YOLOV3_YCB_tiny/ycb.data YOLOV3_YCB_tiny/yolov3-tiny-traffic.cfg YOLOV3_YCB_tiny/backup/yolov3-tiny-traffic.weights

# Demo avec camera limo



## On Limo:
- lancer roscore
- Lancer LIDAR
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
- Lancer caméra
roslaunch astra_camera dabai_u3.launch
- lancer nav (après avoir mapper l'environnement)
roslaunch limo_bringup limo_navigation_ackerman.launch

Lancer ROSCore de Limo
rosrun web_video_server web_video_server

topic
/camera/rgb/image_raw
modifier CUDA version dans MakeFile 
NVCC=/usr/local/cuda-11.8/bin/nvcc
./darknet detector demo YOLOV3_YCB_tiny/ycb.data YOLOV3_YCB_tiny/yolov3-tiny-traffic.cfg YOLOV3_YCB_tiny/backup/yolov3-tiny-traffic.weights http://localhost:8080/stream?topic=/camera/rgb/image_raw



## Rendu spécifique au projet :
- 
  - Un réseau de neurone dont les inférences tournent sur GPU:
    - Classe requises :
      - Tous les panneaux fournis (jouets)
      - Tous les personnages fournis (jouets)
      - Signalisation lumineuse (état des  )
  - Un noeud ROS avec :
    - en entrée : un flux vidéo (caméra)
    - un traitement pour définir le déplacement du robot:
      - Réseau de neurone
      - Post-traitement, par exemple l'état du feu (roue, vert, orange)
    - en sortie : à minima une commande en vélocité du robot, mais idéalement un client d'action lié à la couche de navigation du robot 

La solution dockerisée devra pouvoir tourner indépendamment sur un Turtlebot (sur PC Triton, durant le sprint 1), un Limo (sur Jetson Nano, durant le sprint 2)  

Le projet nécessite de maquetter une route, en collaboration avec le projet de Smart City

## Technologies
* ROS
* Python
* darknet/TF
* Docker

## Liens utiles
* []()






