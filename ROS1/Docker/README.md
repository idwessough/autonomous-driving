# Sprint 1

pour sprint 1, utilisation de docker avec ros melodic dessus, pour piloter le robot depuis un PC distant.

### Connection docker ros2 ou ros1:

*Modifier docker compose pour changer le volume (mettre un dir qui convient au PC utilise)*

*Modifier les var ENV ROS_MASTER_URI et ROS_HOSTNAME pour corresponre avec votre systeme.*

build image docker:

```bash
docker compose build
```

Lancer docker compose:

```bash
docker compose up
```

Dans un autre terminal:

```bash
ssh -Y user@localhost
```

mdp:

```bash
password
```

Si probleme pour relancer le docker, supprimer l'image, puis relancer:

```bash
docker rm ros-melodic
```
