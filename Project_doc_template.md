auteurs : 

# Listes des fonctionnalités :

1. [x] RO
2. [ ] Incomplete features
    1. [ ] Sub-task 1
    2. [x] Sub-task 2
3. [x] ROS Navigation Stack
    1. [x] Sub-task 1
    2. [x] Sub-task 2

# Représentation des noeuds

```mermaid
graph LR
    T1[Odometry source] -- /odom --> Node((local_planner_student))
    T2[Laser source] -- /scan --> Node((local_planner_student))

    S1[ ] -. /move_to/singleGoal .-> Node
    S2[ ] -. /move_to/pathGoal .-> Node

    Node -- /cmd_vel_mux/input/navi -->D[base controller]
```

# Description de l'algo

```mermaid
sequenceDiagram
    participant Alice
    participant Bob
    Alice->John: Hello John, how are you?
    loop Healthcheck
        John->John: Fight against hypochondria
    end
    Note right of John: Rational thoughts <br/>prevail...
    John-->Alice: Great!
    John->Bob: How about you?
    Bob-->John: Jolly good!
```

# Vidéos de présentation

[Lien vers la vidéo pitch youtube](url)

[Lien vers la vidéo tutoriel youtube](url)

# Liste des dépendances et pré-requis

- a
- b

# Procédure de mise en route

- a
- b
- n