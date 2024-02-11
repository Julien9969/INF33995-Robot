# Repos pour le robot

### Robot

Pour rouler le docker du robot, utiliser le fichier `robot/docker-compose.yml`. Celui-ci permet au docker d'avoir accès aux périphériques du robot.

### Simulation

Pour rouler le docker de la simulation, utiliser directement le `Dockerfile` de `simulation/docker-simulation`.

Il est ensuite possible de lancer le docker avec la commande suivante:

`source <emplacement_de_votre_venv_avec_rocker>/bin/activate && rocker --x11 --network=host --device=/dev/dri --volume <emplacement_de_votre_clone_de_INF3995-Robot>:/root/INF3995-Robot --image-name=rosign rosignbase`


Start Rocker
```
rocker --x11 --network=host --device=/dev/dri --volume $(pwd):/root/INF3995-Robot  --image-name=rosign --name=simulation-ign rosignbase 
```
**Lancer la simulation**
```
./deploy-simulation.sh
```
**Open a Bash in Rocker**
```
docker exec -it simulation-ign bash 
```
**Source setup.sh**
```
source INF3995-Robot/ros_ws/install/setup.sh
```

**Le topic ne reçoit rien ?**
```
ros2 topic info </topic_name>
```

**Tester la connection rapidement**  
D'abord publish le ros2 topic pub --rate 1 /limo/cmd_vel... dans le backend  
**puis**
```
ros2 topic echo /limo/cmd_vel
```
