# Repos pour le robot

### Robot

Pour rouler le docker du robot, il suffit de lancer le script `robot/launch-robot.sh` et de rentrer le mot de passe de l'utilisateur nvidia sur le robot lorsque demandé. Le script prend en paramètre l'adresse ip des robots qu'on souhaite lancer, la branch du dépot git sur laquelle on souhaite que les robots se synchronisent ainsi que la commande ros2 que l'on souhaite exécuter sur le robot. Les paramètres sont tous optionnels. Une option par défaut permet d'exécuter du code automatiquement sur le robot sans spécifier de paramètre à la commande. **Il est important d'attendre 1 minute ou 2 après que la commande launch-robot.sh ait terminé pour donner le temps au robot de se déployer**.

Si une erreur survient ou si pour tout autre raison vous souhaitez ne pas utiliser ce script, vous pouvez vous connecter directement au robot, cloner le dépot si ce n'est pas déjà fait, naviguer à `INF3995-Robot/robot/`, construire le robot avec `docker build -t docker-robot .` puis lancer le conteneur avec la commande suivante en remplacant les éléments entre crochet:

``` bash
docker run -d --rm --network=host --ipc=host --pid=host --device=/dev/ttyTHS1 --device=/dev/ydlidar -v /home/nvidia/INF3995-Robot:/root/INF3995-Robot -v /tmp/.X11-unix:/tmp/.X11-unix --env ROS_DOMAIN_ID=62 --env ROBOT_NUM=<numéro_du_robot> docker-robot bash -c '/root/clean_workspace.sh && source /opt/ros/humble/setup.bash && cd root && /root/deploy-robot.sh && source /root/INF3995-Robot/ros_ws/install/setup.bash && <commande_ros_à_exécuter>'
```

### Simulation

Pour rouler le docker de la simulation, utiliser directement le `Dockerfile` de `simulation/docker-simulation`.

docker build -t "rosignbase" ./simulation/docker-simulation/

Il est ensuite possible de lancer le docker avec la commande suivante:

`source <emplacement_de_votre_venv_avec_rocker>/bin/activate && rocker --x11 --device=/dev/dri --volume $(pwd):/root/INF3995-Robot -p 22900:22900 -p 22901:22901 -p 22902:22902 -p 22910-22921:22910-22921 --image-name=rosign --name=simulation-ign rosignbase`


**Lancer la simulation (sur le rocker)**
```
./deploy-simulation.sh
```
**Ouvrir un shell dans la simulation**
```
docker exec -it simulation-ign bash 
```
**Source setup.sh (dans la simulation)**
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
