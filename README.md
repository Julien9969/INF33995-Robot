# Repos pour le robot

### Robot

Pour rouler le docker du robot, il suffit de lancer le script `robot/launch-robot.sh` et de rentrer le mot de passe de l'utilisateur nvidia sur le robot lorsque demandé. Le script prend en paramètre l'adresse ip des robots qu'on souhaite lancer ainsi que la branch du dépot git sur laquelle on souhaite que les robots se synchronise. Par défaut, les adresses ip de nos robots sont prises en compte et la branche main est choisie.

Si une erreur survient ou si pour tout autre raison vous souhaitez ne pas utiliser ce script, vous pouvez vous connecter directement au robot, cloner le dépot si ce n'est pas déjà fait, naviguer à `INF3995-Robot/robot/` et, modifier la valeur de la variable d'environnement `ROBOT_NUM` du fichier `docker-compose.yml` afin que celle du robot courant soit unique, puis lancer le robot avec `docker compose up`.

### Simulation

Pour rouler le docker de la simulation, utiliser directement le `Dockerfile` de `simulation/docker-simulation`.

Il est ensuite possible de lancer le docker avec la commande suivante:

`source <emplacement_de_votre_venv_avec_rocker>/bin/activate && rocker --x11 --network=host --device=/dev/dri --volume <emplacement_de_votre_clone_de_INF3995-Robot>:/root/INF3995-Robot --image-name=rosign rosignbase`
