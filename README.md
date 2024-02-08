# Repos pour le robot

Pour rouler le docker du robot, utiliser le `docker-compose.yml`

Pour rouler le docker de la simulation, utiliser directement le `Dockerfile` de `docker-simulation`

Il est ensuite possible de lancer le docker avec la commande suivante:

`source <emplacement_de_votre_venv_avec_rocker>/bin/activate && rocker --x11 --network=host --device=/dev/dri --volume <emplacement_de_votre_clone_de_INF3995-Robot>:/root/INF3995-Robot --image-name=rosign rosignbase`
