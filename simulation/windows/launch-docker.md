## Lancer la simulation Gazebo sur Windows

### Build docker
Dans la base du répertoire INF3995-Robot:

```sh
docker build -t simulation-docker ./simulation/windows
```
### Run docker
Puis, **dans la base du répertoire!!** INF3995-Robot: (et non dans le dossier windows)
```sh
docker run --rm -p 5901:5901 -v  .:/root/INF3995-Robot -it simulation-docker bash -c "TVNC_WM=xfce /opt/TurboVNC/bin/vncserver -securitytypes None; /bin/bash"
```
### Connect to simulation
Enfin, se connecter par vnc (ex. RealVNC), à localhost:5901.

Et voilà!