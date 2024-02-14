## Lancer la simulation Gazebo sur Windows

### Build docker
Dans le répertoire windows:

```sh
docker build -t simulation-docker .
```
### Run docker
Puis, dans la base du répertoire INF3995-Robot:
```sh
docker run --rm -p 5901:5901 -v  .:/root/INF3995-Robot -it simulation-docker bash -c "TVNC_WM=xfce /opt/TurboVNC/bin/vncserver -securitytypes None; /bin/bash"
```
### Connect to simulation
Enfin, se connecter par vnc (ex. RealVNC), à localhost:5901.

Et voilà!