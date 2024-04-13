FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt upgrade -y
# INSTALL UTILS
RUN apt update && apt install lsb-release wget curl gnupg python3-pip git -y

# OPENGL/MESA UTILS
RUN apt install mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev -y


# INSTALL IGNITION GAZEBO
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt update && apt install ignition-fortress -y

# INSTALL ROS2
RUN apt install software-properties-common -y
RUN pip3 install vcstool colcon-common-extensions
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install ros-humble-desktop -y

# INSTALL MISSING PACKAGES
RUN apt-get clean 
RUN apt update && apt install python3-rosdep2 -y
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt install python3-colcon-common-extensions -y

RUN apt update && apt install -y vim
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "export GZ_VERSION=fortress" >> /root/.bashrc

# DISPLAY STUFF
RUN wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | gpg --dearmor >/etc/apt/trusted.gpg.d/TurboVNC.gpg
RUN echo "deb [signed-by=/etc/apt/trusted.gpg.d/TurboVNC.gpg] https://packagecloud.io/dcommander/turbovnc/any/ any main" | tee /etc/apt/sources.list.d/turbovnc.list > /dev/null

RUN apt update; apt install turbovnc -y
RUN apt install xfce4 terminator -y
RUN update-alternatives --set x-terminal-emulator /usr/bin/terminator

# INSTALL LIMO'S GAZEBO FILES AND STUFF
WORKDIR /root
RUN mkdir /root/INF3995-Robot/ros_ws -p
RUN export TERM=xterm-256color
RUN echo "export ROS_DOMAIN_ID=62" >> /root/.bashrc
# DISPLAY STUFF
#RUN xauth add pop-os/unix:1 MIT-MAGIC-COOKIE-1 1a8a086b0cad131f229f8fe587d633ae
#VOLUME /tmp/.X11-unix:/tmp/.X11-unix

# extra commands pour nos dépendances
RUN apt update && apt install ros-humble-nav2* ros-humble-slam-toolbox -y

COPY deploy-simulation.sh .
RUN chmod +x /root/deploy-simulation.sh
COPY start-files-server.sh .
RUN chmod +x /root/start-files-server.sh
RUN echo "export ROBOT_ENV=SIMULATION" >> /root/.bashrc