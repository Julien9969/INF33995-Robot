#!/bin/bash

apt update && apt upgrade -y
# INSTALL UTILS
apt update && apt install lsb-release wget curl gnupg python3-pip git -y

# OPENGL/MESA UTILS
apt update && apt install mesa-utils libglu1-mesa-dev freeglut3-dev mesa-common-dev -y


# INSTALL IGNITION GAZEBO
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt update && apt install ignition-fortress -y


# INSTALL ROS2
apt update && apt install software-properties-common -y
su -c 'pip3 install vcstool colcon-common-extensions' nvidia
add-apt-repository universe

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update && apt install ros-humble-desktop -y

apt-get clean
apt update && apt install python3-rosdep2 -y
sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt install python3-colcon-common-extensions -y

apt update && apt install -y vim
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
echo "export GZ_VERSION=fortress" >> /root/.bashrc
echo "export ROS_DOMAIN_ID=62" >> /root/.bashrc