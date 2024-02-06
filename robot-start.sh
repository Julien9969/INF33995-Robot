docker pull lagrangeluo/limo_ros2:v1

docker run --network=host \
    -d
    -v=/dev:/dev \
    --privileged \
    --device-cgroup-rule="a *:* rmw" \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    --runtime nvidia
    --gpus=all \
    -w=/workspace \
    --name limo_dev \
    -e LIBGL_ALWAYS_SOFTWARE="1" \
    -e DISPLAY=${DISPLAY} \
    --restart=always \
    -v ~/agx_workspace:/home/limo_ros1_ws \
    lagrangeluo/limo_ros2:v1 \
