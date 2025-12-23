xhost +local:docker
sudo docker run -it --rm \
    -e USER=$USER \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./xarm_ros2:/home/$USER/xarm_ros2 \
    --device /dev/dri:/dev/dri \
    --name xarm_ros2 \
    xarm_ros2:jazzy
