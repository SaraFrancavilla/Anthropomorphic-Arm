#!/usr/bin/env bash

docker run --rm -d -p 6081:80 -p 50000-50020:50000-50020 --security-opt seccomp=unconfined --shm-size=512m --net ursim_net --ip 192.168.56.200 --name ros2 pla10/ros2_ur5_interface


# To connect to the real robot or to the docket simulator, you need to run the following command:
# docker run --rm -d -p 6081:80 --security-opt seccomp=unconfined --shm-size=512m --net ursim_net --ip 192.168.56.200 --name ros2 pla10/ros2_ur5_interface

# If you want to mount a volume to the container to work with your code, you can use the following option:
# -v /HOST/PATH/my_code:/home/ubuntu/ros2_ws/src/my_package 
