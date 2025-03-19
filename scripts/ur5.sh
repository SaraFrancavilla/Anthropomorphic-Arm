#!/usr/bin/env bash

docker run --rm -d --net ursim_net --ip 192.168.56.101 -e ROBOT_MODEL=UR5 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999 --name ursim pla10/ursim_e-series