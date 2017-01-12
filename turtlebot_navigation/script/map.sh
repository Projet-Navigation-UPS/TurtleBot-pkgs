#!/bin/bash

rosrun map_server map_server ../map/aip_hall.yaml &
roslaunch turtlebot_navigation localisation.launch &



