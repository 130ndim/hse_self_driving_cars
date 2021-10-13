#!/bin/bash

#source "/opt/ros/noetic/setup.bash"

#roscore &
#rosrun turtlesim turtlesim_node __name:=runner &
#rosservice call /spawn "{x: 3.0, y: 5.0, theta: 3.14, name: 'chaser'}"
#
#rosrun turtlesim turtle_teleop_key


source "/workspace/devel/setup.bash"
roslaunch turtle_chaser chase.launch

exec "$@"
