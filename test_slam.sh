#!/bin/sh
# The argument world_file has to be an absolute path
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
