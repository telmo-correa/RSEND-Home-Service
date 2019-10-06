#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(readlink -f ~/catkin_ws/src/map/map.yaml)" &
sleep 5
# Kinetic ROS does not support arbitrary bash expressions on launch file, so we pass the location as an environment variable
xterm -e "export NAVIGATION_WITH_MARKERS=$(readlink -f ~/catkin_ws/src/rvizConfig/navigation_with_markers.rviz); roslaunch $(find ~/catkin_ws/src/rvizConfig/view_navigation_with_markers.launch)" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
