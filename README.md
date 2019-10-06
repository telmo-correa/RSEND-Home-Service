# Home Service Robot

This repository is an implementation of the final project for the [Robotics Software Engineer Nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209).

It represents a toy implementation of a robot, responsible for performing simultaneous localization and mapping on a custom map, tasked with collecting an object at a specific location and dropping it at another location.  Implementation is done mostly using public ROS packages, and a couple of custom-purpose C++ packages.

This directory contains:
- A SDF model for a small apartment, its map, and visualization settings
- Clones for various ROS packages, responsible for SLAM
- A couple of ROS packages for collecting and dropping a virtual object at different locations


## References
This project utilizes the following ROS packages, cloned on October 5, 2019.

[gmapping](http://wiki.ros.org/gmapping)
[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

## Setup

The launch files in this repository rely on a Linux-like environment, ROS kinetic (or higher version), and xterm.  General setup instructions below:

```
# Install xterm
sudo apt-get install xterm

# Create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..

# Initial catkin workspace build
catkin_make
cd ~/catkin_ws/src

# Clone this repository and copy the contents to the catkin repository
git clone https://github.com/telmo-correa/RSE-Home-Service.git
cp -r RSE-Home-Service/* .
cd ~/catkin_ws/

# Ensure latest system packages are available
sudo apt-get update

# Install ROS package dependencies
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo

# Rebuild and source workspace
catkin_make
source devel/setup.bash
```

## Tasks

### SLAM Testing

The first part of this project consists of testing simultaneous localization and mapping (SLAM) by driving around a robot, and verifying it can find its own location -- and build a map as it goes along.

This is implemented on the script `test_slam.sh`.  Ensure it's executable and run it:

```
chmod u+x test_slam.sh
./test_slam.sh
```

The contents of the script are as follows:
```
#!/bin/sh
# The argument world_file has to be an absolute path
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"
```

This will open multiple xterm terminals, corresponding to the turtlebot environment on a custom world, the gmapping node, the RViz visualizer, and turtlebot teleop for keyboard control -- focus on the last terminal and use the keyboard to drive the robot around, and observe its SLAM on the RViz window, compared to its actual simulated location on Gazebo.


### Navigation and testing

In a similar setup, `test_navigation.sh` is responsible for demonstrating navigation using a pre-computed map of the enviroment (`map/map.pgm` and `map/map.yaml`).  Make the script executable and run it:

```
chmod u+x test_navigation.sh
./test_navigation.sh
```

Script contents:
```
#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(readlink -f ~/catkin_ws/src/map/map.yaml)" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
```

This will, again, open multiple xterm terminals, but this time it will use AMCL to navigate towards goals set via RVIz.  Explicitly set a new pose there and the robot will attempt to drive to that goal.


### Pick objects

The final script for this task is `pick_objects.sh`.  It will drive the robot to a specific waypoint, print a message, wait 5 seconds, and then drive to another location.  The intended behavior is a simulation of picking up an object and dropping it at another location.

```
chmod u+x pick_objects.sh
./pick_objects.sh
```

Script contents:
```
#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(readlink -f ~/catkin_ws/src/map/map.yaml)" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
```

### Add marker

The second-to-last task is `add_markers.sh`.  It is responsible for running a node that adds virtual objects -- a blue cube, in this implementation -- representing the object to be picked up and dropped off.  The node has been modified for the final task to keep the marker in location until the robot picks it up, so running this task will just add a blue cube that stays in place at the pick up location.

```
chmod u+x add_markers.sh
./add_markers.sh
```

Script contents:
```
#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(readlink -f ~/catkin_ws/src/map/map.yaml)" &
sleep 5
# Kinetic ROS does not support arbitrary bash expressions on launch file, so we pass the location as an environment variable
xterm -e "export NAVIGATION_WITH_MARKERS=$(readlink -f ~/catkin_ws/src/rvizConfig/navigation_with_markers.rviz); roslaunch $(find ~/catkin_ws/src/rvizConfig/view_navigation_with_markers.launch)" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
```

### Home service robot

The final task is at `home_service.sh`.  It combines the functionalities of the last two tasks: the robot drives to the pick up location, picks up the virtual object, then drives to the drop off location and drops the virtual object.

Rather than having the marker just appear and disappear, in this implementation the marker state is kept in the `add_markers` ROS node -- its location in X, Y coordinates and whether it is currently being carried by the robot.  The robot would be able to pick it up whenever it's close by and publishes a relevant message, and be dropped off whenever the robot messages a relevant message (at any location).  This functionality is implemented by having `add_markers` subscribe to the odometer topic -- always keeping a copy of the last known robot location -- and to a new topic, `pickup`, to which messages can be published to pick up the object (if the robot is nearby) or to drop it (anywhere).

```
chmod u+x home_service.sh
./home_service.sh
```

Script contents:
```
#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(readlink -f ~/catkin_ws/src/worlds/apartment.world)" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(readlink -f ~/catkin_ws/src/map/map.yaml)" &
sleep 5
# Kinetic ROS does not support arbitrary bash expressions on launch file, so we pass the location as an environment variable
xterm -e "export NAVIGATION_WITH_MARKERS=$(readlink -f ~/catkin_ws/src/rvizConfig/navigation_with_markers.rviz); roslaunch $(find ~/catkin_ws/src/rvizConfig/view_navigation_with_markers.launch)" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &
xterm -e "rosrun add_markers add_markers" &
```
