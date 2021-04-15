#!/bin/sh
xterm  -e  " 
export ROBOT_INITIAL_POSE='-x 15 -y 6 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../../src/map/myworld.world " &

sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=$(pwd)/../../src/map/mymap.yaml" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun add_markers add_markers "&
sleep 5
xterm  -e  " rosrun pick_objects pick_objects "
