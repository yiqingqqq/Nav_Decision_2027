#!/bin/bash

# Source ROS2 workspace
source ./install/setup.bash

# Launch the ordinary decision behavior tree.  Tunnel mode is intentionally
# separate: after Nav2 is ready, start it with ./run_tunnel.sh.
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py style:=v2.xml use_sim_time:=False
