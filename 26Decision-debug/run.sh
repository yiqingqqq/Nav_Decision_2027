#!/bin/bash

# Source ROS2 workspace
source ./install/setup.bash

# Launch the behavior tree
ros2 launch rm_behavior_tree rm_behavior_tree.launch.py style:=v2.xml use_sim_time:=False