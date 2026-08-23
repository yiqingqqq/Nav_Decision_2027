#!/bin/bash

# Source ROS2 workspace
source ./install/setup.bash

# Publish game progress at 1 Hz 
ros2 topic pub -r 1 /feedback_game_progress std_msgs/msg/UInt8 "{
    data: 4
}" &

# Publish robot hp at 3 Hz
ros2 topic pub -r 3 /feedback_robot_hp std_msgs/msg/UInt16 "{
    data: 400
}" &

# Wait for all background processes
wait
