#!/bin/bash

# Source ROS2 workspace
source ./install/setup.bash

# Launch the RMUC 2026 behavior tree. Override these values after field calibration.
RMUC_ALLIANCE="${RMUC_ALLIANCE:-red}"
RMUC_PRESET="${RMUC_PRESET:-balanced}"
RMUC_FIELD_ORIGIN_X="${RMUC_FIELD_ORIGIN_X:-0.0}"
RMUC_FIELD_ORIGIN_Y="${RMUC_FIELD_ORIGIN_Y:-0.0}"
RMUC_FIELD_ORIGIN_YAW="${RMUC_FIELD_ORIGIN_YAW:-0.0}"

ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
  style:=rmuc_2026.xml \
  preset:="$RMUC_PRESET" \
  alliance:="$RMUC_ALLIANCE" \
  field_origin_x:="$RMUC_FIELD_ORIGIN_X" \
  field_origin_y:="$RMUC_FIELD_ORIGIN_Y" \
  field_origin_yaw:="$RMUC_FIELD_ORIGIN_YAW" \
  use_sim_time:=False
