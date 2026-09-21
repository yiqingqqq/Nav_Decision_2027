#!/bin/bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 and this workspace regardless of the caller's current directory.
source /opt/ros/humble/setup.bash
source "${script_dir}/install/setup.bash"
set -u

if lsof -nP -iTCP:1667 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "ERROR: TCP port 1667 is already used by Groot or another behavior tree." >&2
  lsof -nP -iTCP:1667 -sTCP:LISTEN >&2 || true
  exit 1
fi

# Avoid accepting action names cached by the ROS graph daemon from an earlier run.
ros2 daemon stop >/dev/null 2>&1 || true

# Do not start the decision tree until Nav2 can accept navigation goals.
echo "Waiting for Nav2 action server /navigate_to_pose ..."
for _ in $(seq 1 120); do
  bt_state="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  controller_state="$(ros2 lifecycle get /controller_server 2>/dev/null || true)"
  navigate_info="$(ros2 action info /navigate_to_pose 2>/dev/null || true)"
  follow_info="$(ros2 action info /follow_path 2>/dev/null || true)"
  if [[ "${bt_state}" == *active* ]] && \
     [[ "${controller_state}" == *active* ]] && \
     [[ "${navigate_info}" == *"Action servers: 1"* ]] && \
     [[ "${follow_info}" == *"Action servers: 1"* ]]; then
    # FollowPath is limited to 1.4 m/s in the simulation profile. Keep the
    # tunnel controller at roughly 85% of that limit.
    ros2 param set /controller_server TunnelController.max_vel_x 1.20
    echo "Nav2 is ready. Starting RMUL 2027 tunnel behavior tree."
    exec ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
      style:=tunnel.xml \
      use_sim_time:=True
  fi
  sleep 1
done

echo "ERROR: /navigate_to_pose and /follow_path were not available after 120 seconds." >&2
echo "Start the navigation workspace first, then run publish.sh and run_tunnel.sh." >&2
exit 1
