#!/bin/bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 and this workspace regardless of the caller's current directory.
source /opt/ros/humble/setup.bash
source "${script_dir}/install/setup.bash"
set -u

# Do not start the decision tree until Nav2 can accept navigation goals.
echo "Waiting for Nav2 action server /navigate_to_pose ..."
for _ in $(seq 1 120); do
  lifecycle_state="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  if [[ "${lifecycle_state}" == *active* ]] && \
     ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then
    echo "Nav2 is ready. Starting RMUL 2027 tunnel behavior tree."
    exec ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
      style:=tunnel.xml \
      use_sim_time:=False
  fi
  sleep 1
done

echo "ERROR: /navigate_to_pose was not available after 120 seconds." >&2
echo "Start the navigation workspace first, or use ./run_tunnel_sim.sh for one-command RViz acceptance." >&2
exit 1
