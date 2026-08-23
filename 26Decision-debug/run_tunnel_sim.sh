#!/bin/bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nav_workspace="$(cd "${script_dir}/../26Navigation-master" && pwd)"
child_pids=()

cleanup()
{
  trap - EXIT INT TERM
  for pid in "${child_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  wait "${child_pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
source "${nav_workspace}/install/setup.bash"
source "${script_dir}/install/setup.bash"
set -u

if pgrep -x gzserver >/dev/null || pgrep -f '/component_container_mt([[:space:]]|$)' >/dev/null; then
  echo "ERROR: an existing Gazebo or Nav2 instance is running. Stop it before using this standalone acceptance script." >&2
  exit 1
fi

if lsof -nP -iTCP:1667 -sTCP:LISTEN >/dev/null 2>&1; then
  echo "ERROR: TCP port 1667 is already in use; stop the existing Groot/behavior-tree process first." >&2
  lsof -nP -iTCP:1667 -sTCP:LISTEN >&2 || true
  exit 1
fi

# Remove stale graph information from an earlier ROS run before checking readiness.
ros2 daemon stop >/dev/null 2>&1 || true

echo "Starting RMUL2027 simulation, online mapping, Nav2 and RViz ..."
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL2027 \
  mode:=nav \
  lio:=fastlio \
  localization:=slam_toolbox \
  lio_rviz:=False \
  nav_rviz:=True &
child_pids+=("$!")
nav_pid="${child_pids[0]}"

echo "Waiting for Gazebo and ensuring physics is running ..."
clock_ready=false
for _ in $(seq 1 30); do
  gz world -p 0 >/dev/null 2>&1 || true
  if timeout 1 ros2 topic echo /clock --once >/dev/null 2>&1; then
    clock_ready=true
    break
  fi
  sleep 1
done

if [[ "${clock_ready}" != true ]]; then
  echo "ERROR: Gazebo clock did not start after 30 seconds." >&2
  exit 1
fi

echo "Waiting for Nav2 action server /navigate_to_pose ..."
nav_ready=false
sleep 5
for _ in $(seq 1 120); do
  if ! kill -0 "${nav_pid}" 2>/dev/null; then
    echo "ERROR: navigation launch exited before Nav2 became ready." >&2
    exit 1
  fi
  lifecycle_state="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  if [[ "${lifecycle_state}" == *active* ]] && \
     ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose' && \
     timeout 1 ros2 topic echo /map --once >/dev/null 2>&1; then
    nav_ready=true
    break
  fi
  sleep 1
done

if [[ "${nav_ready}" != true ]]; then
  echo "ERROR: /navigate_to_pose was not available after 120 seconds." >&2
  exit 1
fi

select_controller()
{
  local controller_id="$1"
  ros2 topic pub --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /controller_selector std_msgs/msg/String "{data: ${controller_id}}"
}

navigate_to()
{
  local x="$1"
  local y="$2"
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
}

echo "Nav2 and /map are ready. Driving from the red field-center case to the nearer tunnel endpoint."
select_controller FollowPath
navigate_to 0.0 0.0
navigate_to 1.0 0.6

echo "Switching to TunnelController and driving directly through the tunnel."
select_controller TunnelController
navigate_to 2.5 0.6
navigate_to 4.5 0.6

echo "Tunnel acceptance route finished. Restoring FollowPath."
select_controller FollowPath
echo "Reached the opposite side of the tunnel."
echo "RViz will remain open for inspection. Press Ctrl+C to stop the complete simulation."
wait "${nav_pid}"
