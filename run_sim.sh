#!/bin/bash
# Launch the ERC base simulation inside the running erc_sim container.
#   ./run_sim.sh              # GUI, no depth-cloud node (lighter)
#   ./run_sim.sh full         # GUI + depth-cloud point cloud
#   ./run_sim.sh headless     # no GUI (fastest; camera still publishes via EGL)
set -euo pipefail
mode="${1:-lite}"
args=(depth_cloud:=false)
case "$mode" in
  lite) ;;
  full)     args=(depth_cloud:=true) ;;
  headless) args=(headless:=true depth_cloud:=false) ;;
  *) echo 'Usage: ./run_sim.sh [lite|full|headless]' >&2; exit 2 ;;
esac
if [[ "$mode" != headless ]]; then
  xhost +si:localuser:root >/dev/null 2>&1 || true
  docker exec -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" erc_sim /entrypoint.sh bash -lc '
    source /opt/erc_ws/install/setup.bash
    exec ros2 run erc_solution recording_view
  ' &
  viewer_pid=$!
  trap 'kill "$viewer_pid" 2>/dev/null || true' EXIT
fi
docker exec -i -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" erc_sim /entrypoint.sh bash -lc '
  source /opt/erc_ws/install/setup.bash
  cd /opt/erc_ws
  exec ros2 launch erc_bringup simulation.launch.py "$@"
' bash "${args[@]}"
