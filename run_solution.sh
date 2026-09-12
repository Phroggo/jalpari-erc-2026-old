#!/bin/bash
# Launch the ERC solution inside the running erc_sim container.
#   ./run_solution.sh [shelf_column_number] [book_colour] [extra launch args...]
#   ./run_solution.sh 2 red
#   ROS_DOMAIN_ID=57 ./run_solution.sh 4 blue
set -euo pipefail
column="${1:-2}"
colour="${2:-red}"
if (($#)); then shift; fi
if (($#)); then shift; fi
case "$column" in [1-5]) ;; *) echo 'Column must be 1..5' >&2; exit 2;; esac
case "$colour" in red|green|blue|yellow) ;; *) echo 'Invalid book colour' >&2; exit 2;; esac
docker exec -i -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}" erc_sim /entrypoint.sh bash -lc '
  source /opt/erc_ws/install/setup.bash
  cd /opt/erc_ws
  exec ros2 launch erc_solution solution.launch.py "$@"
' bash "shelf_column_number:=$column" "book_colour:=$colour" "$@"
