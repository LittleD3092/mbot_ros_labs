#!/usr/bin/env bash
# Start the MBot navigation stack in separate terminal windows using Foxglove.
# Usage:
#   ./run_mbot_stack.sh             # uses final_map
#   ./run_mbot_stack.sh maze1       # uses maze1
#
# Requires a graphical Ubuntu session with gnome-terminal.
# Foxglove Studio must be open separately in a browser/desktop app.

set -Eeuo pipefail

WORKSPACE="${HOME}/mbot_ros_labs"
MAP_NAME="${1:-final_map}"

if [[ ! -d "${WORKSPACE}/src/mbot_nav" ]]; then
  echo "ERROR: Cannot find ${WORKSPACE}/src/mbot_nav"
  echo "Edit WORKSPACE near the top of this script if your workspace is elsewhere."
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "ERROR: gnome-terminal was not found."
  echo "This script opens one terminal window per ROS process and requires gnome-terminal."
  exit 1
fi

cd "${WORKSPACE}"

echo "Building mbot_nav and mbot_slam..."
colcon build --packages-select mbot_nav mbot_slam

SETUP="${WORKSPACE}/install/setup.bash"
if [[ ! -f "${SETUP}" ]]; then
  echo "ERROR: ${SETUP} was not created."
  exit 1
fi

open_ros_terminal() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="${title}" -- bash -lc \
    "cd '${WORKSPACE}' && source '${SETUP}' && ${command}; status=\$?; echo; echo '${title} exited with status' \$status; exec bash"
}

echo "Starting robot bringup..."
open_ros_terminal "MBot 1 - Bringup" \
  "ros2 launch mbot_bringup mbot_bringup.launch.py"

sleep 3
echo "Starting map and navigation..."
open_ros_terminal "MBot 2 - Navigation" \
  "ros2 launch mbot_nav path_planning.launch.py map_name:='${MAP_NAME}' pose_source:=tf"

sleep 3
echo "Starting localization..."
open_ros_terminal "MBot 3 - Localization" \
  "ros2 run mbot_slam localization_node --ros-args -p publish_tf:=true"

sleep 3
echo "Starting Foxglove bridge..."
open_ros_terminal "MBot 4 - Foxglove Bridge" \
  "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

echo
echo "The four ROS terminals have been opened."
echo "Now open Foxglove Studio and connect to ws://localhost:8765."
echo "Add panels for the map (/map), TF, LaserScan (/scan), and relevant navigation topics."
echo "Use Foxglove's Publish panel (or another ROS 2 publisher) to publish the initial pose"
echo "to /initialpose if your localization node requires it."
echo "Only continue once localization is initialized and the map->base_footprint TF exists."
read -r -p "Press ENTER to start the motion controller (or Ctrl+C to stop here)... "

echo "Starting motion controller..."
open_ros_terminal "MBot 5 - Controller" \
  "ros2 run mbot_nav controller_node --ros-args -p use_localization:=true"

echo
echo "All five ROS processes have been launched in separate terminal windows."
echo "Keep all windows open. To stop the robot, stop the controller first (Ctrl+C in its window)."