#!/usr/bin/env bash
# MBot ROS 2 navigation stack, run in tmux windows.
# Usage: ./run_task3_1_plan.sh [maze1|maze2]
#        ./run_task3_1_plan.sh stop|estop
# Open Foxglove Studio separately and connect to ws://localhost:8765

set -Eeuo pipefail

WS="${HOME}/mbot_ros_labs"
MAP="${1:-maze1}"
SESSION="mbot"
MAP_DIR="$WS/src/mbot_nav/maps"

die()  { echo -e "\n[ERROR] $*" >&2; exit 1; }
info() { echo "[MBOT] $*"; }

# ROS setup scripts reference unset variables, so source them with -u disabled.
source_safe() { set +u; source "$1"; set -u; }

# --- Emergency stop (runs before any startup checks) ---
if [[ "$MAP" == "stop" || "$MAP" == "estop" ]]; then
    command -v tmux >/dev/null || die "tmux is unavailable."
    tmux has-session -t "$SESSION" 2>/dev/null || { info "No active MBot session."; exit 0; }

    if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx Controller; then
        info "Stopping controller..."
        tmux send-keys -t "$SESSION:Controller" C-c
        sleep 1
    fi
    tmux kill-session -t "$SESSION"

    cat <<EOF

[MBOT] Software stop completed. CHECK THAT THE ROBOT HAS PHYSICALLY STOPPED.
[MBOT] If it keeps moving, use the physical emergency-stop switch immediately.
EOF
    exit 0
fi

# --- Checks ---
[[ "$MAP" =~ ^[a-zA-Z0-9_-]+$ ]] || die "Invalid map name: $MAP"
[[ -d "$WS/src/mbot_nav" ]]      || die "Missing $WS/src/mbot_nav"
[[ -d "$WS/src/mbot_slam" ]]     || die "Missing $WS/src/mbot_slam"
if [[ ! -f "$MAP_DIR/$MAP.yaml" ]]; then
    avail=$(cd "$MAP_DIR" && ls *.yaml 2>/dev/null | sed 's/\.yaml$//' | paste -sd' ' -)
    die "Map not found: $MAP (available: ${avail:-none})"
fi
command -v tmux >/dev/null   || die "tmux not installed. Run: sudo apt install tmux"
command -v colcon >/dev/null || die "colcon is not installed."
[[ -t 0 ]]                   || die "Run from an interactive terminal."
tmux has-session -t "$SESSION" 2>/dev/null &&
    die "An MBot session is already running. Stop it with: $0 stop (or: tmux attach -t $SESSION)"

# --- Source ROS 2 ---
if [[ -z "${ROS_DISTRO:-}" ]]; then
    installs=(/opt/ros/*)
    [[ ${#installs[@]} -eq 1 && -f "${installs[0]}/setup.bash" ]] || die "Cannot determine ROS_DISTRO."
    source_safe "${installs[0]}/setup.bash"
fi
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
[[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
source_safe "$ROS_SETUP"
command -v ros2 >/dev/null || die "ROS 2 not found."
info "ROS: $ROS_DISTRO, map: $MAP"

# --- Build (only the two packages in src/) ---
cd "$WS"
info "Building..."
set +u
colcon build --symlink-install --packages-up-to mbot_nav mbot_slam
set -u
SETUP="$WS/install/setup.bash"
[[ -f "$SETUP" ]] || die "Workspace setup file was not created."
source_safe "$SETUP"

# --- Verify packages and executables ---
# mbot_bringup and foxglove_bridge are not in src/, so they must be installed system-wide.
ros2 pkg prefix mbot_bringup >/dev/null 2>&1 ||
    die "mbot_bringup not found (it is not in $WS/src). Install or source the package that provides it."
ros2 pkg prefix foxglove_bridge >/dev/null 2>&1 ||
    die "foxglove_bridge not found. Run: sudo apt install ros-$ROS_DISTRO-foxglove-bridge"
ros2 pkg executables mbot_slam | grep -q localization_node || die "localization_node not found."
ros2 pkg executables mbot_nav  | grep -q controller_node   || die "controller_node not found."

# --- tmux helper: start_window <name> <command> [delay] ---
# The first call creates the session; later calls add windows.
start_window() {
    local name="$1" cmd="$2"
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux new-window -t "$SESSION" -n "$name"
    else
        tmux new-session -d -s "$SESSION" -n "$name"
    fi
    tmux send-keys -t "$SESSION:$name" \
        "cd '$WS'; source '$ROS_SETUP'; source '$SETUP'; $cmd; rc=\$?; echo; echo '$name exited with status' \$rc; exec bash" C-m
    sleep "${3:-0}"
}

info "Starting bringup, navigation, localization, Foxglove..."
start_window Bringup      "ros2 launch mbot_bringup mbot_bringup.launch.py" 5
start_window Navigation   "ros2 launch mbot_nav path_planning.launch.py map_name:=$MAP pose_source:=tf" 5
start_window Localization "ros2 run mbot_slam localization_node --ros-args -p publish_tf:=true" 3
start_window Foxglove     "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" 2

clear
cat <<EOF
==================================================
 MBOT NAVIGATION STACK STARTED
==================================================
Map: $MAP | ROS: $ROS_DISTRO | tmux session: $SESSION
Running: Bringup, Navigation, Localization, Foxglove bridge

Open Foxglove Studio and connect to: ws://localhost:8765

Before starting the controller, verify:
  [ ] Map on /map          [ ] LiDAR on /scan
  [ ] TF tree publishing   [ ] Localization initialized
  [ ] map -> base_footprint available
  [ ] Initial pose published to /initialpose if needed

Emergency stop (from another terminal):
  $0 stop   (or: estop)
==================================================
EOF

read -r -p "Press ENTER only when localization is ready and initial pose is set. Ctrl+C to skip: "

info "Starting motion controller..."
start_window Controller "ros2 run mbot_nav controller_node --ros-args -p use_localization:=true"

cat <<EOF

Stack fully started.
Attach to all processes: tmux attach -t $SESSION
Emergency stop:          $0 stop
EOF