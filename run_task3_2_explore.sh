#!/usr/bin/env bash
# MBot ROS 2 navigation and exploration stack.
# Usage:
#   ./run_task3_2_explore.sh [maze1|maze2]
#   ./run_task3_2_explore.sh stop|estop

set -Eeuo pipefail

WS="${HOME}/mbot_ros_labs"
MAP="${1:-maze1}"
SESSION="mbot"

die()  { echo -e "\n[ERROR] $*" >&2; exit 1; }
info() { echo "[MBOT] $*"; }

# --------------------------------------------------
# Emergency stop
# --------------------------------------------------
if [[ "$MAP" == "stop" || "$MAP" == "estop" ]]; then
    command -v tmux >/dev/null || die "tmux unavailable."
    tmux has-session -t "$SESSION" 2>/dev/null || { info "No active MBot session."; exit 0; }

    if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx Controller; then
        info "Stopping controller..."
        tmux send-keys -t "$SESSION:Controller" C-c
        sleep 1
    fi

    tmux kill-session -t "$SESSION"
    printf "\n[MBOT] Software stop completed.\n[MBOT] CHECK THAT ROBOT HAS PHYSICALLY STOPPED. Use physical e-stop if moving.\n\n"
    exit 0
fi

# --------------------------------------------------
# Pre-flight checks
# --------------------------------------------------
[[ "$MAP" =~ ^[a-zA-Z0-9_-]+$ ]]           || die "Invalid map name: $MAP"
[[ -d "$WS/src/mbot_nav" ]]                || die "Missing $WS/src/mbot_nav"
[[ -d "$WS/src/mbot_slam" ]]               || die "Missing $WS/src/mbot_slam"
[[ -f "$WS/src/mbot_nav/maps/$MAP.yaml" ]] || die "Map not found: $MAP"
[[ -t 0 ]]                                 || die "Run from an interactive terminal."

for cmd in tmux colcon; do
    command -v "$cmd" >/dev/null || die "$cmd is not installed."
done

tmux has-session -t "$SESSION" 2>/dev/null && die "Session '$SESSION' already running. Stop with: $0 stop"

# --------------------------------------------------
# Source ROS 2 & Build
# --------------------------------------------------
set +u
if [[ -z "${ROS_DISTRO:-}" ]]; then
    installs=(/opt/ros/*)
    [[ ${#installs[@]} -eq 1 && -f "${installs[0]}/setup.bash" ]] || die "Cannot determine ROS_DISTRO."
    source "${installs[0]}/setup.bash"
fi

ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
[[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
source "$ROS_SETUP"
command -v ros2 >/dev/null || die "ROS 2 not found."

info "ROS: $ROS_DISTRO | Map: $MAP"
info "Building packages..."
cd "$WS"
colcon build --symlink-install --packages-up-to mbot_nav mbot_slam

SETUP="$WS/install/setup.bash"
[[ -f "$SETUP" ]] || die "Workspace setup file not created."
source "$SETUP"
set -u

# --------------------------------------------------
# Verify packages & executables
# --------------------------------------------------
pkgs=(mbot_bringup mbot_nav mbot_slam foxglove_bridge)
for p in "${pkgs[@]}"; do
    ros2 pkg prefix "$p" >/dev/null 2>&1 || die "Package '$p' not found."
done

check_exec() { ros2 pkg executables "$1" | grep -q "$2" || die "Executable '$2' not found in '$1'."; }
check_exec mbot_slam slam_node
check_exec mbot_nav navigation_node
check_exec mbot_nav controller_node
check_exec mbot_nav exploration_node

# --------------------------------------------------
# Launch stack in tmux
# --------------------------------------------------
start_window() {
    local name="$1" cmd="$2" delay="${3:-0}"
    local full_cmd="cd '$WS' && source '$ROS_SETUP' && source '$SETUP' && $cmd; echo -e '\n$name exited with status '\$?; exec bash"

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux new-window -t "$SESSION" -n "$name" bash -c "$full_cmd"
    else
        tmux new-session -d -s "$SESSION" -n "$name" bash -c "$full_cmd"
    fi
    sleep "$delay"
}

info "Starting Bringup, Foxglove, SLAM, and Navigation..."
start_window Bringup "ros2 launch mbot_bringup mbot_bringup.launch.py" 5

start_window Foxglove "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" 2

start_window SLAM "ros2 run mbot_slam slam_node" 3
start_window Navigation "ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf" 3

# --------------------------------------------------
# Verification prompt
# --------------------------------------------------
clear
cat <<EOF
==================================================
         MBOT NAVIGATION STACK (FOXGLOVE)
==================================================
Map: $MAP | ROS: $ROS_DISTRO | tmux: $SESSION

Started: Bringup, Foxglove, SLAM, Navigation (TF mode)

Foxglove: Connect to ws://localhost:8765

Verify before starting controller:
  [ ] Robot model & TF tree visible
  [ ] LiDAR publishing on /scan
  [ ] SLAM publishing pose (map -> base_footprint)

Emergency stop: $0 stop
==================================================
EOF

read -r -p "Press ENTER when SLAM & navigation are ready (Ctrl+C to abort): "

# --------------------------------------------------
# Start controller & exploration
# --------------------------------------------------
info "Starting motion controller..."
start_window Controller "ros2 run mbot_nav controller_node --ros-args -p use_localization:=true"

info "Starting exploration node..."
start_window Exploration "ros2 run mbot_nav exploration_node"

cat <<EOF

==================================================
          MBOT STACK FULLY STARTED
==================================================
Running: Bringup, Foxglove (ws://localhost:8765), SLAM, Navigation, Controller, Exploration

Attach: tmux attach -t $SESSION  |  Switch windows: Ctrl+b, <number>
Stop:   $0 stop
==================================================
EOF