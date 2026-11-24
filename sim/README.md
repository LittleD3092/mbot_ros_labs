# Fake MBot: test Checkpoint 3.2 on the laptop

`fake_mbot.py` stands in for the real robot and `mbot_bringup`. It drives a
simulated MBot around a saved maze map using `/cmd_vel`, and publishes the
same `/odom`, `/tf`, `/tf_static` and `/scan` as the real robot. The scan
matches the real one: 1080 rays at 10 Hz in `lidar_link`, with the lidar
mounted backwards (yaw = π). Your `slam_node`, `navigation_node`,
`controller_node` and `exploration_node` run unchanged.

## Quick start (from the host)

```bash
make sim-explore                 # maze1, RViz, full exploration stack
make sim-explore SIM_MAP=maze2   # another map from mbot_nav/maps
make shell                       # another terminal in the same container
```

The sim container uses ROS domain 77, local-only discovery and no static
peer, so nothing in it can reach the real robot, even if it's on the network.

## One node per terminal (closer to the checkpoint instructions)

```bash
make sim                         # terminal 1: opens a shell in the sim container
python3 sim/fake_mbot.py         # (in terminal 1)

make shell                       # terminals 2..5, one command each
ros2 run mbot_slam slam_node
ros2 run mbot_nav navigation_node --ros-args -p pose_source:=tf
ros2 run mbot_nav controller_node --ros-args -p use_localization:=true
ros2 run mbot_nav exploration_node
ros2 run rviz2 rviz2 -d mbot_nav/rviz/path_planning.rviz
```

Start `exploration_node` last, after SLAM has published a first `/map`.
To test the planner by itself, skip it and set goals with RViz's 2D Goal Pose.

## Ground truth (sim only)

| Topic | What |
|---|---|
| `/sim/world_map` | the real maze, in SLAM's `map` frame |
| `/sim/true_path` | where the robot really went |
| `/sim/true_pose` | where the robot really is |

Add these in RViz. If SLAM is working, `/map` lines up with `/sim/world_map`.
If a `BUMP:` warning appears in the fake robot's log, the robot touched a
wall: the planner's clearance or the controller's tracking is too tight.

## Parameters (`-p name:=value`)

| Name | Default | |
|---|---|---|
| `map_yaml` | `mbot_nav/maps/maze1.yaml` | world to simulate |
| `start_x`, `start_y`, `start_theta` | `0, 0, 0` | start pose, in that map's frame |
| `robot_radius` | `0.10` | collision disc, m |
| `odom_v_noise`, `odom_w_noise` | `0.02`, `0.02` | relative odometry error |
| `range_noise` | `0.01` | lidar noise, m (1σ) |
| `seed` | `-1` | fixed seed = repeatable run |
| `max_v`, `max_w` | `0.8`, `4.0` | velocity limits |

## What it doesn't model

Wheel slip on the real floor, lidar motion distortion (scans are
instantaneous, `time_increment = 0`), the MBot's real motor response, and
anything that isn't in the saved map (people, chair legs). Passing in sim
means the logic works. It doesn't guarantee the tuning works on the real
robot.
