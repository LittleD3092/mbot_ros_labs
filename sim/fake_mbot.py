#!/usr/bin/env python3
"""Fake MBot: stands in for the real robot + mbot_bringup so the exploration
stack (slam_node, navigation_node, controller_node, exploration_node) can run
unchanged on a laptop.

It drives a ground-truth robot around a saved maze map using /cmd_vel and
publishes what the real robot publishes:

  /odom                  nav_msgs/Odometry   odom -> base_footprint (with drift)
  /tf                    odom -> base_footprint
  /tf_static             base_footprint -> base_link -> lidar_link (same as the real URDF)
  /scan                  sensor_msgs/LaserScan in lidar_link, 1080 rays, 10 Hz

The real lidar is mounted rotated 180 deg (lidar_link yaw = pi) and its scan
angles are in that frame; the fake one does the same, so slam_node sees data
laid out exactly as on the robot.

Extra topics for checking your work (not on the real robot):

  /sim/true_pose         geometry_msgs/PoseStamped  ground truth, in SLAM's "map" frame
  /sim/true_path         nav_msgs/Path              ground-truth trajectory
  /sim/world_map         nav_msgs/OccupancyGrid     the real maze, in SLAM's "map" frame

SLAM's map frame starts where the robot starts, so the ground truth is
published relative to the start pose. If SLAM is working, /map lines up with
/sim/world_map and /sim/true_path with SLAM's /estimated_path.

Run (from the workspace root, inside the container):
  python3 sim/fake_mbot.py
  python3 sim/fake_mbot.py --ros-args -p map_yaml:=mbot_nav/maps/maze2.yaml
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import numpy as np
    import yaml  # noqa: F401  (used by World)
except ImportError as e:
    sys.exit(f'[fake_mbot] Missing Python module: {e.name}. '
             f'Install it with: sudo apt install python3-{e.name}')

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from fake_mbot_core import World, DiffDrive, wrap

DEFAULT_MAP = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'mbot_nav', 'maps', 'maze1.yaml'))

# Measured from the real robot (mbot_rosbags/slam_test, /tf_static and /scan).
BASE_LINK_Z = 0.042
LIDAR_Z = 0.14179
LIDAR_YAW = math.pi
SCAN_RAYS = 1080
RANGE_MIN, RANGE_MAX = 0.05, 12.0


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakeMBot(Node):
    def __init__(self):
        super().__init__('fake_mbot')

        p = self.declare_parameter
        map_yaml = p('map_yaml', DEFAULT_MAP).value
        start = (p('start_x', 0.0).value, p('start_y', 0.0).value, p('start_theta', 0.0).value)
        self.radius = p('robot_radius', 0.10).value
        self.range_noise = p('range_noise', 0.01).value          # m, 1-sigma
        self.dropout = p('scan_dropout', 0.03).value             # fraction of rays returned as inf
        odom_v_noise = p('odom_v_noise', 0.02).value             # relative, 1-sigma
        odom_w_noise = p('odom_w_noise', 0.02).value
        max_v = p('max_v', 0.8).value
        max_w = p('max_w', 4.0).value
        max_accel = p('max_accel', 1.5).value                    # m/s^2
        max_alpha = p('max_alpha', 8.0).value                    # rad/s^2
        self.cmd_timeout = p('cmd_timeout', 0.5).value           # s without /cmd_vel -> stop
        self.physics_hz = p('physics_rate', 50.0).value
        self.odom_every = max(1, round(self.physics_hz / p('odom_rate', 25.0).value))
        self.scan_every = max(1, round(self.physics_hz / p('scan_rate', 10.0).value))
        seed = p('seed', -1).value
        unknown_is_wall = p('unknown_is_wall', True).value

        if not os.path.isabs(map_yaml) and not os.path.exists(map_yaml):
            alt = os.path.join(os.path.dirname(DEFAULT_MAP), map_yaml)
            map_yaml = alt if os.path.exists(alt) else map_yaml
        if not os.path.exists(map_yaml):
            raise SystemExit(f'[fake_mbot] map_yaml not found: {map_yaml}')

        self.world = World(map_yaml, unknown_is_wall=unknown_is_wall)
        sx, sy, sth = start
        if not bool(self.world.is_free(sx, sy)) or self.world.collides(sx, sy, self.radius):
            raise SystemExit(
                f'[fake_mbot] Start pose ({sx:.2f}, {sy:.2f}) is not in free space with '
                f'{self.radius:.2f} m clearance in {map_yaml}. Pick another with '
                f'-p start_x:=... -p start_y:=... (map frame of that YAML).')

        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.robot = DiffDrive(sx, sy, sth, max_v, max_w, max_accel, max_alpha,
                               odom_v_noise, odom_w_noise, self.rng)
        self.start = start

        n = SCAN_RAYS
        self.angle_min, self.angle_max = -math.pi, math.pi
        self.angle_inc = (self.angle_max - self.angle_min) / (n - 1)
        self.scan_angles = self.angle_min + np.arange(n) * self.angle_inc

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.true_pose_pub = self.create_publisher(PoseStamped, '/sim/true_pose', 10)
        self.true_path_pub = self.create_publisher(Path, '/sim/true_path', 10)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.world_map_pub = self.create_publisher(OccupancyGrid, '/sim/world_map', latched)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.last_cmd_time = None

        self.true_path = Path()
        self.true_path.header.frame_id = 'map'
        self.step_count = 0
        self.last_step_wall = time.monotonic()
        self.bump_count = 0

        self.publish_static_tf()
        self.publish_world_map()
        self.timer = self.create_timer(1.0 / self.physics_hz, self.on_step)

        self.get_logger().info(
            f'Fake MBot up: map {os.path.relpath(map_yaml)}, start ({sx:.2f}, {sy:.2f}, '
            f'{math.degrees(sth):.0f} deg), radius {self.radius:.2f} m. Listening on /cmd_vel.')

    # ---------------------------------------------------------------- inputs
    def on_cmd_vel(self, msg):
        self.robot.set_cmd(msg.linear.x, msg.angular.z)
        self.last_cmd_time = time.monotonic()

    # ------------------------------------------------------------ simulation
    def on_step(self):
        now_wall = time.monotonic()
        dt = min(now_wall - self.last_step_wall, 0.1)  # don't jump after a stall
        self.last_step_wall = now_wall
        if dt <= 0.0:
            return

        if self.last_cmd_time is not None and now_wall - self.last_cmd_time > self.cmd_timeout:
            self.robot.set_cmd(0.0, 0.0)

        self.robot.step(dt, self.world, self.radius)
        if self.robot.bumped:
            self.bump_count += 1
            self.get_logger().warn(
                f'BUMP: robot hit a wall at true pose {self.true_in_start_frame_str()} '
                f'(total {self.bump_count} physics steps in contact). '
                f'Planner clearance or controller tracking is too tight.',
                throttle_duration_sec=1.0)

        self.step_count += 1
        stamp = self.get_clock().now().to_msg()
        do_scan = self.step_count % self.scan_every == 0
        if do_scan or self.step_count % self.odom_every == 0:
            self.publish_odom(stamp)
        if do_scan:
            # Scan is instantaneous at this stamp (time_increment = 0), so the
            # odom TF published just above is exactly what slam_node looks up.
            self.publish_scan(stamp)
            self.publish_truth(stamp)

    # --------------------------------------------------------------- outputs
    def publish_static_tf(self):
        stamp = self.get_clock().now().to_msg()
        tfs = []
        for parent, child, z, yaw in (('base_footprint', 'base_link', BASE_LINK_Z, 0.0),
                                      ('base_link', 'lidar_link', LIDAR_Z, LIDAR_YAW)):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.z = z
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = yaw_to_quat(yaw)
            tfs.append(t)
        self.static_tf_pub.sendTransform(tfs)

    def publish_odom(self, stamp):
        r = self.robot
        qx, qy, qz, qw = yaw_to_quat(r.oth)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = r.ox_
        t.transform.translation.y = r.oy_
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_pub.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = r.ox_
        o.pose.pose.position.y = r.oy_
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x = r.odom_v
        o.twist.twist.angular.z = r.odom_w
        self.odom_pub.publish(o)

    def publish_scan(self, stamp):
        r = self.robot
        world_angles = r.theta + LIDAR_YAW + self.scan_angles
        ranges = self.world.cast(r.x, r.y, world_angles, RANGE_MIN, RANGE_MAX)
        finite = np.isfinite(ranges)
        ranges[finite] += self.rng.normal(0.0, self.range_noise, int(finite.sum()))
        ranges[self.rng.random(len(ranges)) < self.dropout] = np.inf
        ranges[finite & (ranges < RANGE_MIN)] = np.inf

        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = 'lidar_link'
        s.angle_min = float(self.angle_min)
        s.angle_max = float(self.angle_max)
        s.angle_increment = float(self.angle_inc)
        s.time_increment = 0.0
        s.scan_time = float(self.scan_every / self.physics_hz)
        s.range_min = RANGE_MIN
        s.range_max = RANGE_MAX
        s.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(s)

    def true_in_start_frame(self):
        """Ground-truth pose expressed in the frame of the start pose (= SLAM's map frame)."""
        sx, sy, sth = self.start
        dx, dy = self.robot.x - sx, self.robot.y - sy
        c, s = math.cos(-sth), math.sin(-sth)
        return c * dx - s * dy, s * dx + c * dy, wrap(self.robot.theta - sth)

    def true_in_start_frame_str(self):
        x, y, th = self.true_in_start_frame()
        return f'({x:.2f}, {y:.2f}, {math.degrees(th):.0f} deg)'

    def publish_truth(self, stamp):
        x, y, th = self.true_in_start_frame()
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = x, y
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = yaw_to_quat(th)
        self.true_pose_pub.publish(ps)

        last = self.true_path.poses[-1].pose.position if self.true_path.poses else None
        if last is None or math.hypot(x - last.x, y - last.y) > 0.02:
            self.true_path.poses.append(ps)
        self.true_path.header.stamp = stamp
        self.true_path_pub.publish(self.true_path)

    def publish_world_map(self):
        w = self.world
        sx, sy, sth = self.start
        c, s = math.cos(-sth), math.sin(-sth)
        dx, dy = w.ox - sx, w.oy - sy

        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'map'
        g.info.resolution = float(w.res)
        g.info.width, g.info.height = int(w.w), int(w.h)
        g.info.origin.position.x = c * dx - s * dy
        g.info.origin.position.y = s * dx + c * dy
        (g.info.origin.orientation.x, g.info.origin.orientation.y,
         g.info.origin.orientation.z, g.info.origin.orientation.w) = yaw_to_quat(-sth)
        data = np.full(w.occupied.shape, -1, dtype=np.int8)
        data[w.free] = 0
        data[w.occupied] = 100
        g.data = data.ravel().tolist()
        self.world_map_pub.publish(g)


def main():
    rclpy.init()
    try:
        node = FakeMBot()
    except SystemExit as e:
        print(e, file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C can invalidate the context while a timer callback is mid-publish.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
