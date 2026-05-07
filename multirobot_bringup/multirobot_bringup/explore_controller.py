"""Per-robot controller with artificial-potential-field obstacle avoidance.

Subscribes to:
  /<ns>/scan       sensor_msgs/LaserScan  (obstacle sensing)
  /<ns>/odom       nav_msgs/Odometry      (localization in odom frame)
  /<ns>/goal_pose  geometry_msgs/PoseStamped (goal in <ns>/odom frame)

Publishes to:
  /<ns>/cmd_vel    geometry_msgs/Twist     (velocity command)
  /<ns>/at_goal    std_msgs/Bool (latched) (true once arrived; false on new goal)

Algorithm:
  Each tick, build two force vectors in the world (odom) frame:
    - Attractive  : unit vector from robot toward goal.
    - Repulsive   : sum over lidar returns within repulse_radius. Each ray
                    contributes a vector pointing AWAY from that return
                    with magnitude (1/d - 1/repulse_radius), zeroing at
                    the boundary and growing as obstacles get closer.
  Combined heading = k_attract * F_att + k_repulse * F_rep.
  Steer toward that heading; forward speed = max_lin * alignment *
  min(1, |F|), so the robot slows when turning sharply or fighting a
  strong repulsion.

  Local-minima case (attraction and repulsion cancel) → robot stops in
  place. The explorer's stuck-timeout abandons the goal and picks
  another, which moves the attractor and breaks the deadlock.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def _yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ExploreController(Node):
    def __init__(self):
        super().__init__('explore_controller')

        self.declare_parameter('namespace', 'robot1')
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('goal_tolerance', 0.30)
        # Potential-field tunables.
        self.declare_parameter('repulse_radius', 1.20)
        self.declare_parameter('k_attract', 1.0)
        self.declare_parameter('k_repulse', 0.60)
        self.declare_parameter('control_rate_hz', 10.0)
        # Peer repulsion: every controller subscribes to peer robots' /odom
        # and adds a repulsive force when peers come within
        # peer_repulse_radius meters. More reliable than lidar-only
        # avoidance because positions are known exactly, not via noisy scan.
        self.declare_parameter('all_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('all_spawn_offsets',
                                [0.0, 0.5, 0.0, 1.5])
        self.declare_parameter('peer_repulse_radius', 1.50)
        self.declare_parameter('k_peer_repulse', 0.80)

        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.rep_r = float(self.get_parameter('repulse_radius').value)
        self.k_a = float(self.get_parameter('k_attract').value)
        self.k_r = float(self.get_parameter('k_repulse').value)
        rate = float(self.get_parameter('control_rate_hz').value)

        all_names = list(self.get_parameter('all_namespaces')
                         .get_parameter_value().string_array_value)
        all_offsets_flat = list(self.get_parameter('all_spawn_offsets')
                                 .get_parameter_value().double_array_value)
        self.peer_rep_r = float(
            self.get_parameter('peer_repulse_radius').value)
        self.k_peer = float(self.get_parameter('k_peer_repulse').value)
        # Build peers: {ns: (delta_x, delta_y)} where delta is the
        # peer's spawn offset minus my spawn offset, so that
        #   peer_pos_in_my_odom = peer_odom + delta
        offsets = {
            all_names[i]: (all_offsets_flat[2 * i],
                           all_offsets_flat[2 * i + 1])
            for i in range(len(all_names))
        }
        if ns not in offsets:
            self.get_logger().warn(
                f'namespace "{ns}" not in all_namespaces; peer '
                f'repulsion disabled')
            my_off = (0.0, 0.0)
            peer_names = []
        else:
            my_off = offsets[ns]
            peer_names = [n for n in all_names if n != ns]
        self.peer_deltas = {
            n: (offsets[n][0] - my_off[0], offsets[n][1] - my_off[1])
            for n in peer_names
        }
        # latest peer position in MY odom frame (None until first message)
        self.peer_pos = {n: None for n in peer_names}

        scan_topic = f'/{ns}/scan'
        odom_topic = f'/{ns}/odom'
        goal_topic = f'/{ns}/goal_pose'
        cmd_topic = f'/{ns}/cmd_vel'
        at_goal_topic = f'/{ns}/at_goal'

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, latched)
        # Subscribe to each peer's odom; convert into MY frame on receive.
        for peer in self.peer_deltas:
            self.create_subscription(
                Odometry, f'/{peer}/odom',
                lambda msg, n=peer: self._on_peer_odom(n, msg),
                10,
            )

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.at_goal_pub = self.create_publisher(Bool, at_goal_topic, latched)

        self.scan = None
        self.pose = None
        self.goal = None
        self.arrived = False

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'ExploreController (potential field) ready '
            f'(ns={ns}, max_lin={self.max_lin}, k_attract={self.k_a}, '
            f'k_repulse={self.k_r}, repulse_radius={self.rep_r})'
        )

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pose = (p.x, p.y, _yaw_from_quat(o.x, o.y, o.z, o.w))

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.arrived = False
        self._publish_at_goal(False)
        self.get_logger().info(
            f'New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')

    def _on_peer_odom(self, peer_name, msg):
        dx, dy = self.peer_deltas[peer_name]
        self.peer_pos[peer_name] = (
            msg.pose.pose.position.x + dx,
            msg.pose.pose.position.y + dy,
        )

    def _publish_at_goal(self, value):
        m = Bool()
        m.data = value
        self.at_goal_pub.publish(m)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _peer_repulsion(self, rx, ry):
        """Sum of repulsive vectors away from each peer in MY odom frame.
        Magnitude (1/d - 1/R) for d in (0.05, R), zero outside.
        """
        fx, fy = 0.0, 0.0
        for pos in self.peer_pos.values():
            if pos is None:
                continue
            dx = rx - pos[0]
            dy = ry - pos[1]
            d = math.hypot(dx, dy)
            if d <= 0.05 or d >= self.peer_rep_r:
                continue
            weight = (1.0 / d) - (1.0 / self.peer_rep_r)
            # Unit vector AWAY from peer * weight
            fx += weight * dx / d
            fy += weight * dy / d
        return fx, fy

    def _repulsion_world(self, ryaw):
        """Sum of repulsive vectors in world (odom) frame from current scan.

        Returns (rx, ry). Zero if no scan or no rays within repulse_radius.
        """
        s = self.scan
        n = len(s.ranges)
        if n == 0:
            return 0.0, 0.0
        ranges = np.array(s.ranges, dtype=np.float32)
        bearings = s.angle_min + np.arange(n) * s.angle_increment
        valid = (np.isfinite(ranges)
                 & (ranges > s.range_min)
                 & (ranges < s.range_max))
        # Clamp lower bound on range so 1/d doesn't blow up.
        close = valid & (ranges < self.rep_r) & (ranges > 0.05)
        if not np.any(close):
            return 0.0, 0.0
        d = ranges[close]
        b = bearings[close]
        weight = (1.0 / d) - (1.0 / self.rep_r)         # > 0 inside boundary
        world_angle = ryaw + b
        # Each ray: obstacle is at angle world_angle in world frame, repulsion
        # vector points opposite (-cos, -sin), scaled by weight.
        rx = -np.sum(weight * np.cos(world_angle))
        ry = -np.sum(weight * np.sin(world_angle))
        # Normalize by ray count: a wall lit up by 100 rays shouldn't push
        # 100x harder than a single column.
        rx /= len(d)
        ry /= len(d)
        return float(rx), float(ry)

    def _tick(self):
        if self.scan is None or self.pose is None:
            return
        if self.goal is None or self.arrived:
            self._stop()
            return

        gx, gy = self.goal
        rx, ry, ryaw = self.pose
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            self.arrived = True
            self._stop()
            self._publish_at_goal(True)
            self.get_logger().info(f'Arrived at ({gx:.2f}, {gy:.2f})')
            return

        # Attractive force: unit vector toward goal in world frame.
        ax = dx / dist
        ay = dy / dist

        # Repulsive force from lidar in world frame.
        repx, repy = self._repulsion_world(ryaw)

        # Peer repulsion (in same odom frame as ax/ay/repx/repy).
        peer_x, peer_y = self._peer_repulsion(rx, ry)

        # Combined heading vector.
        hx = self.k_a * ax + self.k_r * repx + self.k_peer * peer_x
        hy = self.k_a * ay + self.k_r * repy + self.k_peer * peer_y

        desired_yaw = math.atan2(hy, hx)
        yaw_err = desired_yaw - ryaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        cmd = Twist()
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, 2.0 * yaw_err))
        h_mag = math.hypot(hx, hy)
        alignment = max(0.0, math.cos(yaw_err))
        cmd.linear.x = self.max_lin * alignment * min(1.0, h_mag)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ExploreController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
