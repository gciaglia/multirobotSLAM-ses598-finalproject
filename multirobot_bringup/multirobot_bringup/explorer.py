"""Multi-robot exploration coordinator.

Reads /frontiers (in map_merged frame) and assigns frontier centroids to
idle robots. Per robot, runs a small state machine:

    IDLE  -- pick an unclaimed frontier --> BUSY
    BUSY  -- /<ns>/at_goal == True       --> IDLE
    BUSY  -- no progress for stuck_timeout sec --> IDLE (abandon goal)

Selection strategy is pluggable: this module ships a uniform-random
baseline; UCB1 over multiple strategies will plug in here later.

Frame handling: frontiers are in map_merged. Each robot's odom frame
origin equals its spawn pose in map_merged (no rotation at spawn), so
the goal in a robot's odom frame is simply:

    goal_odom = frontier_merged - spawn_offset

This avoids any TF lookups -- robust enough as long as slam_toolbox's
map<->odom correction stays small relative to inter-frontier distances.
"""

import csv
import math
import os
import random
import time as time_mod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


# --- Per-robot state -------------------------------------------------
# One instance per robot. The explorer node owns a dict {ns: RobotState}.

@dataclass
class RobotState:
    namespace: str
    spawn: Tuple[float, float]          # (x, y) in map_merged frame
    pose: Optional[Tuple[float, float]] = None   # current (x, y) in merged
    busy: bool = False
    goal_merged: Optional[Tuple[float, float]] = None
    goal_started_t: float = 0.0
    goal_started_pose: Optional[Tuple[float, float]] = None
    # Bandit accounting for THIS goal: which arm chose it, and the count
    # of known cells in the merged map at start time. The reward at the
    # end of this goal is (known cells now) - goal_started_cells.
    current_arm: Optional[str] = None
    goal_started_cells: int = 0


@dataclass
class BanditArm:
    """One arm of the UCB1 bandit. `pulls` and `sum_reward` accumulate
    across goal attempts that used this strategy. `successes` counts
    how many of those ended in arrival (vs. stuck-abandon).
    """
    name: str
    pulls: int = 0
    sum_reward: float = 0.0
    successes: int = 0

    @property
    def mean_reward(self) -> float:
        return self.sum_reward / self.pulls if self.pulls > 0 else 0.0

    @property
    def success_rate(self) -> float:
        return self.successes / self.pulls if self.pulls > 0 else 0.0

    def ucb1_score(self, total_pulls: int) -> float:
        # An untried arm gets +inf -> guaranteed exploration first.
        if self.pulls == 0:
            return float('inf')
        explore = math.sqrt(2.0 * math.log(max(1, total_pulls)) / self.pulls)
        return self.mean_reward + explore


# --- The node --------------------------------------------------------

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # Same source-of-truth as multirobot.launch.py: list of namespaces +
        # flat list of [x1,y1, x2,y2, ...] spawn poses. Defaults match the
        # 2-robot defaults.
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('robot_initial_poses',
                                [0.0, 0.5, 0.0, 1.5])
        self.declare_parameter('frontiers_topic', '/frontiers')
        self.declare_parameter('selection_strategy', 'ucb1')  # 'ucb1' | 'random' | 'nearest' | 'furthest' | 'spread'
        self.declare_parameter('stuck_timeout_sec', 10.0)
        self.declare_parameter('stuck_min_progress_m', 0.20)
        self.declare_parameter('claim_separation_m', 2.0)
        # Drop candidates this close (or closer) to the robot's current
        # pose. Prevents the "I'm already there" self-trap in greedy
        # strategies. Should be > controller goal_tolerance (0.3m).
        self.declare_parameter('min_goal_distance_m', 0.6)
        self.declare_parameter('decision_rate_hz', 1.0)
        # Bootstrap: if no frontiers and a robot is IDLE, push a small
        # "wander" goal so slam_toolbox sees motion and publishes its map.
        self.declare_parameter('bootstrap_wander_m', 1.5)
        # Cell value threshold for "occupied" in OccupancyGrid (0..100).
        self.declare_parameter('los_occupied_thresh', 50)
        # Skip the first N cells of the Bresenham line (avoid spurious self-
        # blocked false positives from localization noise around the robot).
        self.declare_parameter('los_skip_near_cells', 2)

        names = list(self.get_parameter('robot_namespaces')
                     .get_parameter_value().string_array_value)
        flat = list(self.get_parameter('robot_initial_poses')
                    .get_parameter_value().double_array_value)
        if len(flat) != 2 * len(names):
            raise ValueError(
                f'robot_initial_poses needs 2*len(namespaces) entries '
                f'(got {len(flat)} for {len(names)} robots)'
            )

        self.stuck_timeout = float(
            self.get_parameter('stuck_timeout_sec').value)
        self.stuck_min_progress = float(
            self.get_parameter('stuck_min_progress_m').value)
        self.claim_sep = float(self.get_parameter('claim_separation_m').value)
        self.min_goal_dist = float(
            self.get_parameter('min_goal_distance_m').value)
        self.bootstrap_wander = float(
            self.get_parameter('bootstrap_wander_m').value)
        self.los_thresh = int(self.get_parameter('los_occupied_thresh').value)
        self.los_skip_near = int(
            self.get_parameter('los_skip_near_cells').value)
        self.strategy = self.get_parameter(
            'selection_strategy').get_parameter_value().string_value

        # Per-robot state, keyed by namespace.
        self.robots = {
            ns: RobotState(namespace=ns, spawn=(flat[2*i], flat[2*i+1]))
            for i, ns in enumerate(names)
        }

        # Bandit arms. Each strategy is a different way to score available
        # frontiers; UCB1 picks WHICH strategy to use this round.
        self.arm_names = ['random', 'nearest', 'furthest', 'spread']
        self.arms = {n: BanditArm(name=n) for n in self.arm_names}
        self.bandit_total_pulls = 0

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # I/O. Frontiers are latched (we want the latest available); per-
        # robot odom is volatile (we just want fresh updates); at_goal is
        # latched (set once when arrived); goal_pose is latched (controller
        # is also latched, see explore_controller.py).
        self.frontiers: Optional[PoseArray] = None
        self.merged_map: Optional[OccupancyGrid] = None
        self.create_subscription(
            PoseArray,
            self.get_parameter('frontiers_topic').value,
            self._on_frontiers,
            latched,
        )
        self.create_subscription(
            OccupancyGrid, '/map_merged', self._on_merged_map, latched)

        self.goal_pubs = {}
        for ns, st in self.robots.items():
            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, s=st: self._on_odom(s, msg),
                10,
            )
            self.create_subscription(
                Bool, f'/{ns}/at_goal',
                lambda msg, s=st: self._on_at_goal(s, msg),
                latched,
            )
            self.goal_pubs[ns] = self.create_publisher(
                PoseStamped, f'/{ns}/goal_pose', latched)

        rate = float(self.get_parameter('decision_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        # Per-pull events log: one CSV row each time _settle_bandit is
        # called. Useful for plotting "arm pulls over time" and "reward
        # per attempt" later.
        stamp = time_mod.strftime('%Y%m%d_%H%M%S')
        self._events_path = os.path.expanduser(
            f'~/multirobot_bandit_events_{stamp}.csv')
        self._events_file = open(self._events_path, 'w', newline='')
        self._events_csv = csv.writer(self._events_file)
        self._events_csv.writerow([
            'wall_time', 'robot', 'arm', 'reward', 'outcome',
            'arm_pulls_after', 'arm_mean_after', 'total_pulls_after',
        ])
        self._events_file.flush()

        self.get_logger().info(
            f'Explorer ready: {len(self.robots)} robots, '
            f'strategy={self.strategy}, stuck_timeout={self.stuck_timeout}s'
        )
        self.get_logger().info(
            f'  bandit events log: {self._events_path}'
        )

    # --- Callbacks ---------------------------------------------------

    def _on_frontiers(self, msg):
        self.frontiers = msg

    def _on_merged_map(self, msg):
        self.merged_map = msg

    def _on_odom(self, st: RobotState, msg):
        # Convert from robot's odom frame to map_merged by adding spawn offset.
        sx, sy = st.spawn
        st.pose = (msg.pose.pose.position.x + sx,
                   msg.pose.pose.position.y + sy)

    def _on_at_goal(self, st: RobotState, msg):
        # at_goal=True is the "I arrived" signal from explore_controller.
        # We only react to True; controller sets it back to False on each new
        # goal so we don't double-handle the same arrival.
        if msg.data and st.busy:
            self.get_logger().info(
                f'[{st.namespace}] arrived at goal '
                f'{st.goal_merged}; freeing'
            )
            self._settle_bandit(st, success=True)
            st.busy = False
            st.goal_merged = None

    # --- Main decision loop -----------------------------------------

    def _tick(self):
        # The whole algorithm: per robot, either check for stuck-and-abandon,
        # or pick a new frontier.
        now = self.get_clock().now().nanoseconds / 1e9

        # First pass (always): detect stuck robots and free them. This must
        # run even when there are no frontiers, so robots stranded on
        # bootstrap goals get rescued.
        for st in self.robots.values():
            if st.busy and self._is_stuck(st, now):
                self.get_logger().warn(
                    f'[{st.namespace}] stuck (no progress in '
                    f'{self.stuck_timeout}s), abandoning {st.goal_merged}'
                )
                self._settle_bandit(st, success=False)
                st.busy = False
                st.goal_merged = None

        no_frontiers_yet = (self.frontiers is None
                            or not self.frontiers.poses)
        if no_frontiers_yet:
            self._bootstrap_idle_robots(now)
            return

        # Build the claimed-set so we don't double-assign.
        claimed = [st.goal_merged for st in self.robots.values()
                   if st.busy and st.goal_merged is not None]

        # Second pass: assign goals to idle robots.
        candidates = [(p.position.x, p.position.y)
                      for p in self.frontiers.poses]

        for st in self.robots.values():
            if st.busy or st.pose is None:
                continue
            available = [
                c for c in candidates
                if math.hypot(c[0] - st.pose[0],
                              c[1] - st.pose[1]) >= self.min_goal_dist
                and not self._too_close_to_any(c, claimed)
            ]
            if not available:
                continue
            choice, arm_name = self._select(st, available)
            self._assign(st, choice, now, arm_name)
            claimed.append(choice)  # keep subsequent picks aware

    # --- Helpers used by the loop -----------------------------------

    def _bootstrap_idle_robots(self, now: float):
        """When there are no frontiers yet, push idle robots out by a small
        random wander goal in the merged frame so SLAM sees motion and
        publishes its first map. Once frontiers exist, _tick takes over.

        Bootstrap goals are tagged with arm='bootstrap' so the reward they
        earn doesn't pollute the four real strategy arms.
        """
        for st in self.robots.values():
            if st.busy or st.pose is None:
                continue
            angle = random.uniform(-math.pi, math.pi)
            dx = self.bootstrap_wander * math.cos(angle)
            dy = self.bootstrap_wander * math.sin(angle)
            target = (st.pose[0] + dx, st.pose[1] + dy)
            self.get_logger().info(
                f'[{st.namespace}] no frontiers yet; bootstrap wander to '
                f'{target}'
            )
            self._assign(st, target, now, 'bootstrap')

    def _is_stuck(self, st: RobotState, now: float) -> bool:
        """Rolling-window stuck check: stuck if the robot has not gained
        at least stuck_min_progress meters toward the goal during the
        most recent stuck_timeout seconds.

        Whenever the robot DOES make that much progress, we slide the
        window forward (update goal_started_pose to current pose,
        goal_started_t to now). So a robot that drove 5m then started
        circling will be flagged stuck stuck_timeout sec after it last
        gained ground.
        """
        if (st.goal_started_pose is None or st.pose is None
                or st.goal_merged is None):
            return False

        window_initial_dist = math.hypot(
            st.goal_started_pose[0] - st.goal_merged[0],
            st.goal_started_pose[1] - st.goal_merged[1])
        current_dist = math.hypot(
            st.pose[0] - st.goal_merged[0],
            st.pose[1] - st.goal_merged[1])
        progress_in_window = window_initial_dist - current_dist

        # Made progress in this window -> slide it forward, not stuck.
        if progress_in_window >= self.stuck_min_progress:
            st.goal_started_pose = st.pose
            st.goal_started_t = now
            return False

        # No meaningful progress yet; only flag stuck once the full
        # window timeout has elapsed.
        return (now - st.goal_started_t) >= self.stuck_timeout

    def _too_close_to_any(self, c, others) -> bool:
        for o in others:
            if math.hypot(c[0] - o[0], c[1] - o[1]) < self.claim_sep:
                return True
        return False

    # --- Strategies (each: (robot, available) -> chosen frontier) ----

    def _strat_random(self, st, available):
        return random.choice(available)

    def _strat_nearest(self, st, available):
        return min(available, key=lambda c: math.hypot(
            c[0] - st.pose[0], c[1] - st.pose[1]))

    def _strat_furthest(self, st, available):
        return max(available, key=lambda c: math.hypot(
            c[0] - st.pose[0], c[1] - st.pose[1]))

    def _strat_spread(self, st, available):
        """Pick the frontier that is furthest from any other robot's pose
        (max-min distance). Encourages robots to fan out."""
        others = [o.pose for o in self.robots.values()
                  if o is not st and o.pose is not None]
        if not others:
            return self._strat_random(st, available)

        def min_dist_to_others(c):
            return min(math.hypot(c[0] - o[0], c[1] - o[1]) for o in others)

        return max(available, key=min_dist_to_others)

    def _strategy_fn(self, name):
        return {
            'random': self._strat_random,
            'nearest': self._strat_nearest,
            'furthest': self._strat_furthest,
            'spread': self._strat_spread,
        }[name]

    # --- Selection: returns (chosen_frontier, arm_name_for_accounting) ---

    def _select(self, st: RobotState, available: List[Tuple[float, float]]):
        if self.strategy == 'ucb1':
            arm = max(self.arms.values(),
                      key=lambda a: a.ucb1_score(self.bandit_total_pulls))
            choice = self._strategy_fn(arm.name)(st, available)
            return choice, arm.name
        # Fixed (non-bandit) strategies: still record the arm name so the
        # reward bookkeeping stays consistent, but bandit selection is off.
        if self.strategy in self.arm_names:
            choice = self._strategy_fn(self.strategy)(st, available)
            return choice, self.strategy
        # Unknown strategy -> default to random.
        return self._strat_random(st, available), 'random'

    def _assign(self, st: RobotState, goal_merged: Tuple[float, float],
                now: float, arm_name: str):
        # Convert merged-frame goal -> robot's local odom frame.
        gx_odom = goal_merged[0] - st.spawn[0]
        gy_odom = goal_merged[1] - st.spawn[1]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{st.namespace}/odom'
        msg.pose.position.x = gx_odom
        msg.pose.position.y = gy_odom
        msg.pose.orientation.w = 1.0
        self.goal_pubs[st.namespace].publish(msg)

        st.busy = True
        st.goal_merged = goal_merged
        st.goal_started_t = now
        st.goal_started_pose = st.pose
        st.current_arm = arm_name
        st.goal_started_cells = self._count_known_cells()
        self.get_logger().info(
            f'[{st.namespace}] assigned {goal_merged} via arm={arm_name} '
            f'(odom-local: ({gx_odom:.2f}, {gy_odom:.2f}))'
        )

    # --- Bandit reward bookkeeping -----------------------------------

    def _count_known_cells(self) -> int:
        """Count cells in /map_merged that are NOT unknown (-1)."""
        if self.merged_map is None:
            return 0
        # Vectorize for speed: data is a list[int8] of length W*H.
        import numpy as _np
        a = _np.asarray(self.merged_map.data, dtype=_np.int16)
        return int((a >= 0).sum())

    def _settle_bandit(self, st: RobotState, success: bool):
        """Called when a goal ends (arrival or stuck-abandon). Updates the
        chosen arm with reward = (cells gained during this attempt)."""
        if st.current_arm is None or st.current_arm == 'bootstrap':
            st.current_arm = None
            return
        gained = self._count_known_cells() - st.goal_started_cells
        # Floor at 0 -- "negative gain" can happen if the merger trims a
        # frame; we don't want to penalize an arm for measurement noise.
        reward = max(0.0, float(gained))
        arm = self.arms[st.current_arm]
        arm.pulls += 1
        arm.sum_reward += reward
        if success:
            arm.successes += 1
        self.bandit_total_pulls += 1
        outcome = 'arrived' if success else 'abandoned'

        # Log to terminal.
        self.get_logger().info(
            f'[{st.namespace}] arm={st.current_arm} reward={reward:.0f} '
            f'({outcome}); arm_mean={arm.mean_reward:.1f} '
            f'pulls={arm.pulls}'
        )
        # Append a row to the per-pull events CSV.
        self._events_csv.writerow([
            f'{time_mod.time():.3f}', st.namespace, st.current_arm,
            f'{reward:.0f}', outcome, arm.pulls,
            f'{arm.mean_reward:.2f}', self.bandit_total_pulls,
        ])
        self._events_file.flush()

        st.current_arm = None
        # Periodically log the full bandit table.
        if self.bandit_total_pulls % 5 == 0:
            self._log_bandit_table()

    def _log_bandit_table(self):
        rows = []
        for n in self.arm_names:
            a = self.arms[n]
            rows.append(f'{n}: p={a.pulls} mean={a.mean_reward:.1f}')
        self.get_logger().info(
            f'bandit (total_pulls={self.bandit_total_pulls}): '
            + ' | '.join(rows)
        )

    def print_bandit_summary(self):
        """Final per-arm summary, printed on shutdown."""
        self.get_logger().info('====== BANDIT FINAL SUMMARY ======')
        self.get_logger().info(f'  events log: {self._events_path}')
        self.get_logger().info(
            f'  total pulls: {self.bandit_total_pulls}')
        self.get_logger().info('  arm        pulls  mean   success_rate')
        self.get_logger().info('  ' + '-' * 40)
        for n in self.arm_names:
            a = self.arms[n]
            self.get_logger().info(
                f'  {n:<10} {a.pulls:<6} {a.mean_reward:<6.1f} '
                f'{a.success_rate:.2f}'
            )
        self.get_logger().info('==================================')

    def close(self):
        if hasattr(self, '_events_file') and not self._events_file.closed:
            self._events_file.close()


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.print_bandit_summary()
        except Exception as e:
            node.get_logger().warn(f'summary failed: {e}')
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
