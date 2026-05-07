"""Metrics logger for multi-robot exploration runs.

Subscribes to /map_merged and per-robot /odom. Every `period_sec` seconds,
writes a CSV row to `output_csv` recording:

    sim_time, cells_known, area_m2, robotN_dist_traveled, ...

On Ctrl-C, prints a summary:
    - total runtime (sim seconds)
    - peak coverage (cells, area in m^2)
    - mapping rate (avg cells/sec, area/sec)
    - time to 25/50/75% of peak coverage
    - total distance traveled per robot

Use the CSV directly with matplotlib / pandas / Excel to plot
cells_known(t) curves and compare strategies.
"""

import csv
import math
import os
import signal
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('robot_initial_poses',
                                [0.0, 0.5, 0.0, 1.5])
        self.declare_parameter('map_topic', '/map_merged')
        self.declare_parameter('period_sec', 1.0)
        self.declare_parameter('output_csv', '')

        names = list(self.get_parameter('robot_namespaces')
                     .get_parameter_value().string_array_value)
        flat = list(self.get_parameter('robot_initial_poses')
                    .get_parameter_value().double_array_value)
        if len(flat) != 2 * len(names):
            raise ValueError(
                f'robot_initial_poses needs 2*len(namespaces) entries '
                f'(got {len(flat)} for {len(names)} robots)'
            )
        self.spawn_offsets = {
            names[i]: (flat[2 * i], flat[2 * i + 1])
            for i in range(len(names))
        }
        map_topic = self.get_parameter('map_topic').value
        self.period = float(self.get_parameter('period_sec').value)
        out = self.get_parameter('output_csv').value
        if not out:
            stamp = time.strftime('%Y%m%d_%H%M%S')
            out = os.path.expanduser(
                f'~/multirobot_metrics_{stamp}.csv')
        self.output_csv = out

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_n_frontiers: int = 0
        # Per-robot: previous (x,y) in odom frame, accumulated distance,
        # latest (x,y) in MERGED frame for trajectory logging.
        self.robot_state: Dict[str, Dict[str, float]] = {
            n: {'prev': None, 'dist': 0.0, 'pos_merged': None}
            for n in names
        }
        # Time-series rows for summary at shutdown.
        self.history: List[Tuple[float, int, Dict[str, float]]] = []
        self.start_time: Optional[float] = None

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic,
                                  self._on_map, latched)
        self.create_subscription(PoseArray, '/frontiers',
                                  self._on_frontiers, latched)
        for n in names:
            self.create_subscription(
                Odometry, f'/{n}/odom',
                lambda msg, ns=n: self._on_odom(ns, msg),
                10,
            )

        # Open CSV, write header.
        self._csv_file = open(self.output_csv, 'w', newline='')
        self._csv = csv.writer(self._csv_file)
        header = ['sim_time', 'cells_known', 'area_m2', 'n_frontiers']
        for n in names:
            header += [f'{n}_dist_m', f'{n}_x', f'{n}_y']
        self._csv.writerow(header)
        self._csv_file.flush()

        self.create_timer(self.period, self._tick)
        self.get_logger().info(
            f'MetricsLogger ready: writing {self.output_csv} '
            f'every {self.period}s for {names}'
        )

    def _on_map(self, msg):
        self.latest_map = msg

    def _on_frontiers(self, msg):
        self.latest_n_frontiers = len(msg.poses)

    def _on_odom(self, ns, msg):
        st = self.robot_state[ns]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if st['prev'] is not None:
            px, py = st['prev']
            d = math.hypot(x - px, y - py)
            # Filter out odom noise spikes (>1m in one tick is impossible).
            if d < 1.0:
                st['dist'] += d
        st['prev'] = (x, y)
        # Merged-frame position: odom-frame + spawn offset.
        sx, sy = self.spawn_offsets[ns]
        st['pos_merged'] = (x + sx, y + sy)

    def _tick(self):
        if self.latest_map is None:
            return
        m = self.latest_map
        t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
        if self.start_time is None:
            self.start_time = t
        rel_t = t - self.start_time

        a = np.asarray(m.data, dtype=np.int16)
        cells_known = int((a >= 0).sum())
        area = cells_known * (m.info.resolution ** 2)
        dists = {n: self.robot_state[n]['dist']
                 for n in self.robot_state}

        row = [f'{rel_t:.3f}', cells_known, f'{area:.3f}',
               self.latest_n_frontiers]
        for n in self.robot_state:
            row.append(f'{dists[n]:.3f}')
            pos = self.robot_state[n]['pos_merged']
            if pos is None:
                row.extend(['', ''])
            else:
                row.extend([f'{pos[0]:.3f}', f'{pos[1]:.3f}'])
        self._csv.writerow(row)
        self._csv_file.flush()

        self.history.append((rel_t, cells_known, dict(dists)))

    # --- Summary printing on shutdown -------------------------------

    def print_summary(self):
        if not self.history:
            self.get_logger().info('No data collected.')
            return
        peak_cells = max(h[1] for h in self.history)
        peak_area = peak_cells * (self.latest_map.info.resolution ** 2
                                   if self.latest_map else 0.0025)
        runtime = self.history[-1][0]
        avg_rate = peak_cells / runtime if runtime > 0 else 0.0

        # Time-to-X%-of-peak
        thresholds = [0.25, 0.50, 0.75]
        ttx = {}
        for f in thresholds:
            target = peak_cells * f
            for t, c, _ in self.history:
                if c >= target:
                    ttx[f] = t
                    break

        last = self.history[-1]
        lines = [
            '',
            '====== exploration metrics summary ======',
            f'output csv:          {self.output_csv}',
            f'sim runtime:         {runtime:.1f} s',
            f'peak coverage:       {peak_cells} cells = {peak_area:.2f} m^2',
            f'avg mapping rate:    {avg_rate:.1f} cells/s',
        ]
        for f in thresholds:
            t = ttx.get(f)
            lines.append(f'time to {int(f*100)}% peak:   '
                         + (f'{t:.1f} s' if t is not None else '— never reached'))
        lines.append('total distance per robot:')
        for n, d in last[2].items():
            lines.append(f'  {n}: {d:.2f} m')
        lines.append('=========================================')
        for line in lines:
            self.get_logger().info(line)

    def close(self):
        if hasattr(self, '_csv_file') and not self._csv_file.closed:
            self._csv_file.close()


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.print_summary()
        except Exception as e:
            node.get_logger().warn(f'summary failed: {e}')
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
