"""Map merger for multi-robot SLAM.

Subscribes to /<ns>/map for each configured robot namespace and publishes a
single fused OccupancyGrid on /map_merged. Each robot's slam_toolbox map frame
is anchored at the robot's startup pose, so we offset each grid by the
declared spawn pose before fusing.

Fusion rule per cell:
  - if all sources are unknown (-1), keep -1
  - else take max() of the known values (occupied dominates free)
"""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

UNKNOWN = -1


class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('robot_initial_poses',
                                [0.0, 0.5, 0.0, 1.5])
        self.declare_parameter('merged_frame', 'map_merged')
        self.declare_parameter('publish_rate_hz', 1.0)

        names = list(self.get_parameter('robot_namespaces')
                     .get_parameter_value().string_array_value)
        flat = list(self.get_parameter('robot_initial_poses')
                    .get_parameter_value().double_array_value)
        self.merged_frame = (self.get_parameter('merged_frame')
                             .get_parameter_value().string_value)
        rate = (self.get_parameter('publish_rate_hz')
                .get_parameter_value().double_value)

        if len(flat) != 2 * len(names):
            raise ValueError(
                f'robot_initial_poses needs 2*len(namespaces) entries '
                f'(got {len(flat)} for {len(names)} robots)'
            )

        self.spawn_offsets = {
            n: (flat[2 * i], flat[2 * i + 1]) for i, n in enumerate(names)
        }
        self.latest_maps = {n: None for n in names}

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        for ns in names:
            topic = f'/{ns}/map'
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, n=ns: self._on_map(n, msg),
                latched,
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self.publisher = self.create_publisher(
            OccupancyGrid, '/map_merged', latched
        )
        self.create_timer(1.0 / rate, self._publish_merged)
        self.get_logger().info(
            f'MapMerger ready: {len(names)} robots, frame={self.merged_frame}'
        )

    def _on_map(self, ns, msg):
        self.latest_maps[ns] = msg

    def _publish_merged(self):
        valid = [(n, m) for n, m in self.latest_maps.items() if m is not None]
        if not valid:
            return

        res = valid[0][1].info.resolution
        for n, m in valid:
            if abs(m.info.resolution - res) > 1e-6:
                self.get_logger().warn(
                    f'Resolution mismatch on {n}: {m.info.resolution} vs {res}'
                )
                return

        bounds = []
        for n, m in valid:
            ox, oy = self.spawn_offsets[n]
            x0 = m.info.origin.position.x + ox
            y0 = m.info.origin.position.y + oy
            x1 = x0 + m.info.width * res
            y1 = y0 + m.info.height * res
            bounds.append((x0, y0, x1, y1))

        min_x = min(b[0] for b in bounds)
        min_y = min(b[1] for b in bounds)
        max_x = max(b[2] for b in bounds)
        max_y = max(b[3] for b in bounds)

        W = int(np.ceil((max_x - min_x) / res))
        H = int(np.ceil((max_y - min_y) / res))
        merged = np.full((H, W), UNKNOWN, dtype=np.int16)

        for n, m in valid:
            ox, oy = self.spawn_offsets[n]
            ci = int(round((m.info.origin.position.x + ox - min_x) / res))
            cj = int(round((m.info.origin.position.y + oy - min_y) / res))
            data = np.array(m.data, dtype=np.int16).reshape(
                m.info.height, m.info.width
            )

            sub = merged[cj:cj + m.info.height, ci:ci + m.info.width]
            known_src = data >= 0
            known_dst = sub >= 0
            take_src = known_src & ~known_dst
            sub[take_src] = data[take_src]
            both = known_src & known_dst
            sub[both] = np.maximum(sub[both], data[both])
            merged[cj:cj + m.info.height, ci:ci + m.info.width] = sub

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.merged_frame
        out.info.resolution = res
        out.info.width = W
        out.info.height = H
        out.info.origin.position.x = min_x
        out.info.origin.position.y = min_y
        out.info.origin.orientation.w = 1.0
        out.data = merged.astype(np.int8).flatten().tolist()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = MapMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
