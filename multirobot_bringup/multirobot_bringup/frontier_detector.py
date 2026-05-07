"""Frontier detector for multi-robot exploration.

Subscribes to /map_merged. A frontier cell is a free cell adjacent to
an unknown cell. Connected frontier cells are clustered with
4-connectivity, clusters smaller than min_cluster_size are filtered as
sensor noise, and each surviving cluster's centroid is published as a
candidate exploration goal.

Outputs:
  /frontiers      geometry_msgs/PoseArray   centroids in map_merged frame
  /frontiers_viz  visualization_msgs/MarkerArray (RViz cylinders)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

try:
    from scipy.ndimage import label as _scipy_label
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

UNKNOWN = -1
FREE_THRESHOLD = 50  # cells with 0 <= value < threshold counted as free


class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        self.declare_parameter('map_topic', '/map_merged')
        self.declare_parameter('frontiers_topic', '/frontiers')
        self.declare_parameter('viz_topic', '/frontiers_viz')
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('rate_hz', 1.0)

        self.min_size = int(self.get_parameter('min_cluster_size').value)
        rate = float(self.get_parameter('rate_hz').value)
        map_topic = self.get_parameter('map_topic').value

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, latched)
        self.frontiers_pub = self.create_publisher(
            PoseArray,
            self.get_parameter('frontiers_topic').value,
            latched,
        )
        self.viz_pub = self.create_publisher(
            MarkerArray, self.get_parameter('viz_topic').value, 10)

        self.latest_map = None
        self.create_timer(1.0 / rate, self._tick)

        backend = 'scipy.ndimage.label' if HAS_SCIPY else 'pure-python BFS'
        self.get_logger().info(
            f'FrontierDetector ready: input={map_topic}, '
            f'min_cluster={self.min_size}, backend={backend}'
        )

    def _on_map(self, msg):
        self.latest_map = msg

    def _tick(self):
        if self.latest_map is None:
            return
        m = self.latest_map
        H = m.info.height
        W = m.info.width
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        if H == 0 or W == 0:
            return
        data = np.array(m.data, dtype=np.int16).reshape(H, W)

        free = (data >= 0) & (data < FREE_THRESHOLD)
        unknown = (data == UNKNOWN)

        # Frontier = free cell with at least one unknown 4-neighbor
        is_frontier = np.zeros_like(free, dtype=bool)
        is_frontier[:, 1:] |= free[:, 1:] & unknown[:, :-1]
        is_frontier[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        is_frontier[1:, :] |= free[1:, :] & unknown[:-1, :]
        is_frontier[:-1, :] |= free[:-1, :] & unknown[1:, :]

        if not np.any(is_frontier):
            self._publish([], m.header.frame_id)
            return

        labels, n = self._label(is_frontier)

        centroids = []
        for cid in range(1, n + 1):
            cells = np.argwhere(labels == cid)
            if len(cells) < self.min_size:
                continue
            cy_mean = float(cells[:, 0].mean())
            cx_mean = float(cells[:, 1].mean())
            wx = ox + (cx_mean + 0.5) * res
            wy = oy + (cy_mean + 0.5) * res
            centroids.append((wx, wy, len(cells)))

        self._publish(centroids, m.header.frame_id)

    def _label(self, mask):
        if HAS_SCIPY:
            labels, n = _scipy_label(mask)
            return labels.astype(np.int32), int(n)
        return self._label_bfs(mask)

    def _label_bfs(self, mask):
        labels = np.zeros(mask.shape, dtype=np.int32)
        H, W = mask.shape
        n = 0
        for y0 in range(H):
            for x0 in range(W):
                if not mask[y0, x0] or labels[y0, x0]:
                    continue
                n += 1
                stack = [(y0, x0)]
                while stack:
                    y, x = stack.pop()
                    if labels[y, x] or not mask[y, x]:
                        continue
                    labels[y, x] = n
                    if y > 0:
                        stack.append((y - 1, x))
                    if y < H - 1:
                        stack.append((y + 1, x))
                    if x > 0:
                        stack.append((y, x - 1))
                    if x < W - 1:
                        stack.append((y, x + 1))
        return labels, n

    def _publish(self, centroids, frame_id):
        stamp = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = frame_id
        for wx, wy, _ in centroids:
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.frontiers_pub.publish(pa)

        ma = MarkerArray()
        # Always start with a DELETEALL so RViz drops stale markers
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, (wx, wy, size) in enumerate(centroids):
            mk = Marker()
            mk.header.stamp = stamp
            mk.header.frame_id = frame_id
            mk.ns = 'frontiers'
            mk.id = i
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position.x = wx
            mk.pose.position.y = wy
            mk.pose.position.z = 0.05
            mk.pose.orientation.w = 1.0
            # Scale grows mildly with cluster size (visual cue).
            r = 0.25 + 0.02 * min(size, 50)
            mk.scale.x = r
            mk.scale.y = r
            mk.scale.z = 0.10
            mk.color.r = 0.0
            mk.color.g = 1.0
            mk.color.b = 0.0
            mk.color.a = 0.85
            ma.markers.append(mk)

        self.viz_pub.publish(ma)


def main():
    rclpy.init()
    node = FrontierDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
