#!/usr/bin/env python3
"""Publish a TinyNav built map as a latched ROS 2 OccupancyGrid for RViz."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class StaticOccupancyGridPublisher(Node):
    def __init__(self, map_path: Path, topic: str, frame_id: str, z: float, period: float):
        super().__init__("static_occupancy_grid_publisher")
        self.map_path = map_path
        self.topic = topic
        self.frame_id = frame_id
        self.z = float(z)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, topic, qos)
        self.msg = self._load_map()
        self.create_timer(max(0.1, float(period)), self.publish_map)
        self.publish_map()

    def _load_map(self) -> OccupancyGrid:
        occupancy_grid_path = self.map_path / "occupancy_grid.npy"
        occupancy_meta_path = self.map_path / "occupancy_meta.npy"
        if not occupancy_grid_path.exists() or not occupancy_meta_path.exists():
            raise FileNotFoundError(
                f"Expected occupancy_grid.npy and occupancy_meta.npy under {self.map_path}"
            )

        occupancy_grid = np.load(occupancy_grid_path)
        occupancy_meta = np.load(occupancy_meta_path)
        origin = occupancy_meta[:3].astype(np.float64)
        resolution = float(occupancy_meta[3])

        # TinyNav map semantics: 0=unknown, 1=free, 2=occupied.
        xy_plane = np.max(occupancy_grid, axis=2)
        grid_data = np.full(xy_plane.shape, -1, dtype=np.int8)
        grid_data[xy_plane == 1] = 0
        grid_data[xy_plane == 2] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.height = int(xy_plane.shape[0])
        msg.info.width = int(xy_plane.shape[1])
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = self.z
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_data.ravel(order="F").tolist()

        unknown = int(np.count_nonzero(xy_plane == 0))
        free = int(np.count_nonzero(xy_plane == 1))
        occupied = int(np.count_nonzero(xy_plane == 2))
        self.get_logger().info(
            f"Loaded static map from {self.map_path}: shape={occupancy_grid.shape}, "
            f"2d={msg.info.height}x{msg.info.width}, resolution={resolution:.3f}, "
            f"origin=({origin[0]:.3f}, {origin[1]:.3f}, z={self.z:.3f}), "
            f"unknown={unknown}, free={free}, occupied={occupied}"
        )
        return msg

    def publish_map(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tinynav-map-path", type=Path, required=True)
    parser.add_argument("--topic", default="/mapping/static_occupancy_grid")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--period", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = StaticOccupancyGridPublisher(
        map_path=args.tinynav_map_path,
        topic=args.topic,
        frame_id=args.frame_id,
        z=args.z,
        period=args.period,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
