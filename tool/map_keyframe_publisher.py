#!/usr/bin/env python3
"""Publish saved TinyNav map keyframe poses for localization debugging."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray


def pose_from_matrix(matrix: np.ndarray, frame_id: str, stamp, project_z: float | None = None) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    t = matrix[:3, 3]
    pose.pose.position.x = float(t[0])
    pose.pose.position.y = float(t[1])
    pose.pose.position.z = float(t[2] if project_z is None else project_z)

    if project_z is None:
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    else:
        # TinyNav camera/body convention uses +Z as forward. Project it to map XY.
        forward = matrix[:3, :3] @ np.array([0.0, 0.0, 1.0])
        yaw = math.atan2(float(forward[1]), float(forward[0]))
        q = Rotation.from_euler("z", yaw).as_quat()

    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])
    return pose


class MapKeyframePublisher(Node):
    def __init__(self, map_path: Path, frame_id: str, period: float, project_z: float | None):
        super().__init__("map_keyframe_publisher")
        self.frame_id = frame_id
        self.map_path = map_path
        self.project_z = project_z
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(PathMsg, "/mapping/map_keyframe_path", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/mapping/map_keyframe_markers", qos)
        self.poses = self.load_poses(map_path)
        mode = "3D" if project_z is None else f"2D projected at z={project_z:.2f}"
        self.get_logger().info(f"Publishing map keyframe trajectory in {mode} on /mapping/map_keyframe_path")
        self.create_timer(max(0.5, float(period)), self.publish)
        self.publish()

    def load_poses(self, map_path: Path) -> list[np.ndarray]:
        poses_path = map_path / "poses.npy"
        if not poses_path.exists():
            raise FileNotFoundError(f"Missing {poses_path}")
        data = np.load(poses_path, allow_pickle=True).item()
        poses = [np.asarray(data[t], dtype=np.float64) for t in sorted(data.keys())]
        self.get_logger().info(f"Loaded {len(poses)} map keyframe poses from {poses_path}")
        return poses

    def publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        path = PathMsg()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        path.poses = [pose_from_matrix(pose, self.frame_id, stamp, self.project_z) for pose in self.poses]
        self.path_pub.publish(path)

        markers = []
        if self.poses:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = "map_keyframes"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.r = 1.0
            line.color.g = 1.0
            line.color.b = 0.1
            line.color.a = 1.0
            for pose in self.poses:
                ps = pose_from_matrix(pose, self.frame_id, stamp, self.project_z)
                line.points.append(ps.pose.position)
            markers.append(line)

            for idx, pose in enumerate(self.poses[:: max(1, len(self.poses) // 40)]):
                arrow = Marker()
                arrow.header.frame_id = self.frame_id
                arrow.header.stamp = stamp
                arrow.ns = "map_keyframe_arrows"
                arrow.id = idx + 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose = pose_from_matrix(pose, self.frame_id, stamp, self.project_z).pose
                arrow.scale.x = 0.35
                arrow.scale.y = 0.07
                arrow.scale.z = 0.07
                arrow.color.r = 1.0
                arrow.color.g = 0.95
                arrow.color.b = 0.1
                arrow.color.a = 0.85
                markers.append(arrow)

        self.marker_pub.publish(MarkerArray(markers=markers))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tinynav-map-path", type=Path, required=True)
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument(
        "--project-z",
        type=float,
        default=None,
        help="Project the saved keyframe trajectory to this fixed Z height for 2D map overlays.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MapKeyframePublisher(args.tinynav_map_path, args.frame_id, args.period, args.project_z)
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
