#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class CurrentPoseMarker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("current_pose_marker")
        self.args = args
        self.trail: deque[Point] = deque(maxlen=args.trail_length)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.sub = self.create_subscription(Odometry, args.pose_topic, self.callback, 20)
        self.get_logger().info(
            f"Publishing {args.pose_topic} as RViz markers on {args.marker_topic} in frame {args.frame_id}"
        )

    def callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z) + self.args.z_offset
        yaw = yaw_from_quat(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.args.frame_id

        arrow = Marker()
        arrow.header = header
        arrow.ns = "current_pose"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose
        arrow.pose.position.z = z
        arrow.scale.x = self.args.arrow_length
        arrow.scale.y = self.args.arrow_width
        arrow.scale.z = self.args.arrow_height
        arrow.color.r = 1.0
        arrow.color.g = 0.05
        arrow.color.b = 0.05
        arrow.color.a = 1.0
        arrow.lifetime = Duration(sec=0).to_msg()

        label = Marker()
        label.header = header
        label.ns = "current_pose"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = z + self.args.text_z_offset
        label.pose.orientation.w = 1.0
        label.scale.z = self.args.text_size
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f"CURRENT\nx={x:.2f} y={y:.2f}\nyaw={math.degrees(yaw):.0f} deg"
        label.lifetime = Duration(sec=0).to_msg()

        p = Point(x=x, y=y, z=z)
        if not self.trail or math.hypot(self.trail[-1].x - x, self.trail[-1].y - y) >= self.args.trail_step:
            self.trail.append(p)

        trail = Marker()
        trail.header = header
        trail.ns = "current_pose"
        trail.id = 2
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = self.args.trail_width
        trail.color.r = 1.0
        trail.color.g = 0.15
        trail.color.b = 0.05
        trail.color.a = 1.0
        trail.points = list(self.trail)
        trail.lifetime = Duration(sec=0).to_msg()

        self.pub.publish(MarkerArray(markers=[arrow, label, trail]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/mapping/current_pose_in_map")
    parser.add_argument("--marker-topic", default="/mapping/current_pose_marker")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--z-offset", type=float, default=1.8)
    parser.add_argument("--text-z-offset", type=float, default=0.35)
    parser.add_argument("--arrow-length", type=float, default=0.9)
    parser.add_argument("--arrow-width", type=float, default=0.16)
    parser.add_argument("--arrow-height", type=float, default=0.16)
    parser.add_argument("--text-size", type=float, default=0.22)
    parser.add_argument("--trail-width", type=float, default=0.08)
    parser.add_argument("--trail-step", type=float, default=0.05)
    parser.add_argument("--trail-length", type=int, default=1000)
    args = parser.parse_args()

    rclpy.init()
    node = CurrentPoseMarker(args)
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
