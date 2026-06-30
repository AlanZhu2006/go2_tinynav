#!/usr/bin/env python3
import argparse
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


class Pose2DTrail(Node):
    def __init__(self, args):
        super().__init__("pose_2d_trail")
        self.args = args
        self.poses = deque(maxlen=args.max_poses)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, args.path_topic, qos)
        self.marker_pub = self.create_publisher(MarkerArray, args.marker_topic, qos)
        self.create_subscription(Odometry, args.pose_topic, self.pose_callback, 10)
        self.get_logger().info(
            f"Projecting {args.pose_topic} to 2D map plane: path={args.path_topic}, "
            f"marker={args.marker_topic}, z={args.z:.2f}"
        )

    def pose_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.frame_id = self.args.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = self.args.z

        q = msg.pose.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        # TinyNav camera/body convention uses +Z as forward. Project it to map XY.
        forward = rot @ np.array([0.0, 0.0, 1.0])
        yaw = math.atan2(float(forward[1]), float(forward[0]))
        q_yaw = R.from_euler("z", yaw).as_quat()
        pose.pose.orientation.x = float(q_yaw[0])
        pose.pose.orientation.y = float(q_yaw[1])
        pose.pose.orientation.z = float(q_yaw[2])
        pose.pose.orientation.w = float(q_yaw[3])

        if not self.poses:
            self.poses.append(pose)
        else:
            last = self.poses[-1].pose.position
            dist = math.hypot(pose.pose.position.x - last.x, pose.pose.position.y - last.y)
            if dist >= self.args.min_step:
                self.poses.append(pose)
            else:
                self.poses[-1] = pose

        self.publish_path(pose.header.stamp)
        self.publish_marker(pose)

    def publish_path(self, stamp):
        path = Path()
        path.header.frame_id = self.args.frame_id
        path.header.stamp = stamp
        path.poses = list(self.poses)
        self.path_pub.publish(path)

    def publish_marker(self, pose: PoseStamped):
        arrow = Marker()
        arrow.header = pose.header
        arrow.ns = "current_pose_2d"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose.pose
        arrow.scale.x = self.args.arrow_length
        arrow.scale.y = 0.12
        arrow.scale.z = 0.16
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.15
        arrow.color.a = 1.0

        dot = Marker()
        dot.header = pose.header
        dot.ns = "current_pose_2d"
        dot.id = 1
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x = pose.pose.position.x
        dot.pose.position.y = pose.pose.position.y
        dot.pose.position.z = pose.pose.position.z
        dot.pose.orientation.w = 1.0
        dot.scale.x = 0.24
        dot.scale.y = 0.24
        dot.scale.z = 0.24
        dot.color.r = 0.0
        dot.color.g = 0.8
        dot.color.b = 1.0
        dot.color.a = 1.0

        label = Marker()
        label.header = pose.header
        label.ns = "current_pose_2d"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pose.pose.position.x
        label.pose.position.y = pose.pose.position.y
        label.pose.position.z = pose.pose.position.z + 0.35
        label.pose.orientation.w = 1.0
        label.scale.z = 0.28
        label.color.r = 0.0
        label.color.g = 1.0
        label.color.b = 0.2
        label.color.a = 1.0
        label.text = "RELOC"

        self.marker_pub.publish(MarkerArray(markers=[arrow, dot, label]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/map/relocalization")
    parser.add_argument("--path-topic", default="/mapping/current_pose_2d_path")
    parser.add_argument("--marker-topic", default="/mapping/current_pose_2d_marker")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--z", type=float, default=0.08)
    parser.add_argument("--max-poses", type=int, default=500)
    parser.add_argument("--min-step", type=float, default=0.03)
    parser.add_argument("--arrow-length", type=float, default=0.85)
    args = parser.parse_args()

    rclpy.init()
    node = Pose2DTrail(args)
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
