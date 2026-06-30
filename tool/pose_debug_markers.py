#!/usr/bin/env python3
import argparse
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


R_CAM_ARROW = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


class PoseDebugMarkers(Node):
    def __init__(self, args):
        super().__init__("pose_debug_markers")
        self.args = args
        self.pub = self.create_publisher(MarkerArray, args.marker_topic, 10)
        self.create_subscription(Odometry, args.pose_topic, self.pose_callback, 10)
        self.get_logger().info(
            f"Publishing RViz +Z-forward pose marker from {args.pose_topic} to {args.marker_topic}"
        )

    def pose_callback(self, msg: Odometry) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.ns = "current_pose_z_forward"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = msg.pose.pose.position
        if self.args.flatten_z is not None:
            marker.pose.position.z = float(self.args.flatten_z)

        q = msg.pose.pose.orientation
        R_world_camera = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        if self.args.yaw_only:
            forward = R_world_camera @ np.array([0.0, 0.0, 1.0])
            yaw = math.atan2(float(forward[1]), float(forward[0]))
            q_arrow = R.from_euler("z", yaw).as_quat()
        else:
            q_arrow = R.from_matrix(R_world_camera @ R_CAM_ARROW).as_quat()
        marker.pose.orientation.x = float(q_arrow[0])
        marker.pose.orientation.y = float(q_arrow[1])
        marker.pose.orientation.z = float(q_arrow[2])
        marker.pose.orientation.w = float(q_arrow[3])
        marker.scale.x = 0.65
        marker.scale.y = 0.08
        marker.scale.z = 0.12
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.15
        marker.color.a = 1.0
        marker.lifetime.sec = 2

        sphere = Marker()
        sphere.header = msg.header
        sphere.ns = "current_pose_z_forward"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = msg.pose.pose.position
        if self.args.flatten_z is not None:
            sphere.pose.position.z = float(self.args.flatten_z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.16
        sphere.scale.y = 0.16
        sphere.scale.z = 0.16
        sphere.color.r = 0.0
        sphere.color.g = 0.85
        sphere.color.b = 1.0
        sphere.color.a = 1.0
        sphere.lifetime.sec = 2

        self.pub.publish(MarkerArray(markers=[marker, sphere]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/map/relocalization")
    parser.add_argument("--marker-topic", default="/mapping/pose_debug_markers")
    parser.add_argument("--flatten-z", type=float, default=None)
    parser.add_argument("--yaw-only", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = PoseDebugMarkers(args)
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
