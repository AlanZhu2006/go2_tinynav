#!/usr/bin/env python3
import argparse
import json
import sys
import tempfile
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Bridge RViz /goal_pose into TinyNav POIs. Each incoming goal updates "
            "pois.json and publishes the same payload to /mapping/cmd_pois."
        )
    )
    parser.add_argument("--tinynav_map_path", required=True)
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--cmd-pois-topic", default="/mapping/cmd_pois")
    parser.add_argument("--marker-topic", default="/rviz_goal_marker")
    parser.add_argument("--pose-marker-topic", default="/rviz_goal_pose_marker")
    parser.add_argument("--poi-id", default="0", help="POI id to write when replacing the active target.")
    parser.add_argument("--poi-name", default="rviz_goal")
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append each RViz goal as a new POI instead of replacing the active POI list.",
    )
    parser.add_argument(
        "--z",
        type=float,
        default=None,
        help="Override goal z. By default the z value from RViz goal_pose is used.",
    )
    parser.add_argument(
        "--marker-z-offset",
        type=float,
        default=1.5,
        help="Raise RViz-only goal markers above the saved POI z value.",
    )
    return parser.parse_args()


def load_pois(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def next_poi_id(data: dict[str, object]) -> str:
    if not data:
        return "0"
    numeric_keys = [int(key) for key in data.keys() if str(key).isdigit()]
    return str(max(numeric_keys, default=-1) + 1)


def write_json_atomic(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as tmp:
        json.dump(data, tmp, indent=2)
        tmp.write("\n")
        tmp_path = Path(tmp.name)
    tmp_path.replace(path)


class RvizGoalToPoi(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("rviz_goal_to_poi")
        self.args = args
        self.map_path = Path(args.tinynav_map_path)
        self.pois_path = self.map_path / "pois.json"
        poi_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, args.cmd_pois_topic, poi_qos)
        self._last_payload: str | None = None
        self.create_timer(2.0, self._republish)
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(MarkerArray, args.marker_topic, marker_qos)
        self.pose_marker_publisher = self.create_publisher(PoseStamped, args.pose_marker_topic, marker_qos)
        self.subscription = self.create_subscription(PoseStamped, args.goal_topic, self.goal_callback, 10)
        self.point_subscription = self.create_subscription(PointStamped, "/clicked_point", self.point_callback, 10)
        self.get_logger().info(
            f"Listening on {args.goal_topic}; writing {self.pois_path}; "
            f"publishing {args.cmd_pois_topic}, {args.marker_topic}, and {args.pose_marker_topic}"
        )

    def _republish(self) -> None:
        if self._last_payload is None:
            return
        out = String()
        out.data = self._last_payload
        self.publisher.publish(out)

    def publish_marker(self, goal_msg: PoseStamped, position: list[float]) -> None:
        header = goal_msg.header
        if not header.frame_id:
            header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        marker_z = float(position[2]) + float(self.args.marker_z_offset)

        arrow = Marker()
        arrow.header = header
        arrow.ns = "rviz_goal"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = goal_msg.pose
        arrow.pose.position.x = float(position[0])
        arrow.pose.position.y = float(position[1])
        arrow.pose.position.z = marker_z
        arrow.scale.x = 0.45
        arrow.scale.y = 0.05
        arrow.scale.z = 0.12
        arrow.color.r = 0.05
        arrow.color.g = 1.0
        arrow.color.b = 0.2
        arrow.color.a = 1.0
        arrow.lifetime = Duration(seconds=0).to_msg()

        sphere = Marker()
        sphere.header = header
        sphere.ns = "rviz_goal"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(position[0])
        sphere.pose.position.y = float(position[1])
        sphere.pose.position.z = marker_z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.16
        sphere.scale.y = 0.16
        sphere.scale.z = 0.16
        sphere.color.r = 1.0
        sphere.color.g = 0.75
        sphere.color.b = 0.05
        sphere.color.a = 1.0
        sphere.lifetime = Duration(seconds=0).to_msg()

        label = Marker()
        label.header = header
        label.ns = "rviz_goal"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(position[0])
        label.pose.position.y = float(position[1])
        label.pose.position.z = marker_z + 0.22
        label.pose.orientation.w = 1.0
        label.scale.z = 0.18
        label.color.r = 0.1
        label.color.g = 0.9
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = "RViz Goal"
        label.lifetime = Duration(seconds=0).to_msg()

        self.marker_publisher.publish(MarkerArray(markers=[arrow, sphere, label]))

        pose_marker = PoseStamped()
        pose_marker.header = header
        pose_marker.pose = goal_msg.pose
        pose_marker.pose.position.x = float(position[0])
        pose_marker.pose.position.y = float(position[1])
        pose_marker.pose.position.z = marker_z
        self.pose_marker_publisher.publish(pose_marker)

    def goal_callback(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        z = position.z if self.args.z is None else self.args.z
        poi_position = [float(position.x), float(position.y), float(z)]
        poi = {
            "id": int(self.args.poi_id) if str(self.args.poi_id).isdigit() else self.args.poi_id,
            "name": self.args.poi_name,
            "position": poi_position,
            "source": "rviz_goal_pose",
            "frame_id": msg.header.frame_id,
        }

        try:
            if self.args.append:
                payload = load_pois(self.pois_path)
                key = next_poi_id(payload)
                poi["id"] = int(key)
                payload[key] = poi
            else:
                payload = {str(self.args.poi_id): poi}
            write_json_atomic(self.pois_path, payload)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Failed to update {self.pois_path}: {exc}")
            return

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._last_payload = out.data
        self.publisher.publish(out)
        self.publish_marker(msg, poi_position)
        self.get_logger().info(
            "Updated TinyNav POI from RViz goal: "
            f"id={poi['id']} position={poi['position']} frame={msg.header.frame_id!r} "
            f"subscribers={self.publisher.get_subscription_count()}"
        )

    def point_callback(self, msg: PointStamped) -> None:
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = msg.point.z
        goal.pose.orientation.w = 1.0
        self.goal_callback(goal)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RvizGoalToPoi(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
