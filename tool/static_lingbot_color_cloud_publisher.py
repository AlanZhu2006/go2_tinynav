import argparse
import os
import shelve
import sys
import time

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from tinynav.core.math_utils import matrix_to_quat
from tool.video_db import VideoDB


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 3)
    return points @ transform[:3, :3].T + transform[:3, 3]


def gravity_level_transform(points, cam_centers, target_z, thr=0.04, iters=500):
    """Return a 4x4 rigid transform that makes the cloud gravity-aligned (Z-up).

    The VO `world` frame is the camera OPTICAL frame (X-right, Y-down, Z-forward), so the real
    floor normal is ~ -Y, not +Z. We RANSAC the dominant floor plane, orient its normal toward the
    camera centers (up), build the rotation that sends that normal to +Z, and translate the floor to
    `target_z`. A single rigid rotation -- not a z-shear -- so walls stay vertical and parallel.
    Returns None if no dominant plane is found.
    """
    n = points.shape[0]
    if n < 500:
        return None
    rng = np.random.RandomState(0)
    sub = points[rng.choice(n, 60000, replace=False)] if n > 60000 else points
    best_n, best_d, best_cnt = None, 0.0, 0
    for _ in range(iters):
        s = sub[rng.choice(len(sub), 3, replace=False)]
        nv = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(nv)
        if nn < 1e-9:
            continue
        nv = nv / nn
        d = -nv @ s[0]
        cnt = int((np.abs(sub @ nv + d) < thr).sum())
        if cnt > best_cnt:
            best_n, best_d, best_cnt = nv, d, cnt
    if best_n is None or best_cnt < 100:
        return None
    nrm, d = best_n, best_d
    # Orient the normal so the cameras lie on its positive (up) side.
    if cam_centers.size and (cam_centers.mean(0) @ nrm + d) < 0:
        nrm, d = -nrm, -d
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(nrm, z)
    s = float(np.linalg.norm(v))
    c = float(nrm @ z)
    if s < 1e-8:
        R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))
    G = np.eye(4)
    G[:3, :3] = R
    p0 = -d * nrm  # a point on the floor plane
    G[2, 3] = target_z - float((R @ p0)[2])
    return G


def voxel_downsample(points: np.ndarray, colors: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    if points.size == 0 or voxel_size <= 0.0:
        return points.reshape(-1, 3), colors.reshape(-1)
    coords = np.floor(points / voxel_size).astype(np.int32)
    coord_view = (
        np.ascontiguousarray(coords)
        .view(np.dtype((np.void, coords.dtype.itemsize * coords.shape[1])))
        .reshape(-1)
    )
    _, unique_idx = np.unique(coord_view, return_index=True)
    unique_idx.sort()
    return points[unique_idx], colors[unique_idx]


def depth_rgb_to_camera_cloud(
    depth: np.ndarray,
    bgr_image: np.ndarray,
    intrinsics: np.ndarray,
    pixel_step: int,
    min_depth: float,
    max_depth: float,
    filter_ground: bool,
    ground_camera_y: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    depth = np.asarray(depth, dtype=np.float32)
    bgr_image = np.asarray(bgr_image, dtype=np.uint8)
    h, w = depth.shape
    if bgr_image.shape[:2] != (h, w):
        raise ValueError(f"depth/image shape mismatch: depth={depth.shape}, image={bgr_image.shape}")

    ys, xs = np.mgrid[0:h:pixel_step, 0:w:pixel_step]
    sampled_depth = depth[0:h:pixel_step, 0:w:pixel_step]
    valid = np.isfinite(sampled_depth) & (sampled_depth >= min_depth) & (sampled_depth <= max_depth)
    if not np.any(valid):
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0,), dtype=np.uint32),
            np.empty((0,), dtype=bool),
        )

    z = sampled_depth[valid].astype(np.float32)
    u = xs[valid].astype(np.float32)
    v = ys[valid].astype(np.float32)
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.column_stack((x, y, z)).astype(np.float32)

    bgr = bgr_image[ys[valid], xs[valid]].astype(np.uint32)
    colors = (bgr[:, 2] << 16) | (bgr[:, 1] << 8) | bgr[:, 0]
    ground_mask = points[:, 1] > ground_camera_y
    if filter_ground:
        keep = points[:, 1] <= ground_camera_y
        points = points[keep]
        colors = colors[keep]
        ground_mask = ground_mask[keep]
    return points, colors.astype(np.uint32), ground_mask


def fit_ground_plane(points: np.ndarray, max_points: int) -> tuple[float, float, float] | None:
    if points.shape[0] < 100:
        return None
    fit_points = points
    if max_points > 0 and points.shape[0] > max_points:
        idx = np.linspace(0, points.shape[0] - 1, max_points, dtype=np.int64)
        fit_points = points[idx]
    xy = fit_points[:, :2].astype(np.float64)
    z = fit_points[:, 2].astype(np.float64)
    # Trim extreme height outliers before fitting the dominant floor plane.
    lo, hi = np.percentile(z, [5.0, 85.0])
    keep = (z >= lo) & (z <= hi)
    if np.count_nonzero(keep) >= 100:
        xy = xy[keep]
        z = z[keep]
    A = np.column_stack((xy[:, 0], xy[:, 1], np.ones_like(z)))
    coeff, *_ = np.linalg.lstsq(A, z, rcond=None)
    return float(coeff[0]), float(coeff[1]), float(coeff[2])


def lower_envelope_points(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.shape[0] == 0:
        return points.reshape(0, 3)
    coords = np.floor(points[:, :2] / voxel_size).astype(np.int32)
    keys = (coords[:, 0].astype(np.int64) << 32) ^ (coords[:, 1].astype(np.int64) & 0xFFFFFFFF)
    order = np.lexsort((points[:, 2], keys))
    sorted_keys = keys[order]
    first = np.r_[True, sorted_keys[1:] != sorted_keys[:-1]]
    return points[order[first]]


def make_cloud_msg(points: np.ndarray, colors: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    dtype = np.dtype(
        [
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.float32),
        ]
    )
    structured = np.zeros(points.shape[0], dtype=dtype)
    if points.size:
        structured["x"] = points[:, 0]
        structured["y"] = points[:, 1]
        structured["z"] = points[:, 2]
        structured["rgb"] = colors.view(np.float32)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, structured)


class IntKeyShelf:
    def __init__(self, filename: str):
        self.db = shelve.open(filename, flag="r")

    def __getitem__(self, key: int):
        return self.db[str(int(key))]

    def close(self):
        self.db.close()


def load_ply(path):
    """Load a binary_little_endian PLY with x,y,z float + r,g,b uchar -> (Nx3 float32, N uint32 packed rgb)."""
    with open(path, "rb") as f:
        n = 0
        while True:
            line = f.readline()
            if line.startswith(b"element vertex"):
                n = int(line.split()[-1])
            if line.strip() == b"end_header":
                break
        a = np.frombuffer(f.read(n * 15), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                                 ("r", "u1"), ("g", "u1"), ("b", "u1")])
    pts = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float32)
    colors = (a["r"].astype(np.uint32) << 16) | (a["g"].astype(np.uint32) << 8) | a["b"].astype(np.uint32)
    return pts, colors


class StaticLingBotColorCloudPublisher(Node):
    def path_from_poses(self):
        """Build the keyframe Path from <map_path>/poses.npy if present (already in the map/leveled frame)."""
        path_msg = Path(); path_msg.header.frame_id = self.args.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        pp = os.path.join(self.args.map_path, "poses.npy") if self.args.map_path else None
        if pp and os.path.exists(pp):
            try:
                poses = np.load(pp, allow_pickle=True).item()
                for ts, T in poses.items():
                    path_msg.poses.append(self.pose_to_msg(np.asarray(T, np.float32), int(ts)))
            except Exception as e:
                self.get_logger().warn(f"path_from_poses: {e}")
        return path_msg

    def __init__(self, args):
        super().__init__("static_lingbot_color_cloud_publisher")
        self.args = args
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cloud_pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.path_pub = self.create_publisher(Path, args.path_topic, qos)
        self.cloud_msg = None
        self.path_msg = None
        self._last_publish = 0.0

        ply = getattr(args, "cloud_ply", None)
        if ply:
            points, colors = load_ply(ply)                       # LingBot-native map: cloud is pre-built + leveled
            path_msg = self.path_from_poses()
            self.get_logger().info(f"Loaded pre-built cloud.ply ({points.shape[0]} pts) from {ply}")
        else:
            points, colors, path_msg = self.build_map_cloud()
        self.cloud_msg = make_cloud_msg(points, colors, self.get_clock().now().to_msg(), args.frame_id)
        self.path_msg = path_msg
        self.get_logger().info(
            f"Built static LingBot color cloud with {points.shape[0]} points from {args.map_path}; "
            f"publishing {args.topic} in frame {args.frame_id}"
        )
        self.timer = self.create_timer(1.0 / max(args.publish_hz, 0.1), self.publish)

    def build_map_cloud(self) -> tuple[np.ndarray, np.ndarray, Path]:
        map_path = self.args.map_path
        poses_path = os.path.join(map_path, "poses.npy")
        intrinsics_path = os.path.join(map_path, "intrinsics.npy")
        depths_path = os.path.join(map_path, "depths")
        rgb_path = os.path.join(map_path, "rgb_images_db")
        for required in (poses_path, intrinsics_path, depths_path + ".db", rgb_path):
            if not os.path.exists(required):
                raise FileNotFoundError(required)

        poses = np.load(poses_path, allow_pickle=True).item()
        intrinsics = np.load(intrinsics_path).astype(np.float32)
        db = IntKeyShelf(depths_path)
        rgb_db = VideoDB(rgb_path, mode="read")

        all_points = []
        all_colors = []
        kept_poses = []
        timestamps = list(poses.keys())[:: self.args.keyframe_stride]
        path_msg = Path()
        path_msg.header.frame_id = self.args.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        try:
            for i, timestamp in enumerate(timestamps):
                if self.args.max_keyframes > 0 and i >= self.args.max_keyframes:
                    break
                depth = db[timestamp]
                image = rgb_db.read(timestamp)
                if image is None:
                    self.get_logger().warn(f"Skipping timestamp {timestamp}: RGB frame not found")
                    continue
                cloud_camera, colors, ground_mask = depth_rgb_to_camera_cloud(
                    depth,
                    image,
                    intrinsics,
                    self.args.pixel_step,
                    self.args.min_depth,
                    self.args.max_depth,
                    self.args.filter_ground,
                    self.args.ground_camera_y,
                )
                if cloud_camera.size == 0:
                    continue
                pose = np.asarray(poses[timestamp], dtype=np.float32)
                cloud_world = transform_points(cloud_camera, pose)
                if self.args.flatten_z is not None:
                    cloud_world[:, 2] = self.args.flatten_z
                if self.args.min_z is not None:
                    keep = cloud_world[:, 2] >= self.args.min_z
                    cloud_world = cloud_world[keep]
                    colors = colors[keep]
                if self.args.max_z is not None:
                    keep = cloud_world[:, 2] <= self.args.max_z
                    cloud_world = cloud_world[keep]
                    colors = colors[keep]
                if cloud_world.size == 0:
                    continue
                all_points.append(cloud_world.astype(np.float32, copy=False))
                all_colors.append(colors.astype(np.uint32, copy=False))
                kept_poses.append((timestamp, pose))
        finally:
            db.close()
            rgb_db.close()

        if not all_points:
            return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.uint32), path_msg
        points = np.vstack(all_points).astype(np.float32, copy=False)
        colors = np.concatenate(all_colors).astype(np.uint32, copy=False)
        # --- Gravity leveling ---------------------------------------------------------------
        # The cloud is in the VO `world` frame = camera OPTICAL frame (X-right, Y-down, Z-forward),
        # so the real "up" is ~ -Y, not +Z. RViz/REP-103 assume Z-up, which is why the raw cloud
        # looks tilted/non-parallel. The legacy z-shear (--level-mode shear) assumes Z is already up
        # and mangles a Y-up cloud. Default "rotate": RANSAC the floor and apply a rigid rotation
        # sending its normal to +Z (walls stay vertical) -- applied identically to cloud and path.
        G = np.eye(4, dtype=np.float64)
        cam_centers = (
            np.array([np.asarray(p, dtype=np.float64)[:3, 3] for _, p in kept_poses], dtype=np.float64)
            if kept_poses else np.empty((0, 3))
        )
        if self.args.level_ground and self.args.level_mode == "rotate":
            Gc = gravity_level_transform(points.astype(np.float64), cam_centers, float(self.args.level_ground_z))
            if Gc is None:
                self.get_logger().warn("Rotational leveling found no dominant floor plane; publishing un-leveled.")
            else:
                G = Gc
                points = (points.astype(np.float64) @ G[:3, :3].T + G[:3, 3]).astype(np.float32)
                cam_z = float(np.median((cam_centers @ G[:3, :3].T + G[:3, 3])[:, 2])) if cam_centers.size else float("nan")
                self.get_logger().info(
                    f"Rotational ground-leveling applied (floor z={self.args.level_ground_z:.2f}); "
                    f"camera height above floor ~ {cam_z - self.args.level_ground_z:.2f} m (expect ~0.3-0.6)."
                )
        elif self.args.level_ground and self.args.level_mode == "shear":
            ground_points = lower_envelope_points(points, self.args.ground_envelope_voxel)
            plane = fit_ground_plane(ground_points, self.args.ground_fit_points)
            if plane is None:
                self.get_logger().warn("Shear leveling: no ground candidates found; publishing un-leveled.")
            else:
                a, b, c = plane
                points[:, 2] = points[:, 2] - (a * points[:, 0] + b * points[:, 1] + c) + self.args.level_ground_z

        # Build the keyframe path in the SAME (leveled) frame as the cloud.
        for timestamp, pose in kept_poses:
            leveled = (G @ np.asarray(pose, dtype=np.float64)).astype(np.float32)
            path_msg.poses.append(self.pose_to_msg(leveled, timestamp))

        if self.args.z_offset != 0.0:
            points[:, 2] += self.args.z_offset
        points, colors = voxel_downsample(points, colors, self.args.voxel_size)
        if self.args.max_points > 0 and points.shape[0] > self.args.max_points:
            idx = np.linspace(0, points.shape[0] - 1, self.args.max_points, dtype=np.int64)
            points = points[idx]
            colors = colors[idx]
        return points, colors, path_msg

    def pose_to_msg(self, pose: np.ndarray, timestamp: int) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.args.frame_id
        msg.header.stamp.sec = int(timestamp // 1_000_000_000)
        msg.header.stamp.nanosec = int(timestamp % 1_000_000_000)
        msg.pose.position.x = float(pose[0, 3])
        msg.pose.position.y = float(pose[1, 3])
        pose_z = self.args.flatten_z if self.args.flatten_z is not None else pose[2, 3]
        msg.pose.position.z = float(pose_z + self.args.z_offset)
        quat = matrix_to_quat(pose[:3, :3])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def publish(self):
        if self.cloud_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = stamp
        self.path_msg.header.stamp = stamp
        self.cloud_pub.publish(self.cloud_msg)
        self.path_pub.publish(self.path_msg)
        now = time.monotonic()
        if now - self._last_publish > 5.0:
            self._last_publish = now
            self.get_logger().info(
                f"Published {self.args.topic} ({self.cloud_msg.width} points) and {self.args.path_topic}"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish a static RGB point cloud from a LingBot/TinyNav map.")
    parser.add_argument("--map-path", required=True)
    parser.add_argument("--cloud-ply", default=None,
                        help="LingBot-native map: load this pre-built gravity-aligned cloud.ply directly "
                        "(skips the depth-reproject rebuild, which assumes depth==rgb resolution).")
    parser.add_argument("--topic", default="/mapping/lingbot_color_cloud")
    parser.add_argument("--path-topic", default="/mapping/lingbot_keyframe_path")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--pixel-step", type=int, default=6)
    parser.add_argument("--keyframe-stride", type=int, default=2)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument(
        "--filter-ground",
        action="store_true",
        help="Drop camera-frame points with y > --ground-camera-y, matching TinyNav occupancy ground filtering.",
    )
    parser.add_argument("--ground-camera-y", type=float, default=0.0)
    parser.add_argument("--min-z", type=float, default=None)
    parser.add_argument("--max-z", type=float, default=None)
    parser.add_argument(
        "--flatten-z",
        type=float,
        default=None,
        help="Project all points onto a fixed z plane for RViz map/goal debugging.",
    )
    parser.add_argument(
        "--z-offset",
        type=float,
        default=0.0,
        help="Shift the published cloud/path upward without flattening it.",
    )
    parser.add_argument(
        "--level-ground",
        action="store_true",
        help="Fit the map floor and remove its world-frame z slope without changing x/y.",
    )
    parser.add_argument(
        "--level-ground-z",
        type=float,
        default=1.2,
        help="Target z height for the leveled ground plane.",
    )
    parser.add_argument(
        "--level-mode",
        choices=("rotate", "shear"),
        default="rotate",
        help="rotate (default): RANSAC floor + rigid rotation to Z-up (correct for the optical world frame). "
        "shear: legacy z-shear (only valid if the cloud is already Z-up).",
    )
    parser.add_argument("--ground-fit-mode", choices=("lower-envelope", "camera-y"), default="lower-envelope")
    parser.add_argument("--ground-envelope-voxel", type=float, default=0.15)
    parser.add_argument("--ground-fit-points", type=int, default=200000)
    parser.add_argument("--max-keyframes", type=int, default=0)
    parser.add_argument("--max-points", type=int, default=600000)
    parser.add_argument("--publish-hz", type=float, default=0.2)
    return parser


def main(args=None):
    parser = build_parser()
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:] if args is None else args)
    if parsed_args.pixel_step < 1:
        parser.error("--pixel-step must be >= 1")
    if parsed_args.keyframe_stride < 1:
        parser.error("--keyframe-stride must be >= 1")
    rclpy.init(args=ros_args)
    node = StaticLingBotColorCloudPublisher(parsed_args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
