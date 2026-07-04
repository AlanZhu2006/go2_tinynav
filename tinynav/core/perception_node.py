import argparse
import json
import logging
import os
import sys
import time
import cv2
import numpy as np
import rclpy
from codetiming import Timer
from cv_bridge import CvBridge
from tinynav.core.lingbot_mono_engine import LingBotMonoEngine
from tinynav.core.models_trt import LightGlueTRT, SuperPointTRT
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tinynav.core.math_utils import np2msg, np2tf, estimate_pose
from tinynav.core.math_utils import uf_init, uf_union, uf_all_sets_list
from tf2_ros import TransformBroadcaster
import asyncio
import gtsam
from collections import deque
from dataclasses import dataclass

from gtsam.symbol_shorthand import X

_N = 5
_M = 1000

_MIN_FEATURES = 20
_KEYFRAME_MIN_DISTANCE = 0.1    # unit: meter
_KEYFRAME_MIN_ROTATE_DEGREE = 0.1 # unit: degree

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def _load_mono_config():
    config_path = os.environ.get("GO2_LINGBOTNAV_CONFIG", "/home/nvidia/twork/go2_lingbotnav/config.yaml")
    try:
        import yaml
        with open(os.path.expanduser(config_path), "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception as exc:
        logger.warning("Could not load mono config %s: %s", config_path, exc)
        return {}


def keyframe_check(T_i, T_j):
    T_ij = np.linalg.inv(T_i) @ T_j
    t_diff = np.linalg.norm(T_ij[:3, 3])
    cos_theta = (np.trace(T_ij[:3, :3]) - 1) / 2
    r_diff = np.degrees(np.arccos(np.clip(cos_theta, -1, 1)))
    return t_diff > _KEYFRAME_MIN_DISTANCE or r_diff > _KEYFRAME_MIN_ROTATE_DEGREE


def Matrix4x4ToGtsamPose3(T: np.ndarray) -> gtsam.Pose3:
    return gtsam.Pose3(gtsam.Rot3(T[:3, :3]), gtsam.Point3(T[:3, 3]))

def depth_to_point(kp, depth, K):
    u, v = int(kp[0]), int(kp[1])
    Z = depth
    X = (u - K[0,2]) * Z / K[0,0]
    Y = (v - K[1,2]) * Z / K[1,1]
    return np.array([X, Y, Z])

def stamp2second(stamp):
    nano_s = np.int64(stamp.sec) * 1_000_000_000 + np.int64(stamp.nanosec)
    return nano_s * 1e-9


# keyframe dataclass
@dataclass
class Keyframe:
    timestamp: float
    image: np.ndarray
    disparity: np.ndarray
    depth: np.ndarray
    pose: np.ndarray
    velocity: np.ndarray
    bias: gtsam.imuBias.ConstantBias
    preintegrated_imu: gtsam.PreintegratedCombinedMeasurements
    latest_imu_timestamp: float
    imu_measurement_count: int = 0

class PerceptionNode(Node):
    def __init__(self, verbose_timer: bool = True):
        super().__init__("perception_node")
        self.verbose_timer = verbose_timer
        self.logger = logging.getLogger(__name__)
        # self.timer_logger = self.logger.info if verbose_timer else self.logger.debug
        # model
        self.superpoint = SuperPointTRT()
        self.light_glue = LightGlueTRT()

        self.last_keyframe_img = None
        self.last_keyframe_features = None

        mono_cfg = _load_mono_config()
        server_cfg = mono_cfg.get("perception_server", {})
        scale_cfg = mono_cfg.get("scale", {})
        host = os.environ.get("LINGBOT_HOST", server_cfg.get("host", "127.0.0.1"))
        port = int(os.environ.get("LINGBOT_PORT", server_cfg.get("port", 5599)))
        scale = float(os.environ.get("LINGBOT_SCALE", scale_cfg.get("value", 1.0)))
        self.depth_engine = LingBotMonoEngine(host=host, port=port, scale=scale)
        self.logger.info("LingBotMonoEngine connected to %s:%d with scale %.6f", host, port, scale)
        self.min_process_interval = max(0.0, float(os.environ.get("LINGBOT_NAV_MIN_INTERVAL", "0.10")))
        self.logger.info("Minimum perception interval: %.3fs", self.min_process_interval)
        # intrinsic
        self.baseline = None
        self.K = None
        self.image_shape = None

        self.T_body_last = np.eye(4)
        self.V_last = None
        self.B_last = None

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=500)

        camera_cfg = mono_cfg.get("camera", {})
        self.color_topic = camera_cfg.get("color_topic", "/camera/camera/color/image_raw")
        self.info_topic = camera_cfg.get("info_topic", "/camera/camera/color/camera_info")
        self.imu_last_received_timestamp = None
        self.camerainfo_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, self.color_topic, self.image_callback, qos_profile)
        self.odom_pub = self.create_publisher(Odometry, "/slam/odometry_visual", 10)
        self.slam_camera_info_pub = self.create_publisher(CameraInfo, "/slam/camera_info", 10)
        self.depth_pub = self.create_publisher(Image, "/slam/depth", 10)
        self.disparity_pub_vis = self.create_publisher(Image, '/slam/disparity_vis', 10)
        self.keyframe_pose_pub = self.create_publisher(Odometry, "/slam/keyframe_odom", 10)
        self.keyframe_image_pub = self.create_publisher(Image, "/slam/keyframe_image", 10)
        self.keyframe_depth_pub = self.create_publisher(Image, "/slam/keyframe_depth", 10)
        self.stats_pub = self.create_publisher(String, "/slam/data", 10)

        self.accel_readings = []
        self.last_processed_timestamp = 0.0

        self.camera_info_msg = None

        # Noise model (continuous-time)
        # for Realsense D435i
        accel_noise_density = 0.50     # [m/s^2/√Hz]
        gyro_noise_density = 0.50 # [rad/s/√Hz]
        bias_acc_rw_sigma = 0.001
        bias_gyro_rw_sigma = 0.0001
        self.pre_integration_params = gtsam.PreintegrationCombinedParams.MakeSharedU()
        self.pre_integration_params.setAccelerometerCovariance((accel_noise_density**2) * np.eye(3))
        self.pre_integration_params.setGyroscopeCovariance((gyro_noise_density**2) * np.eye(3))
        self.pre_integration_params.setIntegrationCovariance(1e-8 * np.eye(3))
        self.pre_integration_params.setBiasAccCovariance(np.eye(3) * bias_acc_rw_sigma**2)
        self.pre_integration_params.setBiasOmegaCovariance(np.eye(3) * bias_gyro_rw_sigma**2)
        self.pre_integration_params.setUse2ndOrderCoriolis(False)
        self.pre_integration_params.setOmegaCoriolis(np.array([0.0, 0.0, 0.0]))

        self.T_imu_body_to_camera = np.array(
                            [[1, 0, 0, 0],
                             [0, 0, -1, 0], 
                             [0, 1, 0, 0],
                             [0, 0, 0, 1]])

        self.imu_measurements = deque(maxlen=1000)

        self.keyframe_queue = []
        self._async_loop = asyncio.new_event_loop()
        self.logger.info("PerceptionNode initialized.")
        self.process_cnt = 0

    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.baseline = 0.0
            self.get_logger().info(f"Color camera intrinsics received from {self.info_topic}.")
            self.camera_info_msg = msg
            self.destroy_subscription(self.camerainfo_sub)

    def image_callback(self, left_msg):
        # SIL watchdog (env-gated): distinguishes "callback not invoked" from "callback slow"
        import os as _os, time as _time
        if _os.environ.get("TINYNAV_CB_WATCHDOG") == "1":
            now = _time.monotonic()
            gap = now - getattr(self, "_wd_last_entry", now)
            self._wd_last_entry = now
            if gap > 1.0:
                print(f"[cbwatchdog] callback INVOCATION gap {gap:.2f}s (executor/starvation side)", flush=True)
            if not hasattr(self, "_wd_wrap"):
                self._wd_wrap = True
        image_timestamp = stamp2second(left_msg.header.stamp)
        if image_timestamp - self.last_processed_timestamp < self.min_process_interval:
            return

        self.last_processed_timestamp = image_timestamp
        loop_start = time.perf_counter()
        with Timer(name="Perception Loop", text="[{name}] Elapsed time: {milliseconds:.0f} ms\n\n", logger=self.logger.info):
            processed = self._async_loop.run_until_complete(self.process(left_msg))
        if _os.environ.get("TINYNAV_CB_WATCHDOG") == "1":
            _dur = time.perf_counter() - loop_start
            if _dur > 0.5:
                print(f"[cbwatchdog] callback DURATION {_dur:.2f}s (slow-callback side)", flush=True)
        if processed:
            processed["stats"]["loop_ms"] = (time.perf_counter() - loop_start) * 1000.0
            self.stats_pub.publish(String(data=json.dumps(processed)))

    def destroy_node(self):
        if self._async_loop is not None:
            self._async_loop.close()
            self._async_loop = None
        return super().destroy_node()

    async def process(self, left_msg):
        if self.K is None or self.T_body_last is None:
            return {
            "stats": {"process_cnt": 0},
            "metrics": {"num_keyframes": 0, "num_tracks": 0, "num_factors": 0, "num_variables": 0, "initial_error": 0.0, "final_error": 0.0}
        }
        self.process_cnt += 1
        color_img = self.bridge.imgmsg_to_cv2(left_msg, "rgb8")
        left_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2GRAY)
        current_timestamp = stamp2second(left_msg.header.stamp)
        if len(self.keyframe_queue) == 0: # first frame
            disparity, depth = await self.depth_engine.infer(color_img, fx=float(self.K[0, 0]))
            self.keyframe_queue.append(
                Keyframe(
                    timestamp=current_timestamp,
                    image=left_img,
                    disparity=disparity,
                    depth=depth,
                    pose=self.T_body_last,
                    velocity=np.zeros(3),
                    bias=gtsam.imuBias.ConstantBias(),
                    preintegrated_imu=gtsam.PreintegratedCombinedMeasurements(self.pre_integration_params, gtsam.imuBias.ConstantBias()),
                    latest_imu_timestamp=current_timestamp
                )
            )
            return {
            "stats": {"process_cnt": 0},
            "metrics": {"num_keyframes": 0, "num_tracks": 0, "num_factors": 0, "num_variables": 0, "initial_error": 0.0, "final_error": 0.0}
        }

        with Timer(name="[Mono Depth Inference]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
            disparity, depth = await self.depth_engine.infer(color_img, fx=float(self.K[0, 0]))
            kf_prev = self.keyframe_queue[-1]
            prev_left_extract_result = await self.superpoint.infer(kf_prev.image)
            current_left_extract_result = await self.superpoint.infer(left_img)

            match_result = await self.light_glue.infer(
                prev_left_extract_result["kpts"],
                current_left_extract_result["kpts"],
                prev_left_extract_result["descps"],
                current_left_extract_result["descps"],
                prev_left_extract_result["mask"],
                current_left_extract_result["mask"],
                kf_prev.image.shape,
                left_img.shape)

        # Mono deployment intentionally drops IMU preintegration. Drift is bounded by map relocalization.

        with Timer(name="[PnP]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
        # do simple pose estimation between last keyframe and current frame
            prev_keypoints = prev_left_extract_result["kpts"][0]  # (n, 2)
            current_keypoints = current_left_extract_result["kpts"][0]  # (n, 2)
            match_indices = match_result["match_indices"][0]
            idx_to_origial = range(len(prev_keypoints))
            valid_mask = match_indices != -1
            kpt_pre = prev_keypoints[valid_mask]
            kpt_cur = current_keypoints[match_indices[valid_mask]]
            idx_valid = np.array(idx_to_origial)[valid_mask]
            logging.debug(f"match cnt: {len(kpt_pre)}")
            state, T_kf_curr, _, _, _ = estimate_pose(
                kpt_pre,
                kpt_cur,
                depth,
                self.K,
                idx_valid
            )
            self.logger.debug("Estimated T_kf_curr:\n", T_kf_curr)
        # for new frame, we first add it as keyframe, if not, we pop it later
        self.keyframe_queue.append(
            Keyframe(
                timestamp=current_timestamp,
                image=left_img,
                disparity=disparity,
                depth=depth,
                pose=self.keyframe_queue[-1].pose @ T_kf_curr,
                velocity=self.keyframe_queue[-1].velocity,
                bias=gtsam.imuBias.ConstantBias(),
                preintegrated_imu=gtsam.PreintegratedCombinedMeasurements(self.pre_integration_params, gtsam.imuBias.ConstantBias()),
                latest_imu_timestamp=current_timestamp
            )
        )
        if len(self.keyframe_queue) > _N:
            self.keyframe_queue.pop(0)
        with Timer(name="[ISAM Processing]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.info):
            with Timer(name="[adding pose priors]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                # we have new graph each time
                graph = gtsam.NonlinearFactorGraph()
                initial_estimate = gtsam.Values()
                for i, keyframe in enumerate(self.keyframe_queue[-_N:]):
                    initial_estimate.insert(X(i), Matrix4x4ToGtsamPose3(keyframe.pose))
                    sigma = np.array([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]) if i == 0 else np.array([2e-1, 2e-1, 2e-1, 2e-1, 2e-1, 2e-1])
                    graph.add(gtsam.PriorFactorPose3(X(i), Matrix4x4ToGtsamPose3(keyframe.pose), gtsam.noiseModel.Diagonal.Sigmas(sigma)))

            with Timer(name="[init extract info]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
                extract_info = [await self.superpoint.infer(kf.image) for kf in self.keyframe_queue[-_N:]]
                uf = uf_init(len(self.keyframe_queue[-_N:]) * _M)

            self.logger.debug(f"Processing {len(self.keyframe_queue)} keyframes for data association.")
            # Process pairs of keyframes from last _N keyframes: extract features (SuperPoint),
            # match by LightGlue, filter by geometric consistency (pose estimation), 
            # and build tracks via Union-Find
            with Timer(name="[cached result]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
                for i in range(max(0, len(self.keyframe_queue) - _N), len(self.keyframe_queue) - 1):
                    with Timer(name="[cached result[1/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        j = i + 1
                        kf_prev = self.keyframe_queue[i]
                        kf_curr = self.keyframe_queue[j]

                    self.logger.debug("timestamp prev: ", kf_prev.timestamp)
                    self.logger.debug("timestamp curr: ", kf_curr.timestamp)
                    with Timer(name="[cached result[1.1/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        prev_left_extract_result = await self.superpoint.infer(kf_prev.image)
                    with Timer(name="[cached result[1.2/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        current_left_extract_result = await self.superpoint.infer(kf_curr.image)

                    with Timer(name="[cached result[1.3/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        match_result = await self.light_glue.infer(
                            prev_left_extract_result["kpts"],
                            current_left_extract_result["kpts"],
                            prev_left_extract_result["descps"],
                            current_left_extract_result["descps"],
                            prev_left_extract_result["mask"],
                            current_left_extract_result["mask"],
                            kf_prev.image.shape,
                            kf_curr.image.shape,
                        )
                    with Timer(name="[cached result[2/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        prev_keypoints = prev_left_extract_result["kpts"][0]  # (n, 2)
                        current_keypoints = current_left_extract_result["kpts"][0]  # (n, 2)
                        match_indices = match_result["match_indices"][0].copy()
                        idx_to_origial = range(len(prev_keypoints))

                        valid_mask = match_indices != -1
                        kpt_pre = prev_keypoints[valid_mask]
                        kpt_cur = current_keypoints[match_indices[valid_mask]]
                        idx_valid = np.array(idx_to_origial)[valid_mask]

                        depth = kf_curr.depth

                        logging.debug(f"match cnt: {len(kpt_pre)}")
                        state, _, _, _, inliers = estimate_pose(
                            kpt_pre,
                            kpt_cur,
                            depth,
                            self.K,
                            idx_valid
                        )
                        inlier_set = set(inliers)
                        if len(inlier_set) > 20:
                            for idx in range(len(match_indices)):
                                if idx not in inlier_set:
                                    match_indices[idx] = -1
                        else:
                            for idx in range(len(match_indices)):
                                match_indices[idx] = -1
                            self.logger.warning(f"match cnt: {len(kpt_pre)} is too small, {len(inlier_set)} inliers.")

                    with Timer(name="[cached result[3/3]]", text="[{name}] Elapsed time: {milliseconds:.03f} ms", logger=self.logger.debug):
                        count = 0
                        for k, match_idx in enumerate(match_indices):
                            if match_idx != -1:
                                idx_prev = i * _M + k
                                idx_curr = j * _M + match_idx
                                uf_union(idx_prev, idx_curr, uf)
                                count += 1
                        self.logger.debug(f"{i} match {j} after Pnp filter count: {count}")

            with Timer(name="[found track]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
                tracks = uf_all_sets_list(uf, min_component_size=2)
                self.logger.debug(f"Found {len(tracks)} tracks after data association.")

            with Timer(name="[add track]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
                # Simplest mono path: drop the stereo smart projection factor. Depth-based PnP
                # supplies local odometry; map_node HLoc relocalization supplies global correction.
                pass

        with Timer(name="[Solver]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
            params = gtsam.LevenbergMarquardtParams()
            # set iteration limit
            params.setMaxIterations(3)
            params.setVerbosityLM("DEBUG")
            lm = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
            result = lm.optimize()

            self.logger.info(f"ISAM optimization done with {graph.size()} factors and {initial_estimate.size()} variables.")
            self.logger.info(f"Initial error: {graph.error(initial_estimate):.4f}, Final error: {graph.error(result):.4f}")

            for i, keyframe in enumerate(self.keyframe_queue[-_N:]):
                T_i = result.atPose3(X(i)).matrix()
                keyframe.pose = T_i
                self.logger.debug(f"Keyframe {i} pose updated:\n{T_i}, at timestamp {keyframe.timestamp}")

        with Timer(text="[Depth as Color] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
            valid_depth = depth[np.isfinite(depth) & (depth > 0)]
            max_depth = np.percentile(valid_depth, 95) if valid_depth.size else 5.0
            disp_vis = np.clip(depth / max(max_depth, 1e-3) * 255.0, 0, 255).astype(np.uint8)
            disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_PLASMA)
            disp_color_msg = self.bridge.cv2_to_imgmsg(disp_color, encoding='bgr8')
            disp_color_msg.header = left_msg.header
            self.disparity_pub_vis.publish(disp_color_msg)

        with Timer(name='[Depth as Cloud', text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
            # publish depth image and camera info for depth topic (required by DepthCloud)
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header.stamp = left_msg.header.stamp
            depth_msg.header.frame_id = "camera"  # Match TF frame
            self.camera_info_msg.header.stamp = left_msg.header.stamp
            self.camera_info_msg.header.frame_id = "camera"  # Match TF frame
            self.slam_camera_info_pub.publish(self.camera_info_msg)
            self.depth_pub.publish(depth_msg)
        self.logger.debug(f"superpoint cache info: {self.superpoint.infer.cache_info()}")
        self.logger.debug(f"lightglue cache info: {self.light_glue.infer.cache_info()}")
        self.logger.debug(f"estimate_pose cache info: {estimate_pose.cache_info()}")

        with Timer(name="[Publish Odometry]", text="[{name}] Elapsed time: {milliseconds:.0f} ms", logger=self.logger.debug):
            self.T_body_last = result.atPose3(X(len(self.keyframe_queue) - 1)).matrix()
            self.V_last = self.keyframe_queue[-1].velocity
            # publish odometry
            self.odom_pub.publish(np2msg(self.T_body_last, left_msg.header.stamp, "world", "camera", self.V_last))
            # publish TF
            self.tf_broadcaster.sendTransform(np2tf(self.T_body_last, left_msg.header.stamp, "world", "camera"))

            last_keyframe = self.keyframe_queue[-2]
            current_keyframe = self.keyframe_queue[-1]
            if keyframe_check(last_keyframe.pose, current_keyframe.pose) or current_keyframe.timestamp - last_keyframe.timestamp > 3.0:
                self.keyframe_pose_pub.publish(np2msg(current_keyframe.pose, left_msg.header.stamp, "world", "camera", current_keyframe.velocity))
                self.keyframe_image_pub.publish(left_msg)
                self.keyframe_depth_pub.publish(depth_msg)
                self.logger.info(
                    "Published keyframe #%d at %.3f",
                    len(self.keyframe_queue),
                    current_keyframe.timestamp,
                )
            else:
                self.keyframe_queue.pop()

        return {
            "stats": {
                "process_cnt": self.process_cnt,
            },
            "metrics": {
                "num_keyframes": len(self.keyframe_queue),
                "num_tracks": len(tracks),
                "num_factors": graph.size(),
                "num_variables": initial_estimate.size(),
                "initial_error": graph.error(initial_estimate),
                "final_error": graph.error(result),
            },
        }


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Run TinyNav perception node.')
    parser.add_argument('--verbose_timer', action='store_true', help='Print timing for key pipeline stages.')
    parsed_args = parser.parse_args(args=sys.argv[1:] if args is None else args)

    perception_node = PerceptionNode(verbose_timer=parsed_args.verbose_timer)

    # SingleThreaded: rclpy MultiThreadedExecutor has a known load-dependent starvation bug
    # (ready subscriptions skipped for seconds). Only 2 subs here; FIFO is what we want.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(perception_node)
    executor.spin()
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
