import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
import numpy as np
import logging
import time
import os

# Module-level logger for cases where self.get_logger() is not available
logger = logging.getLogger(__name__)

class CmdVelControlNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_control_node')
        self.logger = self.get_logger()  # Use ROS2 logger
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/slam/odometry', self.pose_callback, 10)
        self.pose_visual_sub = self.create_subscription(Odometry, '/slam/odometry_visual', self.pose_callback, 10)
        self.create_subscription(Path, '/planning/trajectory_path', self.path_callback, 10)
        self.T_robot_to_camera = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]]
        )
        self.last_path_time = 0.0
        self.pose = None
        self.path = None

        # === Control loop (ported from planning_node_compare style) ===
        # Planner input is typically 7-10 Hz; over-driving cmd publish rate amplifies jitter.
        self.cmd_rate_hz = 12.0
        # Use minima; actual stale thresholds are scaled by observed planner period.
        self.path_stale_slow_s = 0.35
        self.path_stale_stop_s = 0.8
        self.path_stale_slow_factor = 3.5
        self.path_stale_stop_factor = 5.0
        self.max_linear_acc = 0.6   # m/s^2
        self.max_angular_acc = 0.8  # rad/s^2
        self.max_angular_speed = 0.70  # rad/s; match TinyNav/Go2 bridge yaw cap while keeping stale-pose guards.
        self.max_inplace_turn_speed = 0.35
        self.planner_dt = 0.1       # trajectory dt in planning_node
        # planning_node publishes path with for j in range(..., step=10), so points are ~1.0 s apart.
        self.path_pose_stride = 10
        self.path_period_ema = 0.12
        self.path_filter_tau = 0.30
        self.lookahead_steps = 1
        # Static-friction compensation: very small vx often cannot move the robot.
        self.min_effective_linear_speed = 0.1
        self.min_effective_angular_speed = 0.1
        self.linear_engage_threshold = 0.04
        self.fixed_reverse_speed = 0.2
        # Hack: if path first segment points far away from robot heading,
        # rotate in place instead of publishing near-zero cmd_vel.
        self.force_turn_heading_threshold = np.deg2rad(75.0)

        # SAFE-CAPS for closed-loop over slow (~1Hz) perception:
        self.pose_stale_stop_s = 0.8      # no cmd if localization has not updated within this
        self.reloc_jump_thresh = 0.4      # m step between consecutive poses = reloc jump
        self.reloc_freeze_s = 0.6         # freeze cmd this long after a reloc jump
        self.last_pose_mono = None
        self.prev_pose_xy = None
        self.reloc_freeze_until = 0.0
        # ARRIVAL braking: decelerate + stop near the goal so ~1Hz latency does not overshoot into walls.
        # ARRIVAL braking on the FINAL-goal distance (map_node /control/goal_distance), NOT the 2.5m
        # rolling lookahead (/control/target_pose). Stale-aware.
        self.goal_dist = None
        self.goal_dist_time = 0.0
        self.arrival_radius = float(os.environ.get("TINYNAV_CONTROL_ARRIVAL_RADIUS_M", "0.0"))
        self.arrival_gain = 0.7
        self.create_subscription(Float32, '/control/goal_distance', self._goal_dist_cb, 10)
        # RELOC-LOSS stop: if map localization goes stale, do not drive blind on a stale map-frame path.
        self.last_reloc_mono = None
        # Adaptive floor: reloc arrives per KEYFRAME (3s when slow/static), so a fixed 4.0s gate
        # sits ON the cadence boundary — any borderline reloc miss freezes the robot until the
        # next hit (SIL 2026-07-04: drive-freeze chatter, the real dog's stop-and-go signature).
        self.reloc_stale_stop_s = 4.0
        self.reloc_period_ema = None
        self.reloc_search_enabled = os.environ.get("TINYNAV_RELOC_SEARCH", "1") != "0"
        self.reloc_search_max_s = float(os.environ.get("TINYNAV_RELOC_SEARCH_MAX_S", "30.0"))
        self.reloc_search_yaw = float(os.environ.get("TINYNAV_RELOC_SEARCH_YAW", "0.25"))
        self.reloc_search_flip_s = float(os.environ.get("TINYNAV_RELOC_SEARCH_FLIP_S", "8.0"))
        self.reloc_search_start = None
        self.create_subscription(Odometry, '/map/relocalization', self._reloc_cb, 10)
        # Reactive forward E-STOP from LIVE depth (reloc-independent): stop if a wall is close ahead.
        self._bridge = CvBridge()
        self.front_clear = None
        self.front_clear_time = 0.0
        self.front_stop_dist = 0.5
        self.create_subscription(Image, '/slam/depth', self._depth_cb, 5)

        self.latest_cmd = Twist()
        self.prev_cmd = Twist()
        self.last_cmd_pub_time = time.monotonic()
        self.last_path_update_time = None
        self._paused = False
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/nav/paused', self._on_paused, _latched_qos)
        self.cmd_timer = self.create_timer(1.0 / self.cmd_rate_hz, self.cmd_timer_callback)

    def _on_paused(self, msg: Bool):
        self._paused = msg.data
        if not self._paused:
            # Reset prev_cmd so resume starts from zero cleanly
            self.prev_cmd = Twist()

    def _goal_dist_cb(self, msg):
        self.goal_dist = float(msg.data); self.goal_dist_time = time.monotonic()

    def _reloc_cb(self, msg):
        now = time.monotonic()
        if self.last_reloc_mono is not None:
            period = now - self.last_reloc_mono
            self.reloc_period_ema = period if self.reloc_period_ema is None else 0.8 * self.reloc_period_ema + 0.2 * period
        self.last_reloc_mono = now
        self.reloc_search_start = None

    def _depth_cb(self, msg):
        try:
            d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        d = np.asarray(d, dtype=np.float32)
        h, w = d.shape[:2]
        roi = d[int(h*0.30):int(h*0.90), int(w*0.30):int(w*0.70)]
        valid = roi[(roi > 0.1) & (roi < 10.0)]
        self.front_clear = float(np.percentile(valid, 5)) if valid.size > 50 else None
        self.front_clear_time = time.monotonic()

    def pose_callback(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        xy = np.array([p.x, p.y, p.z])
        if (self.prev_pose_xy is not None and self.last_pose_mono is not None
                and (now - self.last_pose_mono) < 0.5
                and np.linalg.norm(xy - self.prev_pose_xy) > self.reloc_jump_thresh):
            self.reloc_freeze_until = now + self.reloc_freeze_s  # reloc jumped -> briefly freeze cmd
        self.prev_pose_xy = xy
        self.last_pose_mono = now
        self.pose = msg

    def _clamp_step(self, target: float, current: float, max_delta: float) -> float:
        return float(np.clip(target - current, -max_delta, max_delta) + current)

    def cmd_timer_callback(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.last_cmd_pub_time)
        self.last_cmd_pub_time = now
        # SIL gate telemetry
        if not hasattr(self, "_gate_n"):
            self._gate_n = {}; self._gate_t0 = now
        def _gate(name):
            self._gate_n[name] = self._gate_n.get(name, 0) + 1
        if now - self._gate_t0 > 15.0:
            print(f"[gatedbg] {self._gate_n}", flush=True)
            self._gate_n = {}; self._gate_t0 = now

        if self._paused:
            _gate("paused")
            self.cmd_pub.publish(Twist())
            self.prev_cmd = Twist()
            return

        # SAFE-CAP: perception stalled or reloc jumped -> do not drive on a stale/jumping pose.
        if (self.last_pose_mono is None or (now - self.last_pose_mono) > self.pose_stale_stop_s
                or now < self.reloc_freeze_until):
            _gate("pose_stale_or_jump")
            self.cmd_pub.publish(Twist())
            self.prev_cmd = Twist()
            return

        # RELOC-LOSS stop: map localization stale -> do not drive blind on a stale map-frame path.
        reloc_stop = self.reloc_stale_stop_s if self.reloc_period_ema is None else max(self.reloc_stale_stop_s, 2.5 * self.reloc_period_ema)
        if self.last_reloc_mono is not None and (now - self.last_reloc_mono) > reloc_stop:
            if self.reloc_search_enabled:
                if self.reloc_search_start is None:
                    self.reloc_search_start = now
                search_age = now - self.reloc_search_start
                if search_age < self.reloc_search_max_s:
                    _gate("reloc_search")
                    out = Twist()
                    phase = int(search_age / max(self.reloc_search_flip_s, 1e-3))
                    direction = 1.0 if phase % 2 == 0 else -1.0
                    out.angular.z = float(direction * min(abs(self.reloc_search_yaw), self.max_inplace_turn_speed))
                    self.cmd_pub.publish(out)
                    self.prev_cmd = out
                    return
            _gate("reloc_stale")
            self.cmd_pub.publish(Twist())
            self.prev_cmd = Twist()
            return

        # Stale-path protection: slow down, then stop if planner has not refreshed.
        age = float('inf') if self.last_path_update_time is None else (now - self.last_path_update_time)
        stale_slow_s = max(self.path_stale_slow_s, self.path_period_ema * self.path_stale_slow_factor)
        stale_stop_s = max(self.path_stale_stop_s, self.path_period_ema * self.path_stale_stop_factor)
        target_cmd = Twist()
        target_cmd.linear.x = self.latest_cmd.linear.x
        target_cmd.angular.z = self.latest_cmd.angular.z
        if age > stale_stop_s:
            _gate("path_stale_stop")
            target_cmd.linear.x = 0.0
            target_cmd.angular.z = 0.0
        elif age > stale_slow_s:
            _gate("path_stale_slow")
            target_cmd.linear.x *= 0.3
            target_cmd.angular.z *= 0.5

        # Reactive forward E-STOP (reloc-independent): wall close ahead -> zero forward, allow turning.
        if (self.front_clear is not None and (now - self.front_clear_time) < 0.8
                and self.front_clear < self.front_stop_dist):
            target_cmd.linear.x = min(target_cmd.linear.x, 0.0)

        out = Twist()
        out.linear.y = 0.0

        # Reverse is a predefined planner vocabulary: straight back at fixed speed.
        # Do not smooth or re-lock it here; just pass it through while stale/paused guards still work.
        if target_cmd.linear.x < 0.0:
            target_cmd.linear.x = 0.0  # SAFE-CAP: never reverse (fall through to forward/turn handling)

        # Forward/turning commands still get acceleration limiting and robot minimum-speed locks.
        max_dv = self.max_linear_acc * dt
        # If we just left reverse mode, do not let acceleration limiting leak another reverse command.
        prev_linear_x = 0.0 if self.prev_cmd.linear.x < 0.0 else self.prev_cmd.linear.x
        out.linear.x = self._clamp_step(target_cmd.linear.x, prev_linear_x, max_dv)
        # Do not acceleration-limit yaw. The planner/control layer already decides the turn rate,
        # and forced rotate-in-place should take effect immediately.
        out.angular.z = float(np.clip(target_cmd.angular.z, -self.max_angular_speed, self.max_angular_speed))

        # Linear x: robot cannot execute tiny non-zero speeds reliably.
        # When engaging forward motion, snap to +min; when stopping/decaying, snap to 0.
        if 0.0 < out.linear.x < self.min_effective_linear_speed:
            out.linear.x = self.min_effective_linear_speed if target_cmd.linear.x >= self.linear_engage_threshold else 0.0
        elif abs(out.linear.x) < self.min_effective_linear_speed:
            out.linear.x = 0.0

        # Angular z: same idea; tiny requested turns snap to executable min, decays snap to 0.
        if 0.0 < abs(out.angular.z) < self.min_effective_angular_speed:
            if abs(target_cmd.angular.z) >= self.min_effective_angular_speed:
                out.angular.z = float(np.sign(target_cmd.angular.z) * self.min_effective_angular_speed)
            else:
                out.angular.z = 0.0

        self.cmd_pub.publish(out)
        self.prev_cmd = out
        
    def path_callback(self, msg):
        if msg is None or self.pose is None:
            return
        if len(msg.poses) < 2:
            return
        self.path = msg

        ros_now = self.get_clock().now().to_msg()
        self.last_path_time = ros_now.sec + ros_now.nanosec * 1e-9
        now_mono = time.monotonic()
        if self.last_path_update_time is not None:
            period = np.clip(now_mono - self.last_path_update_time, 0.05, 0.5)
            self.path_period_ema = 0.85 * self.path_period_ema + 0.15 * float(period)
        self.last_path_update_time = now_mono

        def msg2np(msg):
            T = np.eye(4)
            position = msg.pose.position
            rot = msg.pose.orientation
            quat = [rot.x, rot.y, rot.z, rot.w]
            T[:3, :3] = R.from_quat(quat).as_matrix()
            T[:3, 3] = np.array([position.x, position.y, position.z]).ravel()
            return T
        
        T1 = msg2np(self.path.poses[0])
        step_idx = int(min(self.lookahead_steps, len(self.path.poses) - 1))
        T2 = msg2np(self.path.poses[step_idx])
        T_robot_1 = T1 @ self.T_robot_to_camera
        T_robot_2 = T2 @ self.T_robot_to_camera
        T_robot_2_to_1 = np.linalg.inv(T_robot_1) @ T_robot_2
        p = T_robot_2_to_1[:3, 3]
        heading_err = float(np.arctan2(p[1], p[0]))
        # dt must match actual spacing between published Path poses, not raw trajectory dt.
        dt = self.planner_dt * self.path_pose_stride * max(1, step_idx)
        linear_velocity_vec = p / dt
        r = R.from_matrix(T_robot_2_to_1[:3, :3])
        angular_velocity_vec = r.as_rotvec() / dt

        raw_vx = float(linear_velocity_vec[0])
        # SAFE-CAP: forbid reverse. If the target is behind, rotate to face it (the force-turn gate
        # below fires on the large heading error) instead of backing up blindly.
        vx = float(np.clip(raw_vx, 0.0, 0.5))
        vy = 0.0
        vyaw = np.clip(angular_velocity_vec[2], -self.max_angular_speed, self.max_angular_speed)
        is_backward_segment = False

        # Hack: if path first segment points >80 deg away from robot heading,
        # force an in-place turn. Skip explicit backward segments because reverse
        # naturally has heading_err close to +/-pi.
        # ARC toward the goal instead of spin-then-go: keep forward progress while gently correcting
        # heading. Only spin in place when the goal is nearly BEHIND. Robust to the slow (~1Hz) laggy
        # feedback that made pure rotate-in-place overshoot and hunt forever.
        if (not is_backward_segment) and abs(heading_err) > self.force_turn_heading_threshold:
            vx = 0.0
            vyaw = float(np.clip(heading_err, -self.max_inplace_turn_speed, self.max_inplace_turn_speed))
        else:
            vx = float(vx * max(0.0, np.cos(heading_err)))   # full speed aligned -> 0 near 90deg
            vyaw = float(np.clip(1.5 * heading_err, -self.max_angular_speed, self.max_angular_speed))
            if abs(heading_err) < np.deg2rad(8.0):           # deadband: stop hunting tiny heading errors
                vyaw = 0.0

        vyaw = float(np.clip(vyaw, -self.max_angular_speed, self.max_angular_speed))

        # Store the latest target command directly. Smoothing is intentionally kept
        # only in cmd_timer_callback via acceleration limiting, so planner/control
        # behavior stays easy to reason about during tuning.
        if self.arrival_radius > 0.0 and self.goal_dist is not None and (time.monotonic() - self.goal_dist_time) < 1.0:
            if self.goal_dist < self.arrival_radius:
                vx = 0.0; vyaw = 0.0   # arrived: stop
            else:
                vx = float(min(vx, self.arrival_gain * (self.goal_dist - self.arrival_radius)))
        self.latest_cmd.linear.x = float(vx)
        self.latest_cmd.linear.y = float(vy)
        self.latest_cmd.angular.z = float(vyaw)
        age = 0.0 if self.last_path_update_time is None else (time.monotonic() - self.last_path_update_time)
        self.logger.debug(
            f"cmd vx={self.latest_cmd.linear.x:.3f} vyaw={self.latest_cmd.angular.z:.3f} "
            f"path_age={age:.2f}s path_dt_ema={self.path_period_ema:.2f}s lookahead={step_idx}"
        )

    def destroy_node(self):
        self.logger.info("Destroying cmd_vel_control connection.")
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    
    node = CmdVelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
