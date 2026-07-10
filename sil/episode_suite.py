"""SIL acceptance suite (overnight goal G2): N teleport->localize->goal->score episodes against
ground truth, with freeze/duty-cycle accounting. Runs INSIDE the container.

Success: GT position within GOAL_TOL of goal_gt before EP_TIMEOUT. Freeze: no GT movement >0.05m
for FREEZE_S while a goal is active and not yet reached.
Writes /tmp/claude-1000/sil_episode_results.json after each episode (crash-safe).

Run: python3 sil/episode_suite.py /tmp/claude-1000/sil_episodes_spec.json
"""
import json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Bool

SPEC = json.load(open(sys.argv[1]))["episodes"]
if len(sys.argv) > 2:                      # --single N: one episode per process (host orchestrates
    SPEC = [SPEC[int(sys.argv[2])]]        # node restarts between episodes = clean VO per episode)
OUT = "/tmp/claude-1000/sil_episode_results.json"
GOAL_TOL = 0.45
EP_TIMEOUT = 900.0
LOC_TIMEOUT = 240.0
FREEZE_S = 240.0   # stop-and-go duty ~30%: bursts arrive, don't kill them early

rclpy.init()
node = Node("episode_suite")
state = {
    "gt": None,
    "reloc_t": 0.0,
    "path_t": 0.0,
    "odom_t": 0.0,
    "cmd_nonzero": 0,
    "cmd_total": 0,
    "cmd_nonzero_t": 0.0,
    "paused": False,
    "paused_t": 0.0,
}
node.create_subscription(PoseStamped, "/sim/gt_pose",
                         lambda m: state.__setitem__("gt", np.array([m.pose.position.x, m.pose.position.y])), 10)
node.create_subscription(Odometry, "/map/relocalization",
                         lambda m: state.__setitem__("reloc_t", time.monotonic()), 10)
node.create_subscription(Path, "/planning/trajectory_path",
                         lambda m: state.__setitem__("path_t", time.monotonic()), 10)
node.create_subscription(Odometry, "/slam/odometry_visual",
                         lambda m: state.__setitem__("odom_t", time.monotonic()), 10)
def paused_cb(m):
    state["paused"] = bool(m.data)
    state["paused_t"] = time.monotonic()
_latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(Bool, "/nav/paused", paused_cb, _latched_qos)
def cmd_cb(m):
    state["cmd_total"] += 1
    if abs(m.linear.x) > 0.02 or abs(m.angular.z) > 0.02:
        state["cmd_nonzero"] += 1
        state["cmd_nonzero_t"] = time.monotonic()
node.create_subscription(Twist, "/cmd_vel", cmd_cb, 10)
pub_reset = node.create_publisher(PoseStamped, "/sim/reset_pose", 5)
pub_paused = node.create_publisher(Bool, "/nav/paused", _latched_qos)
pub_pois = node.create_publisher(String, "/mapping/cmd_pois", 5)

def spin(sec):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.1)

def age(now, stamp):
    return None if stamp <= 0.0 else round(now - stamp, 3)

def freeze_cause(now):
    reloc_age = now - state["reloc_t"] if state["reloc_t"] > 0.0 else float("inf")
    path_age = now - state["path_t"] if state["path_t"] > 0.0 else float("inf")
    odom_age = now - state["odom_t"] if state["odom_t"] > 0.0 else float("inf")
    cmd_age = now - state["cmd_nonzero_t"] if state["cmd_nonzero_t"] > 0.0 else float("inf")
    if state["paused"]:
        return "paused_freeze"
    if odom_age > 2.0:
        return "pose_stale_freeze"
    if reloc_age > 8.0:
        return "reloc_stale_freeze"
    if path_age > 3.0:
        return "path_stale_freeze"
    if cmd_age > 10.0:
        return "cmd_zero_freeze"
    return "no_gt_motion_freeze"

def diag(now):
    return {
        "paused": bool(state["paused"]),
        "reloc_age_s": age(now, state["reloc_t"]),
        "path_age_s": age(now, state["path_t"]),
        "odom_age_s": age(now, state["odom_t"]),
        "cmd_nonzero_age_s": age(now, state["cmd_nonzero_t"]),
    }

results = []
for ep in SPEC:
    print(f"=== episode {ep['ep']} ===", flush=True)
    ep_result = {"ep": ep["ep"]}
    # clear goal, teleport, settle
    pub_pois.publish(String(data="{}"))
    spin(2)
    rp = PoseStamped()
    rp.pose.position.x, rp.pose.position.y, rp.pose.position.z = ep["start_h"]
    rp.pose.orientation.z = ep["yaw"]
    # freeze the base during setup: stale planner carrots drove the robot around DURING the
    # localization wait (motion predating goal-send = phantom instant successes). map_node
    # publishes paused=False when the new goal arrives (existing handoff).
    pub_paused.publish(Bool(data=True))
    pub_reset.publish(rp)
    # verify the teleport landed; gt topic convention is (x_h, -z_h)
    want = np.array([ep["start_h"][0], -ep["start_h"][2]])
    for _ in range(10):
        spin(2)
        if state["gt"] is not None and np.linalg.norm(np.array(state["gt"][:2]) - want) < 0.5:
            break
        pub_reset.publish(rp)
    else:
        print(f"ep{ep['ep']}: TELEPORT_NOT_CONFIRMED gt={state['gt']}", flush=True)
        ep_result["teleport_not_confirmed"] = True
    # wait localization: fresh reloc messages arriving
    t0 = time.monotonic()
    ok_loc = False
    while time.monotonic() - t0 < LOC_TIMEOUT:
        spin(2)
        if time.monotonic() - state["reloc_t"] < 6.0:
            ok_loc = True
            break
    if not ok_loc:
        now = time.monotonic()
        d_final = float(np.linalg.norm(state["gt"] - np.array(ep["goal_gt"][:2]))) if state["gt"] is not None else -1
        print(f"ep{ep['ep']}: LOCALIZATION_TIMEOUT final_goal_dist={d_final:.2f}", flush=True)
        ep_result.update(
            outcome="loc_timeout",
            final_dist=d_final,
            freeze_cause="loc_timeout",
            start_h=ep["start_h"],
            goal_gt=ep["goal_gt"][:2],
            final_gt=state["gt"].tolist() if state["gt"] is not None else None,
            **diag(now),
        )
        results.append(ep_result)
        json.dump(results, open(OUT, "w"), indent=1)
        continue
    spin(8)   # let 3-vote/IRLS settle
    # send goal
    g = ep["goal_map"]
    # resend: volatile pub + freshly-restarted map_node = the single goal message can be lost
    # in the DDS rematch window (root cause of whole-episode paused freezes)
    for _ in range(3):
        pub_pois.publish(String(data=json.dumps({"0": {"position": g}})))
        spin(4)
    # release our own latched pause: with TWO transient-local publishers on /nav/paused the
    # subscriber may get our stale True after map_node's False (latched multi-pub footgun)
    pub_paused.publish(Bool(data=False))
    goal_gt = np.array(ep["goal_gt"][:2])
    t0 = time.monotonic()
    state["cmd_nonzero"] = 0; state["cmd_total"] = 0
    state["cmd_nonzero_t"] = 0.0
    last_move_t = time.monotonic()
    last_pos = state["gt"].copy() if state["gt"] is not None else None
    start_gt = state["gt"].copy() if state["gt"] is not None else None
    path_len = 0.0
    start_goal_d = float(np.linalg.norm(state["gt"] - goal_gt)) if state["gt"] is not None else 0.0
    outcome = "timeout"
    outcome_cause = None
    while time.monotonic() - t0 < EP_TIMEOUT:
        spin(2)
        if state["gt"] is None:
            continue
        if last_pos is None or np.linalg.norm(state["gt"] - last_pos) > 0.05:
            if last_pos is not None:
                path_len += float(np.linalg.norm(state["gt"] - last_pos))
            last_pos = state["gt"].copy()
            last_move_t = time.monotonic()
        d = float(np.linalg.norm(state["gt"] - goal_gt))
        # success requires ACTUAL DRIVING: cumulative GT path >= 70% of the start-goal gap
        # (kills every phantom-success class: stale GT, near-start goals, setup drift)
        if d < GOAL_TOL and path_len >= 0.7 * start_goal_d:
            outcome = "SUCCESS"
            break
        if time.monotonic() - last_move_t > FREEZE_S:
            outcome = "freeze"
            outcome_cause = freeze_cause(time.monotonic())
            break
    if outcome == "timeout":
        outcome_cause = "episode_timeout"
    duty = state["cmd_nonzero"] / max(state["cmd_total"], 1)
    d_final = float(np.linalg.norm(state["gt"] - goal_gt)) if state["gt"] is not None else -1
    now = time.monotonic()
    cause_text = f" freeze_cause={outcome_cause}" if outcome_cause else ""
    print(f"ep{ep['ep']}: {outcome} final_goal_dist={d_final:.2f} duty={duty:.2f} path={path_len:.1f}m start_goal_dist={start_goal_d:.2f}{cause_text} t={now-t0:.0f}s", flush=True)
    ep_result.update(
        outcome=outcome,
        final_dist=d_final,
        duty=round(duty, 3),
        path_len=round(path_len, 3),
        start_goal_dist=round(start_goal_d, 3),
        freeze_cause=outcome_cause,
        start_h=ep["start_h"],
        start_gt=start_gt.tolist() if start_gt is not None else None,
        goal_gt=goal_gt.tolist(),
        final_gt=state["gt"].tolist() if state["gt"] is not None else None,
        last_move_age_s=round(now - last_move_t, 3),
        **diag(now),
    )
    results.append(ep_result)
    json.dump(results, open(OUT, "w"), indent=1)

n_ok = sum(1 for r in results if r.get("outcome") == "SUCCESS")
print(f"SUITE DONE: {n_ok}/{len(results)} success", flush=True)
