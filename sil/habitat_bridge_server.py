"""SIL bridge, HOST side: Habitat renderer behind a TCP socket (runs in the `habitat` conda env).

The containerized ROS graph talks to this over localhost — same len-prefixed-pickle framing as the
LingBot depth server. Kinematic Go2: velocity integration, navmesh-blocked moves count collisions.

Protocol (one client):
  {"reset": {"pos": [x,y,z] habitat|null, "yaw": rad}}            -> {"ok", "pos", "yaw", "K"}
  {"step":  {"vx": m/s, "wz": rad/s, "dt": s}}                    -> frame dict
  {"render": {}}                                                  -> frame dict (no motion)
frame dict: {"jpg": bytes, "gt_pose": 4x4 c2w z-up, "pos": [xyz]h, "yaw": rad, "collided": bool}

Run: conda run -n habitat python sil/habitat_bridge_server.py [--scene GLB] [--port 5601]
"""
import argparse, math, pickle, socket, struct
import numpy as np
import cv2
import habitat_sim

W, H, HFOV, CAMH = 848, 480, 70.0, 0.33
Mh2p = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="/tmp/claude-1000/habitat_data/versioned_data/habitat_test_scenes/apartment_1.glb")
ap.add_argument("--port", type=int, default=5601)
a = ap.parse_args()

cfg = habitat_sim.SimulatorConfiguration(); cfg.scene_id = a.scene
rgb = habitat_sim.CameraSensorSpec()
rgb.uuid = "rgb"; rgb.sensor_type = habitat_sim.SensorType.COLOR
rgb.resolution = [H, W]; rgb.hfov = HFOV; rgb.position = [0.0, CAMH, 0.0]
ac = habitat_sim.agent.AgentConfiguration(); ac.sensor_specifications = [rgb]
sim = habitat_sim.Simulator(habitat_sim.Configuration(cfg, [ac]))
pf = sim.pathfinder; ag = sim.get_agent(0)
fx = (W / 2) / math.tan(math.radians(HFOV / 2))
K = [[fx, 0.0, W / 2], [0.0, fx, H / 2], [0.0, 0.0, 1.0]]

state = {"pos": None, "yaw": 0.0}
# simulated auto-exposure (matches exp70's recorder so live frames look like the map's frames)
ae = {"gain": 1.0}

def place(pos, yaw):
    st = habitat_sim.AgentState()
    st.position = np.array(pos, np.float32)
    st.rotation = np.quaternion(math.cos(yaw / 2), 0, math.sin(yaw / 2), 0)
    ag.set_state(st, reset_sensors=True)
    state["pos"], state["yaw"] = np.array(pos, float), float(yaw)

def frame(collided=False):
    obs = sim.get_sensor_observations()
    bgr = cv2.cvtColor(obs["rgb"][:, :, :3], cv2.COLOR_RGB2BGR)
    m = float(bgr.mean())
    g_t = min(6.0, max(1.0, 100.0 / max(m, 1e-3)))
    ae["gain"] = 0.9 * ae["gain"] + 0.1 * g_t
    bgr = np.clip(bgr.astype(np.float32) * ae["gain"], 0, 255).astype(np.uint8)
    ok, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    yaw = state["yaw"]; pos = state["pos"]
    Rh = np.array([[math.cos(yaw), 0, math.sin(yaw)], [0, 1, 0], [-math.sin(yaw), 0, math.cos(yaw)]])
    Rcam = Rh @ np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = Mh2p @ Rcam
    T[:3, 3] = Mh2p @ (pos + np.array([0, CAMH, 0]))
    return {"jpg": enc.tobytes(), "gt_pose": T.tolist(), "pos": pos.tolist(),
            "yaw": yaw, "collided": bool(collided)}

def handle(msg):
    if "reset" in msg:
        r = msg["reset"] or {}
        pos = r.get("pos")
        if pos is None:
            pos = pf.get_random_navigable_point()
        place(pos, float(r.get("yaw", 0.0)))
        return {"ok": True, "pos": state["pos"].tolist(), "yaw": state["yaw"], "K": K}
    if "step" in msg:
        s = msg["step"]
        vx, wz, dt = float(s["vx"]), float(s["wz"]), float(s.get("dt", 0.05))
        state["yaw"] += wz * dt
        fwd = np.array([-math.sin(state["yaw"]), 0.0, -math.cos(state["yaw"])])
        newp = state["pos"] + fwd * vx * dt
        collided = False
        if pf.is_navigable(np.array(newp, np.float32)):
            state["pos"] = newp
        else:
            collided = True
        place(state["pos"], state["yaw"])
        return frame(collided)
    if "render" in msg:
        if state["pos"] is None:
            place(pf.get_random_navigable_point(), 0.0)
        return frame()
    return {"error": "unknown message"}

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", a.port)); srv.listen(1)
print(f"[habitat-bridge] scene={a.scene}")
print(f"[habitat-bridge] listening on :{a.port}", flush=True)
while True:
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[habitat-bridge] client {addr}", flush=True)
    try:
        while True:
            hdr = b""
            while len(hdr) < 8:
                c = conn.recv(8 - len(hdr))
                if not c:
                    raise ConnectionError
                hdr += c
            n = struct.unpack(">Q", hdr)[0]
            buf = b""
            while len(buf) < n:
                buf += conn.recv(n - len(buf))
            out = handle(pickle.loads(buf))
            ob = pickle.dumps(out)
            conn.sendall(struct.pack(">Q", len(ob)) + ob)
    except (ConnectionError, ConnectionResetError, BrokenPipeError):
        print("[habitat-bridge] client disconnected", flush=True)
    finally:
        conn.close()
