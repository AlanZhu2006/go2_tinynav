"""SIL bridge server, GAUSSIAN SPLATTING backend — same TCP protocol as habitat_bridge_server,
but rendering a pretrained 3DGS scene (public playground scenes: playroom/drjohnson/...).

Why GS (user direction): true radiance field -> multiview-consistent novel views (mesh bakes
break view-dependence), and APPEARANCE IS ADJUSTABLE PER FRAME at render time:
  step msg extras: {"exposure": gain, "blur": n_subframes, "noise": sigma}
  - exposure: linear gain before quantization (replays the real AE-vs-manual exposure problem)
  - blur: renders n sub-poses along the commanded motion and averages = physically correct
    motion blur (the real gait x exposure failure mode, now a dial)
  - noise: additive gaussian (sensor noise at high gain)

No collision mesh: kinematic free motion at fixed height (perception/reloc experiments; for nav,
add a density-grid occupancy later).

Run (gs env): python sil/gs_bridge_server.py --ply <point_cloud.ply> [--port 5602]
"""
import argparse, math, pickle, socket, struct
import numpy as np
import torch

W, H, HFOV, CAMH = 848, 480, 70.0, 0.33
fx = (W / 2) / math.tan(math.radians(HFOV / 2))
K = torch.tensor([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]], dtype=torch.float32, device="cuda")

ap = argparse.ArgumentParser()
ap.add_argument("--ply", required=True)
ap.add_argument("--port", type=int, default=5602)
ap.add_argument("--height", type=float, default=None, help="camera height (world units); default = scene median y")
a = ap.parse_args()

# ---- load 3DGS ply (INRIA format) ----
from plyfile import PlyData
pd = PlyData.read(a.ply)["vertex"]
means = torch.tensor(np.stack([pd["x"], pd["y"], pd["z"]], 1), dtype=torch.float32, device="cuda")
sh_dc = np.stack([pd["f_dc_0"], pd["f_dc_1"], pd["f_dc_2"]], 1)
rest_names = sorted([n for n in pd.data.dtype.names if n.startswith("f_rest_")],
                    key=lambda s: int(s.split("_")[-1]))
sh_rest = np.stack([pd[n] for n in rest_names], 1) if rest_names else np.zeros((len(means), 0))
n_coef = 1 + sh_rest.shape[1] // 3
shs = np.concatenate([sh_dc[:, None, :], sh_rest.reshape(len(means), 3, -1).transpose(0, 2, 1)], 1)
shs = torch.tensor(shs, dtype=torch.float32, device="cuda")
opac = torch.sigmoid(torch.tensor(np.array(pd["opacity"]), dtype=torch.float32, device="cuda"))
scales = torch.exp(torch.tensor(np.stack([pd["scale_0"], pd["scale_1"], pd["scale_2"]], 1),
                                dtype=torch.float32, device="cuda"))
quats = torch.tensor(np.stack([pd["rot_0"], pd["rot_1"], pd["rot_2"], pd["rot_3"]], 1),
                     dtype=torch.float32, device="cuda")
quats = quats / quats.norm(dim=1, keepdim=True)
print(f"[gs-bridge] {len(means)} gaussians, sh degree {int(math.sqrt(n_coef))-1}")

from gsplat import rasterization

med = means.median(0).values.cpu().numpy()
cam_y = a.height if a.height is not None else float(med[1])
state = {"pos": np.array([med[0], cam_y, med[2]], float), "yaw": 0.0}

def render(pos, yaw, exposure=1.0):
    # camera looks along -Z(yaw) like habitat convention; 3DGS worlds are COLMAP-style (y down);
    # keep a simple yaw-about-y camera — good enough for planar trajectories in playground scenes.
    c, s = math.cos(yaw), math.sin(yaw)
    R_c2w = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]], float)
    T = np.eye(4); T[:3, :3] = R_c2w; T[:3, 3] = pos
    viewmat = torch.tensor(np.linalg.inv(T), dtype=torch.float32, device="cuda")[None]
    img, _, _ = rasterization(means, quats, scales, opac, shs,
                              viewmat, K[None], W, H, sh_degree=int(math.sqrt(n_coef)) - 1)
    im = img[0].clamp(0, 1).cpu().numpy()
    im = np.clip(im * 255.0 * exposure, 0, 255).astype(np.uint8)
    return im

def frame(vx=0.0, wz=0.0, dt=0.0, exposure=1.0, blur=1, noise=0.0):
    import cv2
    n = max(1, int(blur))
    acc = None
    for k in range(n):
        f = (k + 1) / n
        yaw_k = state["yaw"] + wz * dt * (f - 1.0)
        fwd = np.array([-math.sin(yaw_k), 0.0, -math.cos(yaw_k)])
        pos_k = state["pos"] + fwd * vx * dt * (f - 1.0)
        im = render(pos_k, yaw_k, exposure).astype(np.float32)
        acc = im if acc is None else acc + im
    im = (acc / n)
    if noise > 0:
        im = im + np.random.normal(0, noise * 255, im.shape)
    im = np.clip(im, 0, 255).astype(np.uint8)
    ok, enc = cv2.imencode(".jpg", cv2.cvtColor(im, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
    T = np.eye(4)
    c, s = math.cos(state["yaw"]), math.sin(state["yaw"])
    T[:3, :3] = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    T[:3, 3] = state["pos"]
    return {"jpg": enc.tobytes(), "gt_pose": T.tolist(), "pos": state["pos"].tolist(),
            "yaw": state["yaw"], "collided": False}

def handle(msg):
    if "reset" in msg:
        r = msg["reset"] or {}
        if r.get("pos") is not None:
            state["pos"] = np.array(r["pos"], float)
        state["yaw"] = float(r.get("yaw", 0.0))
        return {"ok": True, "pos": state["pos"].tolist(), "yaw": state["yaw"],
                "K": [[float(fx), 0.0, W / 2], [0.0, float(fx), H / 2], [0.0, 0.0, 1.0]]}
    if "step" in msg:
        s_ = msg["step"]
        vx, wz, dt = float(s_["vx"]), float(s_["wz"]), float(s_.get("dt", 0.05))
        state["yaw"] += wz * dt
        fwd = np.array([-math.sin(state["yaw"]), 0.0, -math.cos(state["yaw"])])
        state["pos"] = state["pos"] + fwd * vx * dt
        return frame(vx, wz, dt, float(s_.get("exposure", 1.0)), int(s_.get("blur", 1)),
                     float(s_.get("noise", 0.0)))
    if "render" in msg:
        r = msg["render"] or {}
        return frame(exposure=float(r.get("exposure", 1.0)), blur=int(r.get("blur", 1)),
                     noise=float(r.get("noise", 0.0)))
    return {"error": "unknown"}

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", a.port)); srv.listen(1)
print(f"[gs-bridge] listening on :{a.port}", flush=True)
while True:
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        while True:
            hdr = b""
            while len(hdr) < 8:
                c = conn.recv(8 - len(hdr))
                if not c:
                    raise ConnectionError
                hdr += c
            nb = struct.unpack(">Q", hdr)[0]
            buf = b""
            while len(buf) < nb:
                buf += conn.recv(nb - len(buf))
            out = handle(pickle.loads(buf))
            ob = pickle.dumps(out)
            conn.sendall(struct.pack(">Q", len(ob)) + ob)
    except (ConnectionError, ConnectionResetError, BrokenPipeError):
        print("[gs-bridge] client disconnected", flush=True)
    finally:
        conn.close()
