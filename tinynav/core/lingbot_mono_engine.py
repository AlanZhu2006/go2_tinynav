"""Drop-in replacement for go2_tinynav's StereoEngineTRT — MONOCULAR metric depth.

TinyNav's perception/relocalization (SuperPoint+LightGlue VO, DINOv2+LightGlue+PnP map relocalization,
GTSAM pose-graph) are depth-source-agnostic — they only consume (image, depth, K). Stereo just supplied
the depth. Give them mono metric depth here and the whole system becomes monocular, unchanged downstream.

Why LingBot (scale-consistent streaming depth) over a per-frame metric model (DepthPro/Metric3D):
TinyNav's frame-to-keyframe PnP VO needs CROSS-FRAME depth consistency, which LingBot's streaming
multi-view recon gives and a per-frame model does not (we measured: cross-frame consistency beats
per-frame cleanliness for PnP/ICP). LingBot is up-to-scale -> one metric scale factor (DepthPro /
commanded-motion bootstrap / camera-height) makes it metric.

Two compute modes:
  - SOCKET (default): thin client to perception_server/server.py on a workstation GPU (LingBot is heavy;
    runs ~6-7 FPS on a 4090). Go2 Orin keeps running TinyNav (TRT) >20Hz; only depth is offboarded.
  - LOCAL: if you TRT-export a metric mono model, run it onboard like TinyNav's stereo model (productionize).

Interface mirrors StereoEngineTRT.infer(left, right, baseline, fx) so it slots into perception_node.py:
    self.stereo_engine = StereoEngineTRT()
    -> self.stereo_engine = LingBotMonoEngine(host="WORKSTATION_IP", port=5599, scale=...)
The `right`/`baseline` args are accepted and ignored (mono). Returns (disparity=None, depth).
"""
import asyncio, pickle, socket, struct, numpy as np


def _send(sock, obj):
    b = pickle.dumps(obj); sock.sendall(struct.pack(">Q", len(b)) + b)


def _recv(sock):
    h = b""
    while len(h) < 8:
        c = sock.recv(8 - len(h));  h += c
        if not c: raise ConnectionError("perception server closed")
    n = struct.unpack(">Q", h)[0]; buf = b""
    while len(buf) < n:
        buf += sock.recv(n - len(buf))
    return _decode_obj(pickle.loads(buf))


def _decode_obj(obj):
    if isinstance(obj, dict) and obj.get("__ndarray__"):
        arr = np.frombuffer(obj["data"], dtype=np.dtype(obj["dtype"]))
        return arr.reshape(obj["shape"]).copy()
    if isinstance(obj, dict):
        return {key: _decode_obj(value) for key, value in obj.items()}
    if isinstance(obj, list):
        return [_decode_obj(value) for value in obj]
    return obj


class LingBotMonoEngine:
    def __init__(self, host="127.0.0.1", port=5599, scale=1.0, reset=True):
        """scale: metric multiplier for LingBot's up-to-scale depth. Set from DepthPro (~6%),
        a commanded-motion bootstrap, or camera-height; or 1.0 if the server already returns metric."""
        self.scale = float(scale)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        if reset:
            _send(self.sock, {"reset": True}); _recv(self.sock)

    def _infer_sync(self, img, fx=None):
        # img: HxW (mono8) or HxWx3. LingBot wants RGB; replicate gray to 3ch if needed.
        rgb = img if img.ndim == 3 else np.repeat(img[..., None], 3, axis=2)
        msg = {"rgb": np.ascontiguousarray(rgb.astype(np.uint8))}
        if fx is not None:
            msg["f_px"] = float(fx)
        _send(self.sock, msg)
        out = _recv(self.sock)
        depth = out["depth"].astype(np.float32) * self.scale
        # out also carries "reliability" (depth_conf-gated, in [0,1]) — TinyNav doesn't use it yet,
        # but the planning/costmap layer can read it via a side channel to gate free-space (the 14-24x win).
        return depth, out.get("reliability"), out.get("c2w")

    async def infer(self, left_img, right_img=None, baseline=None, fx=None):
        """Async to match StereoEngineTRT. Returns (disparity, depth); disparity is None (mono)."""
        depth, _rel, _pose = await asyncio.to_thread(self._infer_sync, left_img, fx)
        return None, depth

    def reset(self):
        _send(self.sock, {"reset": True}); _recv(self.sock)
