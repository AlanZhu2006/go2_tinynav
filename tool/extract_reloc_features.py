"""Extract reloc features for a LingBot-native map -- RUN ON THE ROBOT (tinynav env, TRT engines).

Reloc matches query features against the map's stored features, so the map's features.db/embeddings.db MUST
be produced by the SAME extractors the runtime uses (SuperPointTRT + Dinov2TRT). Building them with any other
extractor (different SuperPoint/DINOv2 build) would silently break matching. So this runs on the Orin with the
identical TRT models, over the map's keyframe images (kf_images/), keyed by frame index to match poses.npy /
depths.db.

  source tinynav_setup.bash && source .venv/bin/activate
  PYTHONPATH=go2_tinynav_mono python tool/extract_reloc_features.py --map output/latest_lingbot_map

Writes (TinyNav format): features.db (dict {kpts,scores,descps,mask}) + embeddings.db (768-d L2-normed).
"""
import argparse, asyncio, glob, os, shelve
import numpy as np, cv2

from tinynav.core.models_trt import SuperPointTRT, Dinov2TRT
from tool.video_db import VideoDB


class IntKeyShelf:
    """Inlined from tinynav.core.build_map_node (importing it pulls in rclpy/ROS). Identical on-disk format."""
    def __init__(self, filename, flag="c"):
        self.db = shelve.open(filename, flag=flag)

    def __setitem__(self, key, value):
        self.db[str(key)] = value

    def keys(self):
        return [int(k) for k in self.db.keys()]

    def close(self):
        self.db.close()


def _key_from_path(path):
    return int(os.path.basename(path).split(".")[0].split("_")[-1])


def _filter_to_pose_depth_keys(map_dir, files):
    valid = None
    pose_path = os.path.join(map_dir, "poses.npy")
    if os.path.exists(pose_path):
        poses = np.load(pose_path, allow_pickle=True).item()
        valid = {int(k) for k in poses.keys()}

    depth_base = os.path.join(map_dir, "depths")
    try:
        depth_db = shelve.open(depth_base, flag="r")
        depth_keys = {int(k) for k in depth_db.keys()}
        depth_db.close()
        valid = depth_keys if valid is None else (valid & depth_keys)
    except Exception:
        pass

    if valid is None:
        return files, 0
    filtered = [f for f in files if _key_from_path(f) in valid]
    return filtered, len(files) - len(filtered)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="LingBot-native map dir (must contain kf_images/)")
    ap.add_argument("--images", default=None, help="keyframe png dir (default <map>/kf_images)")
    a = ap.parse_args()
    img_dir = a.images or os.path.join(a.map, "kf_images")
    files = sorted(glob.glob(f"{img_dir}/*.png"))
    if not files:
        raise SystemExit(f"no keyframe pngs in {img_dir}")
    files, skipped = _filter_to_pose_depth_keys(a.map, files)
    if not files:
        raise SystemExit(f"no keyframe pngs in {img_dir} matched poses.npy/depths")
    if skipped:
        print(f"filtered {skipped} keyframe images without matching pose/depth entries", flush=True)
    print(f"{len(files)} keyframes | extracting SuperPoint + DINOv2 (runtime TRT models)", flush=True)

    sp = SuperPointTRT(); dn = Dinov2TRT()
    feats = IntKeyShelf(os.path.join(a.map, "features"), flag="n")
    embs = IntKeyShelf(os.path.join(a.map, "embeddings"), flag="n")
    # Also store keyframe images as VideoDBs so the map is a complete TinyNavDB drop-in (reloc itself
    # doesn't read them, but TinyNavDB opens them at load). mono: rgb=infra1=the keyframe image.
    rgb_db = VideoDB(os.path.join(a.map, "rgb_images_db"), mode="write")
    infra_db = VideoDB(os.path.join(a.map, "infra1_images_db"), mode="write")
    n = 0
    for f in files:
        key = _key_from_path(f)
        bgr = cv2.imread(f)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)             # runtime feeds grayscale keyframes
        ft = asyncio.run(sp.infer(gray))
        em = np.asarray(asyncio.run(dn.infer(gray))).reshape(-1).astype(np.float32)
        nrm = np.linalg.norm(em)
        if nrm > 0:
            em = em / nrm                                         # match stored norm=1.0
        feats[key] = ft
        embs[key] = em
        rgb_db.write(key, bgr)
        infra_db.write(key, gray)
        n += 1
        if n % 20 == 0:
            print(f"  {n}/{len(files)}", flush=True)
    feats.close(); embs.close(); rgb_db.close(); infra_db.close()
    print(f"wrote features.db + embeddings.db + rgb/infra1_images_db ({n} keyframes) -> {a.map}", flush=True)


if __name__ == "__main__":
    main()
