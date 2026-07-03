"""Torch fallback for models_trt — same classes/async interface, no TensorRT engines needed.

Purpose: software-in-the-loop sim on x86 where the Jetson .plan engines don't exist. Same
underlying networks (SuperPoint/LightGlue via the `lightglue` package, DINOv2-base via
transformers), so accuracy matches the offline validation pipelines (exp72/76); only latency
differs (torch is slower — SIL timing is therefore conservative).
Enable with TINYNAV_TORCH_MODELS=1 (models_trt re-exports from here).
"""
import numpy as np
import cv2
import torch

_dev = "cuda" if torch.cuda.is_available() else "cpu"
N_KPTS = 1024


class SuperPointTRT:
    def __init__(self, engine_path=None):
        from lightglue import SuperPoint
        self.model = SuperPoint(max_num_keypoints=N_KPTS).eval().to(_dev)
        self.input_shape = np.array([480, 848])

    async def infer(self, input_image: np.ndarray, threshold=None):
        img = input_image
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        t = torch.from_numpy(img).float()[None, None].to(_dev) / 255.0
        with torch.no_grad():
            f = self.model.extract(t)
        k = f["keypoints"][0].cpu().numpy()          # [n,2] in input coords
        d = f["descriptors"][0].cpu().numpy()        # [n,256]
        n = k.shape[0]
        kpts = np.zeros((1, N_KPTS, 2), np.float32)
        descps = np.zeros((1, N_KPTS, d.shape[1]), np.float32)
        mask = np.zeros((1, N_KPTS), np.float32)
        kpts[0, :n] = k[:N_KPTS]
        descps[0, :n] = d[:N_KPTS]
        mask[0, :n] = 1.0
        return {"kpts": kpts, "descps": descps, "mask": mask[:, :, None]}


class LightGlueTRT:
    def __init__(self, engine_path=None):
        from lightglue import LightGlue
        self.model = LightGlue(features="superpoint").eval().to(_dev)

    async def infer(self, kpts0, kpts1, desc0, desc1, mask0, mask1, img_shape0, img_shape1,
                    match_threshold=None):
        m0 = mask0.reshape(-1) > 0.5
        m1 = mask1.reshape(-1) > 0.5
        n0, n1 = int(m0.sum()), int(m1.sum())
        out = np.full((1, kpts0.shape[1]), -1, np.int64)
        if n0 < 4 or n1 < 4:
            return {"match_indices": out}
        sz0 = torch.tensor([[float(img_shape0.reshape(-1)[0]), float(img_shape0.reshape(-1)[1])]], device=_dev)
        sz1 = torch.tensor([[float(img_shape1.reshape(-1)[0]), float(img_shape1.reshape(-1)[1])]], device=_dev)
        f0 = {"keypoints": torch.from_numpy(kpts0[:, :n0]).float().to(_dev),
              "descriptors": torch.from_numpy(desc0[:, :n0]).float().to(_dev),
              "image_size": sz0}
        f1 = {"keypoints": torch.from_numpy(kpts1[:, :n1]).float().to(_dev),
              "descriptors": torch.from_numpy(desc1[:, :n1]).float().to(_dev),
              "image_size": sz1}
        with torch.no_grad():
            res = self.model({"image0": f0, "image1": f1})
        matches = res["matches"][0].cpu().numpy()    # [m,2] indices into valid sets
        for a, b in matches:
            out[0, int(a)] = int(b)
        return {"match_indices": out}


class Dinov2TRT:
    def __init__(self, engine_path=None):
        from transformers import AutoModel
        self.model = AutoModel.from_pretrained("facebook/dinov2-base").eval().to(_dev)
        self.mean = torch.tensor([0.485, 0.456, 0.406], device=_dev).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225], device=_dev).view(1, 3, 1, 1)

    async def infer(self, image):
        img = cv2.resize(image, (224, 224), interpolation=cv2.INTER_CUBIC)
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        t = torch.from_numpy(img).float().permute(2, 0, 1)[None].to(_dev) / 255.0
        t = (t - self.mean) / self.std
        with torch.no_grad():
            e = self.model(pixel_values=t).last_hidden_state[:, 0, :]
        return e.squeeze(0).cpu().numpy()


class FoundationStereoTRT:
    def __init__(self, *a, **k):
        raise RuntimeError("FoundationStereoTRT has no torch fallback (mono SIL does not use stereo)")


class RetinifyTRT:
    def __init__(self, *a, **k):
        raise RuntimeError("RetinifyTRT has no torch fallback (mono SIL does not use stereo)")
