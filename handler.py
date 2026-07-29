import base64
import logging
import os
import sys
import tempfile

import cv2
import imageio
import numpy as np
import open3d as o3d
import runpod
import torch
from omegaconf import OmegaConf

sys.path.append("/app/FoundationStereo")
from core.foundation_stereo import FoundationStereo
from core.utils.utils import InputPadder
from Utils import depth2xyzmap, set_logging_format, set_seed, toOpen3dCloud, vis_disparity

CKPT_DIR = os.environ.get("CKPT_DIR", "/runpod-volume/pretrained_models/11-33-40/model_best_bp2.pth")

set_logging_format()
set_seed(0)
torch.autograd.set_grad_enabled(False)

_model = None
_device = None


def _load_model():
    global _model, _device
    if _model is not None:
        return _model

    if not os.path.exists(CKPT_DIR):
        raise RuntimeError(
            f"Checkpoint not found at {CKPT_DIR}. Attach the network volume that holds "
            f"pretrained_models/11-33-40/ to this endpoint."
        )

    cfg = OmegaConf.load(os.path.join(os.path.dirname(CKPT_DIR), "cfg.yaml"))
    if "vit_size" not in cfg:
        cfg["vit_size"] = "vitl"
    args = OmegaConf.create(cfg)

    _device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logging.info(f"Using device: {_device}")

    model = FoundationStereo(args)
    ckpt = torch.load(CKPT_DIR, map_location=_device)
    logging.info(f"ckpt global_step:{ckpt['global_step']}, epoch:{ckpt['epoch']}")
    model.load_state_dict(ckpt["model"])
    model.to(_device)
    model.eval()

    _model = model
    return _model


def _b64_to_bgr(b64_str):
    data = base64.b64decode(b64_str)
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Could not decode image from base64 input")
    return img


def _file_to_b64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def handler(job):
    inp = job["input"]
    model = _load_model()

    scale = float(inp.get("scale", 0.4))
    valid_iters = int(inp.get("valid_iters", 16))
    hiera = int(inp.get("hiera", 0))
    z_far = float(inp.get("z_far", 10))
    remove_invisible = int(inp.get("remove_invisible", 1))
    denoise_cloud = int(inp.get("denoise_cloud", 1))
    denoise_nb_points = int(inp.get("denoise_nb_points", 30))
    denoise_radius = float(inp.get("denoise_radius", 0.03))
    get_pc = int(inp.get("get_pc", 1))

    fx, fy, cx, cy = (float(inp[k]) for k in ("fx", "fy", "cx", "cy"))
    baseline_m = float(inp["baseline_m"])

    img0_bgr = _b64_to_bgr(inp["left_image_b64"])
    img1_bgr = _b64_to_bgr(inp["right_image_b64"])
    img0 = cv2.cvtColor(img0_bgr, cv2.COLOR_BGR2RGB)
    img1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2RGB)

    assert scale <= 1, "scale must be <=1"
    img0 = cv2.resize(img0, fx=scale, fy=scale, dsize=None)
    img1 = cv2.resize(img1, fx=scale, fy=scale, dsize=None)
    H, W = img0.shape[:2]
    img0_ori = img0.copy()

    img0_t = torch.as_tensor(img0).to(_device).float()[None].permute(0, 3, 1, 2)
    img1_t = torch.as_tensor(img1).to(_device).float()[None].permute(0, 3, 1, 2)
    padder = InputPadder(img0_t.shape, divis_by=32, force_square=False)
    img0_t, img1_t = padder.pad(img0_t, img1_t)

    with torch.cuda.amp.autocast(True):
        if not hiera:
            disp = model.forward(img0_t, img1_t, iters=valid_iters, test_mode=True)
        else:
            disp = model.run_hierachical(img0_t, img1_t, iters=valid_iters, test_mode=True, small_ratio=0.5)
    disp = padder.unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(H, W)

    with tempfile.TemporaryDirectory() as tmp:
        vis_img = np.concatenate([img0_ori, vis_disparity(disp)], axis=1)
        vis_path = os.path.join(tmp, "vis.png")
        imageio.imwrite(vis_path, vis_img)

        result = {"vis_png_b64": _file_to_b64(vis_path)}

        if remove_invisible:
            yy, xx = np.meshgrid(np.arange(disp.shape[0]), np.arange(disp.shape[1]), indexing="ij")
            us_right = xx - disp
            disp[us_right < 0] = np.inf

        if get_pc:
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
            K[:2] *= scale
            depth = K[0, 0] * baseline_m / disp
            xyz_map = depth2xyzmap(depth, K)
            pcd = toOpen3dCloud(xyz_map.reshape(-1, 3), img0_ori.reshape(-1, 3))
            pts = np.asarray(pcd.points)
            keep_ids = np.arange(len(pts))[(pts[:, 2] > 0) & (pts[:, 2] <= z_far)]
            pcd = pcd.select_by_index(keep_ids)

            cloud_path = os.path.join(tmp, "cloud.ply")
            o3d.io.write_point_cloud(cloud_path, pcd)
            result["cloud_ply_b64"] = _file_to_b64(cloud_path)

            if denoise_cloud:
                _, ind = pcd.remove_radius_outlier(nb_points=denoise_nb_points, radius=denoise_radius)
                inlier = pcd.select_by_index(ind)
                denoise_path = os.path.join(tmp, "cloud_denoise.ply")
                o3d.io.write_point_cloud(denoise_path, inlier)
                result["cloud_denoise_ply_b64"] = _file_to_b64(denoise_path)

        return result


runpod.serverless.start({"handler": handler})
