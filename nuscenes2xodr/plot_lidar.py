#!/usr/bin/env python3
"""
Top-down (bird's-eye) renderer for the original nuScenes LIDAR_TOP point clouds.

For every mini scene we render one BEV image per keyframe, in the SAME frame as
`plot_topdown.py`: ego at the centre, **north-up / map-oriented** (not ego-heading
up), ±VIEW_HALF metres. Because the local map frame is a pure translation of the
nuScenes global frame, rotating the ego-frame points by the ego pose (rotation
only, no translation) puts them in exactly the top-down's frame — so frame i here
lines up 1:1 with:
  - output/topdown/<scene>/frames/NNN.png
  - output/cameras/<scene>/CAM_*/NNN.png   (and the i-th nuScenes keyframe)

Point clouds come straight from v1.0-mini/samples/LIDAR_TOP; the sensor->ego
extrinsic is read from `calibrated_sensor`, the ego pose from `ego_pose`.

Outputs, per scene, into output/lidar/<scene>/:
  - frames/NNN.png   one LiDAR BEV per keyframe (coloured by height)
  - lidar.gif        the animated sequence

Usage:
  python plot_lidar.py                 # all scenes
  python plot_lidar.py scene-0061 ...  # selected scenes
"""

import os
import sys
import glob
import json
import math

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
DATAROOT = os.path.join(HERE, "..", "v1.0-mini")
TABLES = os.path.join(DATAROOT, "v1.0-mini")
LIDAR_DIR = os.path.join(OUT, "lidar")

VIEW_HALF = 50.0            # metres from ego to each edge (matches plot_topdown)
GIF_FPS = 4
DPI = 110
BG = "#0b0b16"             # dark background
HEIGHT_CLIP = (-2.0, 8.0)  # colour range for point height (m above ego)


# ----------------------------------------------------------------------------
# nuScenes table loading + rigid transforms
# ----------------------------------------------------------------------------
def _load(name):
    return json.load(open(os.path.join(TABLES, name + ".json")))


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def quat_yaw(q):
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1-2*(y*y+z*z))


def build_index():
    """scene name -> ordered list of LIDAR_TOP keyframe sample_data records."""
    scene_t = _load("scene")
    sample_t = _load("sample")
    sd_t = _load("sample_data")
    lidar_sd = {x["sample_token"]: x for x in sd_t
                if "LIDAR_TOP" in x["filename"] and x["is_key_frame"]}
    by_scene = {}
    for s in scene_t:
        smap = {x["token"]: x for x in sample_t if x["scene_token"] == s["token"]}
        order, tok = [], s["first_sample_token"]
        while tok:
            if tok in lidar_sd:
                order.append(lidar_sd[tok])
            tok = smap.get(tok, {}).get("next", "")
        by_scene[s["name"]] = order
    cs = {c["token"]: c for c in _load("calibrated_sensor")}
    ep = {e["token"]: e for e in _load("ego_pose")}
    return by_scene, cs, ep


def load_points_egocentric(sd, cs, ep):
    """Return (x, y, z) points centred on the ego, in map (north-up) orientation."""
    p = np.fromfile(os.path.join(DATAROOT, sd["filename"]),
                    dtype=np.float32).reshape(-1, 5)
    c = cs[sd["calibrated_sensor_token"]]
    e = ep[sd["ego_pose_token"]]
    # sensor -> ego
    xyz = quat_to_R(c["rotation"]) @ p[:, :3].T + np.array(c["translation"])[:, None]
    # ego -> map orientation, centred on ego (rotation only, drop translation)
    xyz = quat_to_R(e["rotation"]) @ xyz
    return xyz[0], xyz[1], xyz[2], quat_yaw(e["rotation"])


# ----------------------------------------------------------------------------
# render one scene
# ----------------------------------------------------------------------------
def render_scene(scene, records, cs, ep, n_cap=None):
    out_dir = os.path.join(LIDAR_DIR, scene)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    n = len(records) if n_cap is None else min(len(records), n_cap)
    frame_paths = []
    for i in range(n):
        x, y, z, gyaw = load_points_egocentric(records[i], cs, ep)
        m = (np.abs(x) <= VIEW_HALF) & (np.abs(y) <= VIEW_HALF)
        x, y, z = x[m], y[m], z[m]
        order = np.argsort(z)                       # draw low points first

        fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=DPI)
        ax.set_facecolor(BG)
        ax.scatter(x[order], y[order], c=np.clip(z[order], *HEIGHT_CLIP),
                   cmap="viridis", s=0.5, linewidths=0, zorder=1)
        # ego marker (red) with heading arrow, matching the top-down colour
        ax.annotate("", xy=(math.cos(gyaw) * 4.6, math.sin(gyaw) * 4.6),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="#e33", lw=1.8),
                    zorder=5)
        ax.scatter(0, 0, marker="o", s=28, color="#e33",
                   edgecolors="white", linewidths=0.6, zorder=6)

        ax.set_xlim(-VIEW_HALF, VIEW_HALF)
        ax.set_ylim(-VIEW_HALF, VIEW_HALF)
        ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_edgecolor("#999999")

        fp = os.path.join(frames_dir, f"{i:03d}.png")
        fig.savefig(fp, bbox_inches="tight", pad_inches=0.05, facecolor="white")
        plt.close(fig)
        frame_paths.append(fp)

    imgs = [Image.open(p).convert("RGB") for p in frame_paths]
    w0, h0 = imgs[0].size
    imgs = [im if im.size == (w0, h0) else im.resize((w0, h0)) for im in imgs]
    gif_path = os.path.join(out_dir, "lidar.gif")
    imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / GIF_FPS), loop=0)
    print(f"[{scene}] wrote {n} frames + {os.path.relpath(gif_path, HERE)}")


def main():
    # scenes present in the processed output (drives which ones we render + frame cap)
    caps = {}
    for m in glob.glob(os.path.join(OUT, "*_meta.json")):
        d = json.load(open(m))
        caps[d["scene"]] = len(d["ego_trajectory_local"])

    print("indexing nuScenes LIDAR_TOP keyframes ...")
    by_scene, cs, ep = build_index()

    wanted = sys.argv[1:]
    scenes = sorted(s for s in caps if s in by_scene)
    if wanted:
        scenes = [s for s in scenes if s in wanted]

    print(f"rendering {len(scenes)} scene(s) -> {os.path.relpath(LIDAR_DIR, HERE)}")
    for s in scenes:
        render_scene(s, by_scene[s], cs, ep, n_cap=caps.get(s))


if __name__ == "__main__":
    main()
