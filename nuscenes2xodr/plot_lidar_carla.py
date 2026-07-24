#!/usr/bin/env python3
"""
Top-down (bird's-eye) renderer for the LiDAR sweeps captured *inside CARLA*
(`load_xodr_in_carla.py --lidar`), styled to match `plot_lidar.py` so the CARLA
sensor can be compared 1:1 against the original nuScenes LIDAR_TOP.

Input:  output/lidar_carla/<scene>/NNN.ply   (CARLA sensor.lidar.ray_cast sweeps)
Output: output/lidar_carla/<scene>/frames/NNN.png  +  lidar.gif

CARLA writes each sweep in the sensor's own left-handed frame (x forward, +y to
the right, z up, relative to the 1.84 m roof mount). To land in the SAME frame as
plot_lidar.py — ego centred, north-up / map-oriented, coloured by height above
ground — we:
  * flip Y            (CARLA left-handed -> right-handed, like carla_from_opendrive)
  * add the mount height to Z  (sensor-relative -> ~height above ground, matching
                                nuScenes' ego-relative Z so the colour maps line up)
  * rotate by the recorded ego map-yaw (from <scene>_meta.json), so a fixed map
    feature sits at a fixed screen position across frames — exactly as
    plot_lidar.py rotates the nuScenes points by the ego pose.
Frame NNN here lines up with output/lidar/<scene>/frames/NNN.png (nuScenes) and
output/cameras/<scene>/CAM_*/NNN.png.

Usage:
  python plot_lidar_carla.py                 # all scenes with captured sweeps
  python plot_lidar_carla.py scene-0061 ...  # selected scenes
  python plot_lidar_carla.py --heading-up    # ego-heading-up instead of north-up
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
LIDAR_DIR = os.path.join(OUT, "lidar_carla")

# kept identical to plot_lidar.py so the two renders are visually comparable
VIEW_HALF = 50.0            # metres from ego to each edge
GIF_FPS = 4
DPI = 110
BG = "#0b0b16"             # dark background
HEIGHT_CLIP = (-2.0, 8.0)  # colour range for point height (m above ground)
MOUNT_Z = 1.84             # LiDAR roof-mount height (see NUSCENES_LIDAR in loader)


def load_meta_yaws():
    """scene name -> list of ego map-yaws (radians), one per keyframe."""
    yaws = {}
    for m in glob.glob(os.path.join(OUT, "*_meta.json")):
        d = json.load(open(m))
        yaws[d["scene"]] = [p["yaw"] for p in d["ego_trajectory_local"]]
    return yaws


def read_ply(path):
    """Read a CARLA ascii PLY sweep -> (x, y, z) arrays in the sensor frame."""
    with open(path, "r") as f:
        line = ""
        while line.strip() != "end_header":
            line = f.readline()
            if not line:
                return np.empty(0), np.empty(0), np.empty(0)
        data = np.loadtxt(f, dtype=np.float32)
    if data.ndim == 1:                       # 0 or 1 point
        data = data.reshape(-1, 4)
    return data[:, 0], data[:, 1], data[:, 2]


def to_plot_frame(x, y, z, yaw, heading_up):
    """CARLA sensor frame -> plot_lidar frame (ego-centred, height above ground).

    north-up (default): rotate by the ego map-yaw so the map is fixed on screen.
    heading-up:         no rotation; the ego always faces +x (to the right).
    """
    ex, ey = x, -y                           # left-handed -> right-handed
    h = z + MOUNT_Z                           # sensor-relative -> height above ground
    if heading_up or yaw is None:
        return ex, ey, h
    c, s = math.cos(yaw), math.sin(yaw)
    return ex * c - ey * s, ex * s + ey * c, h


def render_scene(scene, yaws, heading_up):
    in_dir = os.path.join(LIDAR_DIR, scene)
    plys = sorted(glob.glob(os.path.join(in_dir, "[0-9]" * 3 + ".ply")))
    if not plys:
        print(f"[{scene}] no .ply sweeps in {os.path.relpath(in_dir, HERE)}; skipping")
        return

    frames_dir = os.path.join(in_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    scene_yaws = yaws.get(scene, [])

    frame_paths = []
    for p in plys:
        i = int(os.path.splitext(os.path.basename(p))[0])
        yaw = scene_yaws[i] if i < len(scene_yaws) else None
        x, y, z = read_ply(p)
        x, y, z = to_plot_frame(x, y, z, yaw, heading_up)
        m = (np.abs(x) <= VIEW_HALF) & (np.abs(y) <= VIEW_HALF)
        x, y, z = x[m], y[m], z[m]
        order = np.argsort(z)                       # draw low points first
        gyaw = 0.0 if (heading_up or yaw is None) else yaw

        fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=DPI)
        ax.set_facecolor(BG)
        ax.scatter(x[order], y[order], c=np.clip(z[order], *HEIGHT_CLIP),
                   cmap="viridis", s=0.5, linewidths=0, zorder=1)
        # ego marker (red) with heading arrow, matching plot_lidar.py
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
    gif_path = os.path.join(in_dir, "lidar.gif")
    imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / GIF_FPS), loop=0)
    print(f"[{scene}] wrote {len(frame_paths)} frames + "
          f"{os.path.relpath(gif_path, HERE)}")


def main():
    args = sys.argv[1:]
    heading_up = "--heading-up" in args
    wanted = [a for a in args if not a.startswith("--")]

    yaws = load_meta_yaws()
    scenes = sorted(os.path.basename(d) for d in glob.glob(os.path.join(LIDAR_DIR, "*"))
                    if os.path.isdir(d))
    if wanted:
        scenes = [s for s in scenes if s in wanted]
    if not scenes:
        print(f"No captured sweeps in {os.path.relpath(LIDAR_DIR, HERE)}. "
              f"Run load_xodr_in_carla.py --lidar first.")
        return

    orient = "heading-up" if heading_up else "north-up"
    print(f"rendering {len(scenes)} scene(s) ({orient}) -> "
          f"{os.path.relpath(LIDAR_DIR, HERE)}")
    for s in scenes:
        render_scene(s, yaws, heading_up)


if __name__ == "__main__":
    main()
