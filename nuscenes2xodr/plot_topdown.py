#!/usr/bin/env python3
"""
Top-down (bird's-eye) renderer for the nuScenes -> CARLA mini scenarios.

For every scene we already export:
  - a road network as OpenDRIVE           (output/<map>_<scene>.xodr)
  - per-frame ego + agent poses           (output/<map>_<scene>_meta.json)
  - CARLA / nuScenes camera frames        (output/cameras/<scene>/CAM_*/NNN.png)

The meta file is sampled at the nuScenes keyframe rate (dt = 0.5 s), so frame i
of the top-down corresponds 1:1 with:
  - output/cameras/<scene>/CAM_*/{i:03d}.png   (the aligned camera montage)
  - the i-th keyframe / sample of v1.0-mini      (the source of the sweeps)

Outputs, per scene, into output/topdown/<scene>/:
  - frames/NNN.png   one BEV image per timestep (same index as the cameras)
  - topdown.gif      the animated sequence

Usage:
  python plot_topdown.py                 # all scenes
  python plot_topdown.py scene-0061 ...  # selected scenes
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
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
import lxml.etree as ET
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
TOPDOWN_DIR = os.path.join(OUT, "topdown")

GIF_FPS = 4
DPI = 110

# ego-centric square window: metres from the ego to each edge (half side length)
VIEW_HALF = 50.0

# history / future trajectory overlay — matches load_xodr_in_carla.py defaults
HIST_SECONDS = 2.0
FUTURE_SECONDS = 6.0
TRAJ_HZ = 2.0

# trajectory group colours (RGB 0-1); history is drawn dimmer than the future,
# exactly as _TRAJ_BASE / _traj_color in the CARLA sim.
TRAJ_BASE = {
    "ego":        (0.85, 0.12, 0.12),    # red    (hero / ego vehicle)
    "vehicle":    (0.12, 0.47, 0.86),    # blue   (surrounding vehicles)
    "pedestrian": (0.0, 0.863, 0.235),   # green
    "cyclist":    (1.0, 0.235, 0.784),   # magenta
}
HIST_DIM = 0.45   # brightness factor for the history segment


def traj_group(category):
    """Map a nuScenes category to a CARLA trajectory group / colour."""
    c = category.lower()
    if "pedestrian" in c:
        return "pedestrian"
    if "bicycle" in c or "motorcycle" in c:
        return "cyclist"
    return "vehicle"


def traj_color(group, is_future):
    r, g, b = TRAJ_BASE.get(group, TRAJ_BASE["vehicle"])
    f = 1.0 if is_future else HIST_DIM
    return (r * f, g * f, b * f)


def agent_color(category):
    """Box fill colour — same palette as the trajectory group."""
    return TRAJ_BASE[traj_group(category)]


# ----------------------------------------------------------------------------
# OpenDRIVE -> lane-surface polygons (all geometries are straight <line/>)
# ----------------------------------------------------------------------------
def _poly(coeffs, ds):
    a, b, c, d = coeffs
    return a + b * ds + c * ds * ds + d * ds * ds * ds


def parse_xodr(path, step=1.0):
    """Return a list of (Nx2) polygons, one per driving lane surface."""
    root = ET.parse(path).getroot()
    polygons = []
    centerlines = []
    for road in root.findall("road"):
        pv = road.find("planView")
        if pv is None:
            continue

        # sample the reference line -> arrays of station s, x, y, heading
        S, X, Y, H = [], [], [], []
        for g in pv.findall("geometry"):
            gx = float(g.get("x")); gy = float(g.get("y"))
            hdg = float(g.get("hdg")); glen = float(g.get("length"))
            s0 = float(g.get("s"))
            n = max(1, int(math.ceil(glen / step)))
            for k in range(n):
                ds = glen * k / n
                S.append(s0 + ds)
                X.append(gx + ds * math.cos(hdg))
                Y.append(gy + ds * math.sin(hdg))
                H.append(hdg)
            # segment end point
            S.append(s0 + glen)
            X.append(gx + glen * math.cos(hdg))
            Y.append(gy + glen * math.sin(hdg))
            H.append(hdg)
        if len(S) < 2:
            continue
        S = np.array(S); X = np.array(X); Y = np.array(Y); H = np.array(H)

        lanes_el = road.find("lanes")
        lo = lanes_el.find("laneOffset")
        lo_c = (float(lo.get("a")), float(lo.get("b")),
                float(lo.get("c")), float(lo.get("d"))) if lo is not None else (0, 0, 0, 0)
        lo_s = float(lo.get("s")) if lo is not None else 0.0
        offset = _poly(lo_c, S - lo_s)

        sec = lanes_el.find("laneSection")
        sec_s = float(sec.get("s"))

        # left (+t) and right (-t) driving lanes, ordered outward from centre
        for side, sign in (("left", +1.0), ("right", -1.0)):
            container = sec.find(side)
            if container is None:
                continue
            lanes = sorted(container.findall("lane"),
                           key=lambda l: abs(int(l.get("id"))))
            inner = offset.copy()          # t of the boundary toward the centre
            for lane in lanes:
                w = lane.find("width")
                wc = (float(w.get("a")), float(w.get("b")),
                      float(w.get("c")), float(w.get("d"))) if w is not None else (0, 0, 0, 0)
                w_s = float(w.get("sOffset")) if w is not None else 0.0
                width = _poly(wc, S - (sec_s + w_s))
                outer = inner + sign * width

                if lane.get("type") == "driving":
                    nx = -np.sin(H); ny = np.cos(H)      # +t (left) unit normal
                    ix = X + inner * nx; iy = Y + inner * ny
                    ox = X + outer * nx; oy = Y + outer * ny
                    ring = np.concatenate(
                        [np.stack([ix, iy], 1), np.stack([ox, oy], 1)[::-1]], 0)
                    polygons.append(ring)
                inner = outer

        centerlines.append(np.stack([X + offset * (-np.sin(H)),
                                     Y + offset * np.cos(H)], 1))
    return polygons, centerlines


# ----------------------------------------------------------------------------
# oriented box corners
# ----------------------------------------------------------------------------
def box_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    l2, w2 = length / 2.0, width / 2.0
    local = [(l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2)]
    return [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in local]


# ----------------------------------------------------------------------------
# render one scene
# ----------------------------------------------------------------------------
def _draw_polyline(ax, pts, group, is_future, zbase):
    """Draw one history/future polyline + points, CARLA-style."""
    if len(pts) < 1:
        return
    pts = np.asarray(pts)
    col = traj_color(group, is_future)
    lw = 2.2 if is_future else 1.6
    z = zbase + (1 if is_future else 0)
    if len(pts) >= 2:
        ax.plot(pts[:, 0], pts[:, 1], color=col, lw=lw, zorder=z,
                solid_capstyle="round")
    ax.scatter(pts[:, 0], pts[:, 1], s=9 if is_future else 6, color=col,
               zorder=z + 0.1, edgecolors="none")


def render_scene(meta_path, xodr_path):
    meta = json.load(open(meta_path))
    scene = meta["scene"]
    print(f"[{scene}] parsing road network ...")
    polygons, centerlines = parse_xodr(xodr_path)

    ego_traj = meta["ego_trajectory_local"]
    agent_frames = meta["agent_frames"]
    dt = meta.get("frame_dt", 0.5)
    n_frames = len(ego_traj)

    # trajectory sampling (same convention as the CARLA overlay)
    step = max(1, int(round((1.0 / TRAJ_HZ) / dt)))
    hn = int(round(HIST_SECONDS * TRAJ_HZ))
    fn = int(round(FUTURE_SECONDS * TRAJ_HZ))

    # per-agent position track, keyed by id -> {frame: (x, y)}
    agent_xy, agent_grp = {}, {}
    for fi, fr in enumerate(agent_frames):
        for a in fr:
            agent_xy.setdefault(a["id"], {})[fi] = (a["x"], a["y"])
            agent_grp.setdefault(a["id"], traj_group(a["category"]))

    out_dir = os.path.join(TOPDOWN_DIR, scene)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    frame_paths = []
    for i in range(n_frames):
        e = ego_traj[i]
        cx, cy = e["x"], e["y"]

        fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=DPI)

        # --- road surface (map-fixed, clipped to the ego window) ---
        ax.add_collection(PatchCollection(
            [MplPolygon(p, closed=True) for p in polygons],
            facecolor="#d6d6d6", edgecolor="none", zorder=1))
        for cl in centerlines:
            ax.plot(cl[:, 0], cl[:, 1], color="#ffffff", lw=0.8,
                    ls=(0, (6, 6)), zorder=2, alpha=0.9)

        # --- history / future trajectories: ego + every agent present now ---
        def track(pos, k):
            return pos.get(k)

        hist_idx = list(range(max(0, i - hn * step), i + 1, step))
        fut_idx = list(range(i, min(n_frames, i + fn * step + 1), step))

        ego_pos = {k: (ego_traj[k]["x"], ego_traj[k]["y"]) for k in range(n_frames)}
        _draw_polyline(ax, [ego_pos[k] for k in hist_idx], "ego", False, 3)
        _draw_polyline(ax, [ego_pos[k] for k in fut_idx], "ego", True, 3)
        for iid, xy in agent_xy.items():
            if i not in xy:
                continue
            grp = agent_grp[iid]
            h = [xy[k] for k in hist_idx if k in xy]
            f = [xy[k] for k in fut_idx if k in xy]
            _draw_polyline(ax, h, grp, False, 3)
            _draw_polyline(ax, f, grp, True, 3)

        # --- agent boxes (current frame) ---
        for a in agent_frames[i]:
            wl = a["wlh"]
            corners = box_corners(a["x"], a["y"], a["yaw"], wl[1], wl[0])
            col = agent_color(a["category"])
            ax.add_patch(MplPolygon(corners, closed=True, facecolor=col,
                                    edgecolor="black", lw=0.4, alpha=0.95, zorder=8))
            hx = a["x"] + math.cos(a["yaw"]) * wl[1] * 0.5
            hy = a["y"] + math.sin(a["yaw"]) * wl[1] * 0.5
            ax.plot([a["x"], hx], [a["y"], hy], color="black", lw=0.5, zorder=8.1)

        # --- ego vehicle (always centred) ---
        ego_corners = box_corners(cx, cy, e["yaw"], 4.6, 1.9)
        ax.add_patch(MplPolygon(ego_corners, closed=True,
                                facecolor=TRAJ_BASE["ego"], edgecolor="black",
                                lw=0.9, zorder=9))
        ax.annotate("", xy=(cx + math.cos(e["yaw"]) * 4.6,
                            cy + math.sin(e["yaw"]) * 4.6),
                    xytext=(cx, cy),
                    arrowprops=dict(arrowstyle="->", color="black", lw=1.4),
                    zorder=9.1)

        ax.set_xlim(cx - VIEW_HALF, cx + VIEW_HALF)
        ax.set_ylim(cy - VIEW_HALF, cy + VIEW_HALF)
        ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_edgecolor("#999999")
        ax.set_facecolor("#f4f1ea")

        fp = os.path.join(frames_dir, f"{i:03d}.png")
        fig.savefig(fp, bbox_inches="tight", pad_inches=0.05, facecolor="white")
        plt.close(fig)
        frame_paths.append(fp)

    # --- gif ---
    imgs = [Image.open(p).convert("RGB") for p in frame_paths]
    w0, h0 = imgs[0].size
    imgs = [im if im.size == (w0, h0) else im.resize((w0, h0)) for im in imgs]
    gif_path = os.path.join(out_dir, "topdown.gif")
    imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / GIF_FPS), loop=0)
    print(f"[{scene}] wrote {n_frames} frames + {os.path.relpath(gif_path, HERE)}")
    return gif_path


def main():
    metas = sorted(glob.glob(os.path.join(OUT, "*_meta.json")))
    by_scene = {}
    for m in metas:
        d = json.load(open(m))
        xodr = m.replace("_meta.json", ".xodr")
        if os.path.exists(xodr):
            by_scene[d["scene"]] = (m, xodr)

    wanted = sys.argv[1:]
    if wanted:
        by_scene = {k: v for k, v in by_scene.items() if k in wanted}
        for name in wanted:
            if name not in by_scene:
                print(f"!! unknown scene: {name}")

    print(f"rendering {len(by_scene)} scene(s) -> {os.path.relpath(TOPDOWN_DIR, HERE)}")
    for scene, (m, x) in sorted(by_scene.items()):
        render_scene(m, x)


if __name__ == "__main__":
    main()
