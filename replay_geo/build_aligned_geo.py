#!/usr/bin/env python
"""Aligned REAL vs SIM panels for the geo-map CARLA replays.

For every keyframe of a scene, builds one panel in the style of
nuscenes2xodr/output/aligned_full (self-contained, reads the nuScenes tables
directly — no nuscenes-devkit, no imports from nuscenes2xodr):

    +------------------------------+-----------+
    |  REAL  (6 nuScenes cameras)  |           |
    +------------------------------+ TOP-DOWN  |
    |  SIM   (6 CARLA cameras)     |  (BEV)    |
    +------------------------------+-----------+

  REAL      the 6 real nuScenes camera JPGs for that keyframe
            (v1.0-mini tables walked with plain json)
  SIM       the 6 CARLA views captured on the geo map with the RoadRunner
            buildings (replay_geo_carla.py --cameras --buildings)
  TOP-DOWN  the replay_geo.py frame: geo map + OSM footprints + RoadRunner
            boxes + agent boxes (auto-generated if missing)

Missing SIM cameras render as a placeholder, so the real side works even
before the CARLA run. Output: results/<base>/aligned/frames/NNN.png and
results/<base>/aligned/aligned.gif.

Usage (base Python 3.12 env):
    python build_aligned_geo.py                  # all geo scenes
    python build_aligned_geo.py --scene 0103
    python build_aligned_geo.py --fps 4 --cell-width 400
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RR_DIR = ROOT / "output_roadrunner"
NUSC = ROOT / "v1.0-mini"                    # dataroot (images relative to it)
TABLES = NUSC / "v1.0-mini"                  # the json tables
# results root; override with REPLAY_GEO_RESULTS (used by main.py -> OUTPUT/)
RESULTS = Path(os.environ.get("REPLAY_GEO_RESULTS", HERE / "results"))

CAM_GRID = [
    ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"],
    ["CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"],
]
PAD = 12
BG = (255, 255, 255)


def _font(size):
    for name in ("arial.ttf", "Arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()


def _corner_label(img, text, color=(255, 255, 255), right=False):
    d = ImageDraw.Draw(img)
    f = _font(max(14, img.width // 24))
    tb = d.textbbox((0, 0), text, font=f)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    pad = 6
    if right:
        box = [img.width - tw - 3 * pad, 0, img.width, th + 2 * pad]
        pos = (img.width - tw - 2 * pad, pad - tb[1])
    else:
        box = [0, 0, tw + 3 * pad, th + 2 * pad]
        pos = (pad, pad - tb[1])
    d.rectangle(box, fill=(20, 20, 20))
    d.text(pos, text, fill=color, font=f)


# ------------------------------------------------------------ nuScenes tables
def real_keyframe_files(scene_name):
    """Per-keyframe {CAM: jpg-path} dicts for a scene, in sample order."""
    scenes = json.loads((TABLES / "scene.json").read_text())
    samples = json.loads((TABLES / "sample.json").read_text())
    sdata = json.loads((TABLES / "sample_data.json").read_text())

    rec = next((s for s in scenes if s["name"] == scene_name), None)
    if rec is None:
        return []
    nxt = {s["token"]: s["next"] for s in samples}
    order, tok = [], rec["first_sample_token"]
    while tok:
        order.append(tok)
        tok = nxt.get(tok, "")

    by_sample = {}
    for sd in sdata:
        fn = sd["filename"]
        if not sd["is_key_frame"] or not fn.startswith("samples/CAM"):
            continue
        cam = fn.split("/")[1]
        by_sample.setdefault(sd["sample_token"], {})[cam] = NUSC / fn
    return [by_sample.get(t, {}) for t in order]


# ------------------------------------------------------------ montage pieces
def cell_image(path, size, missing_text):
    if path is not None and Path(path).is_file():
        img = Image.open(path).convert("RGB").resize(size, Image.LANCZOS)
    else:
        img = Image.new("RGB", size, (60, 60, 60))
        d = ImageDraw.Draw(img)
        f = _font(size[1] // 8)
        tb = d.textbbox((0, 0), missing_text, font=f)
        d.text(((size[0] - tb[2]) // 2, (size[1] - tb[3]) // 2),
               missing_text, fill=(160, 160, 160), font=f)
    return img


def montage6(files, cell, tag):
    """2x3 zero-padding camera montage with per-cell labels + REAL/SIM tag."""
    w, h = cell
    out = Image.new("RGB", (3 * w, 2 * h), BG)
    for r, row in enumerate(CAM_GRID):
        for c, cam in enumerate(row):
            img = cell_image(files.get(cam), cell, "missing")
            _corner_label(img, cam.replace("CAM_", "").replace("_", " "))
            out.paste(img, (c * w, r * h))
    _corner_label(out, tag, color=(255, 90, 90), right=True)
    return out


def ensure_topdown_frames(base, args):
    """The replay_geo.py per-keyframe stills; generate them when absent.

    Ego-centred (--follow) by default, matching the aligned_full BEV column;
    --full-map-topdown switches to the whole-map view.
    """
    sub = "frames" if args.full_map_topdown else "frames_panel"
    fdir = RESULTS / base / sub
    if not (fdir.is_dir() and any(fdir.glob("*.png"))):
        print(f"  generating {'full-map' if args.full_map_topdown else 'ego-centred'} "
              f"top-down frames for {base} ...")
        cmd = [sys.executable, str(HERE / "replay_geo.py"), "--scene", base]
        if args.full_map_topdown:
            cmd += ["--save-frames"]
        else:
            cmd += ["--panel-frames", "--window", str(args.window)]
        subprocess.run(cmd, check=True, cwd=str(HERE))
    return sorted(fdir.glob("*.png"))


# ------------------------------------------------------------ panel per scene
def build_scene(base, args):
    meta = json.loads((RR_DIR / f"{base}_meta.json").read_text())
    scene_name = meta["scene"]                       # e.g. scene-0103
    real = real_keyframe_files(scene_name)
    if not real:
        print(f"[{base}] no nuScenes samples found for {scene_name}, skipped")
        return
    td = ensure_topdown_frames(base, args)
    sim_dir = RESULTS / base / "carla_cams"
    have_sim = sim_dir.is_dir()

    n = min(len(real), len(td)) if td else len(real)
    out_dir = RESULTS / base / "aligned"
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    cell = (args.cell_width, int(args.cell_width * 9 / 16))
    paths = []
    for i in range(n):
        real_m = montage6(real[i], cell, "REAL")
        sim_files = {cam: sim_dir / cam / f"{i:03d}.png"
                     for cam in sum(CAM_GRID, [])} if have_sim else {}
        sim_m = montage6(sim_files, cell, "SIM")

        H = real_m.height + PAD + sim_m.height
        left = Image.new("RGB", (real_m.width, H), BG)
        left.paste(real_m, (0, 0))
        left.paste(sim_m, (0, real_m.height + PAD))

        panel = left
        if td:
            td_img = Image.open(td[i]).convert("RGB")
            td_img = td_img.resize(
                (int(round(td_img.width * H / td_img.height)), H), Image.LANCZOS)
            _corner_label(td_img, "TOP-DOWN", color=(255, 90, 90), right=True)
            panel = Image.new("RGB", (left.width + PAD + td_img.width, H), BG)
            panel.paste(left, (0, 0))
            panel.paste(td_img, (left.width + PAD, 0))

        fp = frames_dir / f"{i:03d}.png"
        panel.save(fp)
        paths.append(fp)

    imgs = [Image.open(p).convert("RGB") for p in paths]
    gif = out_dir / "aligned.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / args.fps), loop=0)
    note = "" if have_sim else "  (no sim frames yet - SIM side is placeholders)"
    print(f"[{base}] {n} frames -> {gif.relative_to(ROOT)}"
          f"  ({imgs[0].width}x{imgs[0].height}){note}")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--scene", help="scene id substring, e.g. 0103")
    p.add_argument("--fps", type=float, default=2.0, help="GIF playback fps")
    p.add_argument("--cell-width", type=int, default=480,
                   help="width of each camera cell (16:9)")
    p.add_argument("--window", type=float, default=80.0,
                   help="ego-centred top-down window size (m)")
    p.add_argument("--full-map-topdown", action="store_true",
                   help="use the whole-map top-down instead of ego-centred")
    args = p.parse_args()

    bases = sorted(d.name for d in RR_DIR.iterdir()
                   if d.is_dir() and (d / f"{d.name}_geo.xodr").is_file())
    if args.scene:
        bases = [b for b in bases if args.scene in b]
    if not bases:
        sys.exit(f"no scene matching '{args.scene}'")
    for base in bases:
        build_scene(base, args)


if __name__ == "__main__":
    main()
