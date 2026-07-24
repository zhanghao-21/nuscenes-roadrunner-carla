#!/usr/bin/env python
"""Assemble GIFs from the CARLA chase-cam frames recorded by replay_geo_carla.py.

Scans ``results/<base>/carla_frames/*.png`` (written with --record) and writes
``results/<base>/<base>_carla.gif``. Run in the base Python 3.12 env (needs
Pillow); the carla915 env doesn't have Pillow, so recording and GIF assembly
are separate steps.

Usage:
    python make_gifs.py                 # all scenes with recorded frames
    python make_gifs.py --scene 0103    # one scene
    python make_gifs.py --fps 4 --scale 0.5
"""

import argparse
import os
import sys
from pathlib import Path

from PIL import Image

# results root; override with REPLAY_GEO_RESULTS (used by main.py -> OUTPUT/)
RESULTS = Path(os.environ.get("REPLAY_GEO_RESULTS",
                              Path(__file__).resolve().parent / "results"))


def build_gif(base, fps, scale):
    frames_dir = RESULTS / base / "carla_frames"
    pngs = sorted(frames_dir.glob("*.png"))
    if not pngs:
        return None
    images = []
    for p in pngs:
        img = Image.open(p).convert("RGB")
        if scale != 1.0:
            img = img.resize((int(img.width * scale), int(img.height * scale)),
                             Image.LANCZOS)
        images.append(img.quantize(colors=256, dither=Image.Dither.NONE))
    gif = RESULTS / base / (base + "_carla.gif")
    images[0].save(gif, save_all=True, append_images=images[1:],
                   duration=int(1000 / fps), loop=0, optimize=False)
    return gif, len(images)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--scene", help="scene id substring, e.g. 0103")
    p.add_argument("--fps", type=float, default=2.0,
                   help="playback fps (frames are 2 Hz keyframes; 2 = real time)")
    p.add_argument("--scale", type=float, default=0.5,
                   help="resize factor for the GIF (0.5 -> 640x360 from 1280x720)")
    args = p.parse_args()

    if not RESULTS.is_dir():
        sys.exit("no results folder yet - record frames first "
                 "(replay_geo_carla.py --record)")
    bases = sorted(d.name for d in RESULTS.iterdir()
                   if (d / "carla_frames").is_dir())
    if args.scene:
        bases = [b for b in bases if args.scene in b]
    if not bases:
        sys.exit("no recorded carla_frames found"
                 + (" for '%s'" % args.scene if args.scene else "")
                 + " - run replay_geo_carla.py --record first")

    for base in bases:
        out = build_gif(base, args.fps, args.scale)
        if out is None:
            print("[%s] no frames, skipped" % base)
        else:
            gif, n = out
            print("[%s] %d frames -> %s" % (base, n, gif.relative_to(RESULTS.parent)))


if __name__ == "__main__":
    main()
