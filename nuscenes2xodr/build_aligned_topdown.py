#!/usr/bin/env python3
"""
Combine the three aligned views into a single per-frame panel + GIF:

    +-----------------------------+---------------+
    |  REAL  (6 nuScenes cameras) |               |
    +-----------------------------+   TOP-DOWN    |
    |  SIM   (6 CARLA cameras)    |   (BEV)       |
    +-----------------------------+---------------+

Sources (all sampled at the nuScenes keyframe rate, so frame i is the same
timestep everywhere):
  - output/aligned/<scene>/comparison.gif   REAL-over-SIM camera montage
  - output/topdown/<scene>/frames/NNN.png   ego-centric bird's-eye view

Output, per scene, into output/aligned_topdown/<scene>/:
  - frames/NNN.png
  - aligned.gif

Usage:
  python build_aligned_topdown.py                 # all scenes
  python build_aligned_topdown.py scene-0061 ...  # selected scenes
"""

import os
import sys
import glob

from PIL import Image, ImageSequence, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
ALIGNED = os.path.join(OUT, "aligned")            # REAL/SIM montage (replay)
TOPDOWN = os.path.join(OUT, "topdown")
DST = os.path.join(OUT, "aligned_topdown")

GIF_FPS = 4
PAD = 12               # white gutter between the camera block and the BEV
BG = (255, 255, 255)


def _font(size):
    for name in ("Arial.ttf", "arial.ttf",
                 "/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/Library/Fonts/Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:
            pass
    return ImageFont.load_default()


def _label(img, text):
    """Draw a REAL/SIM-style tag in the top-right corner of img (in place)."""
    d = ImageDraw.Draw(img)
    f = _font(max(18, img.width // 26))
    tb = d.textbbox((0, 0), text, font=f)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    pad = 8
    x1, y1 = img.width - tw - 3 * pad, 0
    d.rectangle([x1, y1, img.width, th + 2 * pad], fill=(20, 20, 20))
    d.text((x1 + pad, y1 + pad - tb[1]), text, fill=(255, 90, 90), font=f)


def gif_frames(path):
    im = Image.open(path)
    return [f.convert("RGB").copy() for f in ImageSequence.Iterator(im)]


def build_scene(scene):
    comp_gif = os.path.join(ALIGNED, scene, "comparison.gif")
    td_dir = os.path.join(TOPDOWN, scene, "frames")
    if not os.path.exists(comp_gif):
        print(f"!! {scene}: missing {os.path.relpath(comp_gif, HERE)}"); return
    if not os.path.isdir(td_dir):
        print(f"!! {scene}: missing top-down frames"); return

    comp = gif_frames(comp_gif)
    td_paths = sorted(glob.glob(os.path.join(td_dir, "*.png")))
    n = min(len(comp), len(td_paths))
    if n == 0:
        print(f"!! {scene}: nothing to combine"); return

    out_dir = os.path.join(DST, scene)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    H = comp[0].height                      # 1080; panel height for everything
    out_paths = []
    for i in range(n):
        left = comp[i]                      # REAL over SIM, already labelled
        td = Image.open(td_paths[i]).convert("RGB")
        # scale the square BEV to the panel height
        tw = int(round(td.width * H / td.height))
        td = td.resize((tw, H), Image.LANCZOS)
        _label(td, "TOP-DOWN")

        W = left.width + PAD + td.width
        canvas = Image.new("RGB", (W, H), BG)
        canvas.paste(left, (0, 0))
        canvas.paste(td, (left.width + PAD, 0))

        fp = os.path.join(frames_dir, f"{i:03d}.png")
        canvas.save(fp)
        out_paths.append(fp)

    imgs = [Image.open(p).convert("RGB") for p in out_paths]
    gif_path = os.path.join(out_dir, "aligned.gif")
    imgs[0].save(gif_path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / GIF_FPS), loop=0)
    print(f"[{scene}] {n} frames + {os.path.relpath(gif_path, HERE)}  "
          f"({imgs[0].width}x{imgs[0].height})")


def main():
    scenes = sorted(os.path.basename(os.path.dirname(p))
                    for p in glob.glob(os.path.join(TOPDOWN, "*", "frames")))
    wanted = sys.argv[1:]
    if wanted:
        scenes = [s for s in scenes if s in wanted]
    print(f"combining {len(scenes)} scene(s) -> {os.path.relpath(DST, HERE)}")
    for s in scenes:
        build_scene(s)


if __name__ == "__main__":
    main()
