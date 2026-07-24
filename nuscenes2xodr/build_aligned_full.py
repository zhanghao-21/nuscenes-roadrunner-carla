#!/usr/bin/env python3
"""
Combine the aligned views into one per-frame panel + GIF:

    +-----------------------------+-----------+-----------+
    |  REAL  (6 nuScenes cameras) | LIDAR     |           |
    +-----------------------------+-----------+ TOP-DOWN  |
    |  SIM   (6 CARLA cameras)    | LIDAR SIM |  (BEV)    |
    +-----------------------------+-----------+-----------+

All are the same nuScenes keyframe at frame i (ego-centred, north-up BEVs). The
middle column mirrors the camera column: the real nuScenes LIDAR_TOP BEV on top,
the CARLA-captured LiDAR BEV below it.

Sources:
  - output/aligned/<scene>/comparison.gif       REAL-over-SIM 6-camera montage
  - output/lidar/<scene>/frames/NNN.png         LIDAR_TOP BEV     (plot_lidar.py)
  - output/lidar_carla/<scene>/frames/NNN.png   CARLA LiDAR BEV   (plot_lidar_carla.py)
  - output/topdown/<scene>/frames/NNN.png       reconstructed BEV (plot_topdown.py)

Output, per scene, into output/aligned_full/<scene>/:
  - frames/NNN.png
  - aligned.gif

Usage:
  python build_aligned_full.py                 # all scenes
  python build_aligned_full.py scene-0061 ...  # selected scenes
"""

import os
import sys
import glob

from PIL import Image, ImageSequence, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output")
ALIGNED = os.path.join(OUT, "aligned")        # REAL/SIM montage (replay)
LIDAR = os.path.join(OUT, "lidar")            # nuScenes LIDAR_TOP BEV
LIDAR_CARLA = os.path.join(OUT, "lidar_carla")  # CARLA-captured LiDAR BEV
TOPDOWN = os.path.join(OUT, "topdown")
DST = os.path.join(OUT, "aligned_full")

GIF_FPS = 4
PAD = 12
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
    f = _font(max(18, img.width // 20))
    tb = d.textbbox((0, 0), text, font=f)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    pad = 8
    d.rectangle([img.width - tw - 3*pad, 0, img.width, th + 2*pad], fill=(20, 20, 20))
    d.text((img.width - tw - 2*pad, pad - tb[1]), text, fill=(255, 90, 90), font=f)


def gif_frames(path):
    im = Image.open(path)
    return [f.convert("RGB").copy() for f in ImageSequence.Iterator(im)]


def scaled_to_height(img, H):
    w = int(round(img.width * H / img.height))
    return img.resize((w, H), Image.LANCZOS)


def stack_v(top, bottom, pad, bg):
    """Stack two images vertically (centred), with `pad` px between them."""
    w = max(top.width, bottom.width)
    col = Image.new("RGB", (w, top.height + pad + bottom.height), bg)
    col.paste(top, ((w - top.width) // 2, 0))
    col.paste(bottom, ((w - bottom.width) // 2, top.height + pad))
    return col


def build_scene(scene):
    comp_gif = os.path.join(ALIGNED, scene, "comparison.gif")
    td_dir = os.path.join(TOPDOWN, scene, "frames")
    li_dir = os.path.join(LIDAR, scene, "frames")
    lc_dir = os.path.join(LIDAR_CARLA, scene, "frames")
    for req in (comp_gif, td_dir, li_dir, lc_dir):
        if not os.path.exists(req):
            print(f"!! {scene}: missing {os.path.relpath(req, HERE)}"); return

    comp = gif_frames(comp_gif)
    td = sorted(glob.glob(os.path.join(td_dir, "*.png")))
    li = sorted(glob.glob(os.path.join(li_dir, "*.png")))
    lc = sorted(glob.glob(os.path.join(lc_dir, "*.png")))
    n = min(len(comp), len(td), len(li), len(lc))
    if n == 0:
        print(f"!! {scene}: nothing to combine"); return

    out_dir = os.path.join(DST, scene)
    frames_dir = os.path.join(out_dir, "frames")
    os.makedirs(frames_dir, exist_ok=True)

    H = comp[0].height                         # panel height for everything
    half = (H - PAD) // 2                       # each stacked LiDAR BEV row height
    out_paths = []
    for i in range(n):
        left = comp[i]                                     # REAL over SIM (labelled)
        # second column: nuScenes LIDAR (top) over CARLA LiDAR (bottom), like the
        # REAL/SIM camera column to its left
        lidar_real = scaled_to_height(Image.open(li[i]).convert("RGB"), half)
        lidar_sim = scaled_to_height(Image.open(lc[i]).convert("RGB"), half)
        _label(lidar_real, "LIDAR")
        _label(lidar_sim, "LIDAR SIM")
        lidar = stack_v(lidar_real, lidar_sim, PAD, BG)
        topd = scaled_to_height(Image.open(td[i]).convert("RGB"), H)
        _label(topd, "TOP-DOWN")

        W = left.width + PAD + lidar.width + PAD + topd.width
        canvas = Image.new("RGB", (W, H), BG)
        x = 0
        canvas.paste(left, (x, 0));   x += left.width + PAD
        canvas.paste(lidar, (x, 0));  x += lidar.width + PAD
        canvas.paste(topd, (x, 0))

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
