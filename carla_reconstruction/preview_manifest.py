#!/usr/bin/env python3
"""Render an offline QA preview of a persistent-CARLA scene manifest."""

import argparse
import json
import math
import os

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from nusc_carla.xodr import read_xodr


def box_polygon(box):
    c = math.cos(box["yaw_rad"])
    s = math.sin(box["yaw_rad"])
    corners = []
    for x, y in ((box["half_x"], box["half_y"]),
                 (box["half_x"], -box["half_y"]),
                 (-box["half_x"], -box["half_y"]),
                 (-box["half_x"], box["half_y"])):
        corners.append((box["x"] + c * x - s * y,
                        box["y"] + s * x + c * y))
    return corners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    manifest_path = os.path.abspath(args.manifest)
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    with open(manifest["trajectories"]["meta"], "r", encoding="utf-8") as stream:
        meta = json.load(stream)
    xodr = read_xodr(manifest["source"]["xodr"], sample_spacing=1.5)

    figure, axis = plt.subplots(figsize=(10, 9), dpi=140)
    for road in xodr["roads"]:
        if not road["samples"]:
            continue
        xs = [p[0] for p in road["samples"]]
        ys = [p[1] for p in road["samples"]]
        axis.plot(xs, ys, color="#8291a6", linewidth=max(1.0, road["width"] * 1.25),
                  solid_capstyle="round", zorder=1)
        axis.plot(xs, ys, color="#d8dde5", linewidth=0.35, alpha=0.8, zorder=2)

    for building in manifest["environment"]["buildings"]:
        axis.add_patch(Polygon(box_polygon(building), closed=True,
                               facecolor="#ba7b4b", edgecolor="#6d3a1c",
                               linewidth=0.45, alpha=0.75, zorder=3))
    trees = manifest["environment"]["trees"]
    if trees:
        axis.scatter([tree["x"] for tree in trees], [tree["y"] for tree in trees],
                     s=18, color="#2f8f54", edgecolors="#16492b", linewidths=0.4,
                     label="trees", zorder=5)
    signs = manifest["environment"]["traffic_signs"]
    if signs:
        axis.scatter([sign["x"] for sign in signs], [sign["y"] for sign in signs],
                     s=28, marker="s", color="#ef4444", label="OSM signs", zorder=6)
    lights = manifest["environment"]["traffic_lights"]
    if lights:
        axis.scatter([light["x"] for light in lights], [light["y"] for light in lights],
                     s=32, marker="^", color="#facc15", edgecolors="black",
                     linewidths=0.4, label="OpenDRIVE lights", zorder=7)
    static_objects = manifest["environment"].get("nuscenes_static_objects", [])
    if static_objects:
        axis.scatter([item["x"] for item in static_objects],
                     [item["y"] for item in static_objects],
                     s=20, marker="x", color="#7c3aed",
                     label="nuScenes static objects", zorder=7)

    ego = meta.get("ego_trajectory_local", [])
    if ego:
        axis.plot([point["x"] for point in ego], [point["y"] for point in ego],
                  color="#ff7a00", linewidth=2.2, label="ego trajectory", zorder=8)
    for frame in meta.get("agent_frames", []):
        if frame:
            axis.scatter([item["x"] for item in frame], [item["y"] for item in frame],
                         s=2, color="#2474d2", alpha=0.08, zorder=4)

    bounds = manifest["map"]["ground_bounds"]
    axis.set_xlim(bounds["x_min"], bounds["x_max"])
    axis.set_ylim(bounds["y_min"], bounds["y_max"])
    axis.set_aspect("equal", adjustable="box")
    axis.set_facecolor("#eef2e8")
    axis.grid(True, linewidth=0.25, alpha=0.3)
    axis.set_title("%s: persistent CARLA reconstruction preview" % manifest["scene"])
    axis.set_xlabel("OpenDRIVE local x (m)")
    axis.set_ylabel("OpenDRIVE local y (m)")
    axis.legend(loc="upper right", fontsize=8)
    figure.tight_layout()
    output = (os.path.abspath(args.output) if args.output else
              os.path.join(os.path.dirname(manifest_path), "manifest_preview.png"))
    figure.savefig(output, bbox_inches="tight")
    plt.close(figure)
    print("Wrote " + output)


if __name__ == "__main__":
    main()
