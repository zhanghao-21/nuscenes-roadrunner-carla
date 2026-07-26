#!/usr/bin/env python3
"""Prepare persistent-CARLA reconstruction manifests for nuScenes scenes."""

import argparse
import json
import os
import sys

from nusc_carla.manifest import build_manifest, discover_scenes, resolve_scene, write_manifest


HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SOURCE = os.environ.get(
    "NUSCENES_RR_OUTPUT",
    os.path.abspath(os.path.join(HERE, "..", "output_roadrunner")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", help="scene substring (e.g. 0103), or 'all'")
    parser.add_argument("--source-root", default=DEFAULT_SOURCE,
                        help="read-only output_roadrunner directory")
    parser.add_argument("--output-root", default=os.path.join(HERE, "generated"))
    parser.add_argument("--nuscenes-dataroot",
                        help="raw nuScenes root; defaults to a v1.0-mini sibling of source-root")
    parser.add_argument("--nuscenes-version", default="v1.0-mini")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--tree-spacing", type=float, default=13.0,
                        help="metres between candidates in mapped green areas")
    parser.add_argument("--tree-road-clearance", type=float, default=6.0,
                        help="extra metres outside the nearest road half-width")
    parser.add_argument("--max-trees", type=int, default=400)
    args = parser.parse_args()

    if args.list:
        for scene in discover_scenes(args.source_root):
            print(scene)
        return 0
    if not args.scene:
        parser.error("--scene is required unless --list is used")

    scenes = (discover_scenes(args.source_root) if args.scene.lower() == "all"
              else [resolve_scene(args.scene, args.source_root)])
    if not scenes:
        raise RuntimeError("no scene metadata found under " + args.source_root)

    summary = {}
    for scene in scenes:
        manifest = build_manifest(
            scene, args.source_root, tree_spacing=args.tree_spacing,
            tree_road_clearance=args.tree_road_clearance,
            max_trees=args.max_trees,
            nuscenes_dataroot=args.nuscenes_dataroot,
            nuscenes_version=args.nuscenes_version)
        scene_output = os.path.join(os.path.abspath(args.output_root), scene)
        path = os.path.join(scene_output, "scene_manifest.json")
        write_manifest(manifest, path)
        summary[scene] = manifest["counts"]
        print("Wrote " + path)
    print(json.dumps(summary if len(scenes) > 1 else summary[scenes[0]], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
