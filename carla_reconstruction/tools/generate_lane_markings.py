#!/usr/bin/env python3
"""Generate a CARLA/Unreal lane-marking overlay from a scene manifest."""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from nusc_carla.markings import generate_marking_obj
from nusc_carla.nuscenes_markings import generate_nuscenes_marking_obj


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True, help="output .obj path")
    parser.add_argument("--source", choices=("nuscenes", "opendrive"), default="nuscenes",
                        help="surveyed divider geometry (default), or the legacy estimated overlay")
    parser.add_argument("--map-json", help="override the manifest nuScenes map-expansion JSON path")
    parser.add_argument("--step", type=float, default=None,
                        help="maximum ribbon sampling step in metres")
    parser.add_argument("--dash-length", type=float, default=3.0)
    parser.add_argument("--dash-gap", type=float, default=6.0)
    parser.add_argument("--height-cm", type=float, default=2.0,
                        help="paint height above the road; avoids z-fighting")
    parser.add_argument("--stripe-width", type=float, default=0.13,
                        help="nuScenes visual stripe width in metres (not surveyed)")
    parser.add_argument("--double-separation", type=float, default=0.24,
                        help="distance between double-stripe centerlines in metres (not surveyed)")
    parser.add_argument("--duplicate-tolerance", type=float, default=0.30,
                        help=("maximum separation in metres for treating "
                              "parallel lane boundaries as the same marking"))
    parser.add_argument("--no-deduplicate", action="store_true",
                        help="legacy OpenDRIVE source only: render every roadMark boundary")
    parser.add_argument("--preserve-double-dashed", action="store_true",
                        help=("legacy OpenDRIVE source: retain two dashed stripes; "
                              "the nuScenes source always retains explicit double styles"))
    return parser


def run(args):
    step = args.step if args.step is not None else (0.5 if args.source == "nuscenes" else 1.0)
    if args.source == "nuscenes":
        if args.no_deduplicate or args.duplicate_tolerance != 0.30:
            raise ValueError("distance-based deduplication options apply only to --source opendrive")
        stats = generate_nuscenes_marking_obj(
            args.manifest, os.path.abspath(args.output), map_json=args.map_json,
            step=step, dash_length=args.dash_length, dash_gap=args.dash_gap,
            height_m=args.height_cm / 100.0, stripe_width=args.stripe_width,
            double_separation=args.double_separation)
        print("Generated nuScenes divider overlay at " + os.path.abspath(args.output))
        print("Ribbons: %d; invented boundaries: 0; double-dashed collapses: 0" % stats["ribbons"])
        audit = stats["road_alignment"]
        if audit["outside_sample_count"]:
            print("WARNING: %d/%d paint samples lie outside the approximate OpenDRIVE lane envelope. "
                  "Inspect the JSON alignment report; surveyed positions were not moved." %
                  (audit["outside_sample_count"], audit["sample_count"]))
        if not stats["vertices"]:
            print("No explicitly styled dividers in this patch; the rebuild will remove the old overlay.")
        return stats
    if args.map_json:
        raise ValueError("--map-json requires --source nuscenes")
    with open(args.manifest, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    stats = generate_marking_obj(
        manifest["source"]["xodr"], os.path.abspath(args.output),
        step=step, dash_length=args.dash_length, dash_gap=args.dash_gap,
        double_separation=args.double_separation,
        height_m=args.height_cm / 100.0,
        deduplicate=not args.no_deduplicate,
        duplicate_tolerance=args.duplicate_tolerance,
        preserve_double_broken=args.preserve_double_dashed)
    print("Generated lane-marking overlay at " + os.path.abspath(args.output))
    print(json.dumps(stats, sort_keys=True))
    return stats


def main():
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
