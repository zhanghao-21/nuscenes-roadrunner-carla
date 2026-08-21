#!/usr/bin/env python3
"""Generate a CARLA/Unreal lane-marking overlay from a scene manifest."""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from nusc_carla.markings import generate_marking_obj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True, help="output .obj path")
    parser.add_argument("--step", type=float, default=1.0,
                        help="maximum ribbon sampling step in metres")
    parser.add_argument("--dash-length", type=float, default=3.0)
    parser.add_argument("--dash-gap", type=float, default=6.0)
    parser.add_argument("--height-cm", type=float, default=2.0,
                        help="paint height above the road; avoids z-fighting")
    parser.add_argument("--duplicate-tolerance", type=float, default=0.30,
                        help=("maximum separation in metres for treating "
                              "parallel lane boundaries as the same marking"))
    parser.add_argument("--no-deduplicate", action="store_true",
                        help="render every OpenDRIVE roadMark boundary")
    parser.add_argument("--preserve-double-dashed", action="store_true",
                        help=("render OpenDRIVE 'broken broken' as two "
                              "stripes instead of the CARLA visual default"))
    args = parser.parse_args()
    with open(args.manifest, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    stats = generate_marking_obj(
        manifest["source"]["xodr"], os.path.abspath(args.output),
        step=args.step, dash_length=args.dash_length, dash_gap=args.dash_gap,
        height_m=args.height_cm / 100.0,
        deduplicate=not args.no_deduplicate,
        duplicate_tolerance=args.duplicate_tolerance,
        preserve_double_broken=args.preserve_double_dashed)
    print("Generated lane-marking overlay at " + os.path.abspath(args.output))
    print(json.dumps(stats, sort_keys=True))


if __name__ == "__main__":
    main()
