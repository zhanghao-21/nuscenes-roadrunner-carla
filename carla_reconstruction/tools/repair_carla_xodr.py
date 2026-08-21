#!/usr/bin/env python3
"""Repair CARLA-incompatible road/junction ID collisions in an OpenDRIVE file."""

import argparse
import json
import os
import sys


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from nusc_carla.xodr import write_carla_compatible_xodr  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    output = parser.add_mutually_exclusive_group(required=True)
    output.add_argument("--output")
    output.add_argument("--in-place", action="store_true")
    args = parser.parse_args()
    source = os.path.abspath(args.input)
    target = source if args.in_place else os.path.abspath(args.output)
    mapping = write_carla_compatible_xodr(source, target)
    print(json.dumps({
        "input": source,
        "output": target,
        "remapped_count": len(mapping),
        "junction_id_mapping": {str(key): value for key, value in mapping.items()},
    }, sort_keys=True))


if __name__ == "__main__":
    main()
