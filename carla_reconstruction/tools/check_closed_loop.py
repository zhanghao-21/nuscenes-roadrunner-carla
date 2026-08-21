#!/usr/bin/env python3
"""Read-only checks for Traffic Manager and SUMO hybrid prerequisites."""

import argparse
import json
import os
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--carla-root", default=r"C:\carla")
    parser.add_argument("--sumo-home", default=os.environ.get(
        "SUMO_HOME", r"C:\Traffic software\SUMO"))
    args = parser.parse_args()

    with open(os.path.abspath(args.manifest), "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    package = manifest["map"]["package_name"]
    source = manifest["map"]["asset_name"]
    target = manifest["map"]["target_asset_name"]
    map_folder = os.path.join(
        os.path.abspath(args.carla_root), "Unreal", "CarlaUE4", "Content",
        package, "Maps", source)
    required_files = {
        "CARLA SUMO converter": os.path.join(
            args.carla_root, "Co-Simulation", "Sumo", "util", "netconvert_carla.py"),
        "CARLA SUMO bridge": os.path.join(
            args.carla_root, "Co-Simulation", "Sumo", "run_synchronization.py"),
        "decorated OpenDRIVE": os.path.join(
            map_folder, "OpenDrive", target + ".xodr"),
        "decorated TM binary": os.path.join(
            map_folder, "TM", target + ".bin"),
        "SUMO executable": os.path.join(args.sumo_home, "bin", "sumo.exe"),
        "SUMO netconvert": os.path.join(args.sumo_home, "bin", "netconvert.exe"),
        "SUMO Python tools": os.path.join(args.sumo_home, "tools", "sumolib", "__init__.py"),
    }
    problems = []
    for label, path in required_files.items():
        if os.path.isfile(path):
            print("OK   %-24s %s" % (label, path))
        else:
            problems.append("missing %s: %s" % (label, path))

    try:
        import carla
        print("OK   %-24s %s" % ("CARLA Python API", carla.__file__))
    except ImportError as error:
        problems.append("CARLA Python API is unavailable in this environment: %s" % error)

    tools = os.path.join(os.path.abspath(args.sumo_home), "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    try:
        import sumolib
        import traci
        print("OK   %-24s %s" % ("SUMO Python API", sumolib.__file__))
        print("OK   %-24s %s" % ("TraCI Python API", traci.__file__))
    except ImportError as error:
        problems.append("SUMO/TraCI Python API is unavailable: %s" % error)

    if problems:
        for problem in problems:
            print("FAIL " + problem)
        return 1
    print("All closed-loop prerequisites passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
