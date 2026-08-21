#!/usr/bin/env python3
"""Stage a CARLA Filmbox/OpenDRIVE package for CARLA's `make import`."""

import argparse
import json
import os
import shutil
import sys


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from nusc_carla.xodr import write_carla_compatible_xodr  # noqa: E402


def stage_package(manifest_path, fbx_path, output_root):
    """Copy authoritative map inputs and RoadRunner metadata into staging."""
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    output_root = os.path.abspath(output_root)
    carla_import = os.path.normcase(os.path.abspath(r"C:\carla\Import"))
    if os.path.normcase(output_root).startswith(carla_import):
        raise RuntimeError("refusing to write directly into C:\\carla\\Import")
    name = manifest["map"]["asset_name"]
    package_name = manifest["map"].get("package_name", name)
    package = os.path.join(output_root, package_name)
    os.makedirs(package, exist_ok=True)

    fbx_path = os.path.abspath(fbx_path)
    if not os.path.isfile(fbx_path):
        raise FileNotFoundError("RoadRunner FBX not found: " + fbx_path)
    fbx_target = os.path.join(package, name + ".fbx")
    xodr_target = os.path.join(package, name + ".xodr")
    shutil.copy2(fbx_path, fbx_target)
    # Preserve the nuScenes-derived geometry and topology as the runtime
    # authority, while remapping only road/junction ID collisions that CARLA
    # 0.9.15 otherwise interprets as dead-end roads.
    write_carla_compatible_xodr(manifest["source"]["xodr"], xodr_target)

    rrdata_source = os.path.splitext(fbx_path)[0] + ".rrdata.xml"
    if not os.path.isfile(rrdata_source):
        raise FileNotFoundError(
            "RoadRunner CARLA material metadata not found: " + rrdata_source
            + ". Re-export with export_roadrunner_base (CARLA Filmbox).")
    shutil.copy2(rrdata_source, os.path.join(package, name + ".rrdata.xml"))

    description = {
        "maps": [{"name": name, "source": "./" + name + ".fbx",
                  "xodr": "./" + name + ".xodr", "use_carla_materials": True}],
        "props": [],
    }
    with open(os.path.join(package, package_name + ".json"), "w", encoding="utf-8") as stream:
        json.dump(description, stream, indent=2)
        stream.write("\n")
    return package, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--fbx", required=True, help="RoadRunner FBX export")
    parser.add_argument("--output-root", required=True,
                        help="local staging directory; never C:\\carla\\Import")
    args = parser.parse_args()
    package, manifest = stage_package(args.manifest, args.fbx, args.output_root)
    print("Staged CARLA import package at " + package)
    print("Review it, then copy this folder into C:\\carla\\Import and run make import.")
    print("Expected imported level: " + manifest["map"]["unreal_source_level"])


if __name__ == "__main__":
    main()
