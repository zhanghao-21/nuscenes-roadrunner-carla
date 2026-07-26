#!/usr/bin/env python3
"""Stage an FBX/OpenDRIVE map package locally for CARLA's `make import`."""

import argparse
import json
import os
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--fbx", required=True, help="RoadRunner FBX export")
    parser.add_argument("--output-root", required=True,
                        help="local staging directory; never C:\\carla\\Import")
    args = parser.parse_args()
    with open(args.manifest, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    output_root = os.path.abspath(args.output_root)
    carla_import = os.path.normcase(os.path.abspath(r"C:\carla\Import"))
    if os.path.normcase(output_root).startswith(carla_import):
        raise RuntimeError("refusing to write directly into C:\\carla\\Import")
    name = manifest["map"]["asset_name"]
    package_name = manifest["map"].get("package_name", name)
    package = os.path.join(output_root, package_name)
    os.makedirs(package, exist_ok=True)
    fbx_target = os.path.join(package, name + ".fbx")
    xodr_target = os.path.join(package, name + ".xodr")
    shutil.copy2(os.path.abspath(args.fbx), fbx_target)
    shutil.copy2(manifest["source"]["xodr"], xodr_target)
    description = {
        "maps": [{"name": name, "source": "./" + name + ".fbx",
                  "xodr": "./" + name + ".xodr", "use_carla_materials": True}],
        "props": [],
    }
    with open(os.path.join(package, package_name + ".json"), "w", encoding="utf-8") as stream:
        json.dump(description, stream, indent=2)
        stream.write("\n")
    print("Staged CARLA import package at " + package)
    print("Review it, then copy this folder into C:\\carla\\Import and run make import.")
    print("Expected imported level: " + manifest["map"]["unreal_source_level"])


if __name__ == "__main__":
    main()
