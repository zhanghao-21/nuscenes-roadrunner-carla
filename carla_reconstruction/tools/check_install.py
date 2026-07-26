#!/usr/bin/env python3
"""Read-only checks for the source-built CARLA/Unreal prerequisites."""

import argparse
import json
import os
import sys


def asset_file(content_root, asset_path):
    relative = asset_path.replace("/Game/", "", 1).replace("/", os.sep)
    return os.path.join(content_root, relative + ".uasset")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carla-root", default=r"C:\carla")
    parser.add_argument("--unreal-root", default=r"C:\UnrealEngine")
    parser.add_argument("--catalog", default=os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "asset_catalog.json")))
    args = parser.parse_args()
    project = os.path.join(args.carla_root, "Unreal", "CarlaUE4", "CarlaUE4.uproject")
    editor = os.path.join(args.unreal_root, "Engine", "Binaries", "Win64", "UE4Editor-Cmd.exe")
    content = os.path.join(args.carla_root, "Unreal", "CarlaUE4", "Content")
    problems = []
    for label, path in (("CARLA project", project), ("Unreal editor", editor)):
        if os.path.isfile(path):
            print("OK   %-20s %s" % (label, path))
        else:
            problems.append("missing %s: %s" % (label, path))

    if os.path.isfile(project):
        with open(project, "r", encoding="utf-8-sig") as stream:
            uproject = json.load(stream)
        enabled = {item["Name"] for item in uproject.get("Plugins", []) if item.get("Enabled")}
        for plugin in ("PythonScriptPlugin", "EditorScriptingUtilities", "CarlaTools"):
            if plugin in enabled:
                print("OK   plugin               " + plugin)
            else:
                problems.append("plugin is not enabled: " + plugin)

    with open(args.catalog, "r", encoding="utf-8") as stream:
        catalog = json.load(stream)
    assets = []
    for values in catalog["buildings"].values():
        assets.extend(values)
    for values in catalog["trees"].values():
        assets.extend(values)
    assets.extend(catalog["traffic_signs"].values())
    for values in catalog.get("nuscenes_static_objects", {}).values():
        assets.extend(values)
    assets.append(catalog["traffic_light"])
    missing_assets = [asset for asset in assets if not os.path.isfile(asset_file(content, asset))]
    print("OK   catalog assets       %d/%d present" %
          (len(assets) - len(missing_assets), len(assets)))
    problems.extend("missing catalog asset: " + asset for asset in missing_assets)

    if problems:
        for problem in problems:
            print("FAIL " + problem)
        return 1
    print("All read-only prerequisite checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
