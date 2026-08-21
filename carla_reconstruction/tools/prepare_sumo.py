#!/usr/bin/env python3
"""Build a SUMO network and nuScenes-seeded routes for hybrid simulation."""

import argparse
import json
import os
import shutil
import subprocess
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop import SCHEMA_VERSION  # noqa: E402
from carla_reconstruction.closed_loop.sumo_routes import write_sumo_scenario  # noqa: E402
from carla_reconstruction.closed_loop.tracks import load_manifest_tracks  # noqa: E402


def _default_output(manifest):
    return os.path.join(REPO_ROOT, "carla_reconstruction", "generated",
                        "closed_loop", manifest["scene"], "sumo")


def _convert_network(args, xodr_path, network_path):
    script = os.path.join(args.carla_root, "Co-Simulation", "Sumo", "util",
                          "netconvert_carla.py")
    if not os.path.isfile(script):
        raise FileNotFoundError("CARLA SUMO converter was not found: " + script)
    environment = os.environ.copy()
    if args.sumo_home:
        environment["SUMO_HOME"] = os.path.abspath(args.sumo_home)
    # CARLA 0.9.15's Windows wrapper invokes netconvert with ``shell=True``;
    # apostrophes/spaces in the OneDrive source path are consequently split.
    # Stage a read-only copy beside the output and give tempfile a writable root.
    staged_xodr = os.path.join(os.path.dirname(network_path), "input.xodr")
    if os.path.normcase(xodr_path) != os.path.normcase(staged_xodr):
        shutil.copy2(xodr_path, staged_xodr)
    temp_root = os.path.join(os.path.dirname(network_path), "tmp")
    os.makedirs(temp_root, exist_ok=True)
    environment["TEMP"] = temp_root
    environment["TMP"] = temp_root
    command = [sys.executable, script, staged_xodr, "--output", network_path]
    if args.guess_tls:
        command.append("--guess-tls")
    print("Running:", subprocess.list2cmdline(command))
    subprocess.run(command, check=True, env=environment, cwd=os.path.dirname(script))


def run(args):
    manifest_path = os.path.abspath(args.manifest)
    manifest, bundle = load_manifest_tracks(manifest_path)
    output_dir = os.path.abspath(args.output or _default_output(manifest))
    os.makedirs(output_dir, exist_ok=True)

    xodr_path = os.path.abspath(args.xodr or manifest["source"]["xodr"])
    if not os.path.isfile(xodr_path):
        raise FileNotFoundError("OpenDRIVE input was not found: " + xodr_path)
    network_path = os.path.join(output_dir, "network.net.xml")
    if args.net_file:
        source_network = os.path.abspath(args.net_file)
        if not os.path.isfile(source_network):
            raise FileNotFoundError("SUMO network was not found: " + source_network)
        if os.path.normcase(source_network) != os.path.normcase(network_path):
            shutil.copy2(source_network, network_path)
    elif not (args.keep_existing_net and os.path.isfile(network_path)):
        _convert_network(args, xodr_path, network_path)

    if not os.path.isfile(network_path):
        raise RuntimeError("SUMO network generation did not create " + network_path)

    excluded = set(args.critical_actor)
    config_path, report_path, report = write_sumo_scenario(
        network_path=network_path,
        tracks=bundle.actors,
        output_dir=output_dir,
        scene_name=manifest["scene"],
        maximum_snap_distance=args.maximum_snap_distance,
        excluded_actor_ids=excluded,
        step_length=args.step_length,
        end_time=args.duration)

    hybrid = {
        "schema_version": SCHEMA_VERSION,
        "manifest": manifest_path,
        "map": manifest["map"]["runtime_name"],
        "step_length": args.step_length,
        "duration": args.duration if args.duration is not None else bundle.duration,
        "seed": args.seed,
        "sumo": {
            "config": os.path.abspath(config_path),
            "network": os.path.abspath(network_path),
            "route_report": os.path.abspath(report_path),
            "tls_manager": args.tls_manager,
        },
        "ego": {
            "track_id": "ego",
            "authority": args.ego_authority,
        },
        "critical_actors": [
            {
                "track_id": actor_id,
                "authority": "carla_reference",
                "events": [],
            }
            for actor_id in sorted(excluded)
        ],
        "background": {
            "authority": "sumo",
            "track_ids": [item["id"] for item in report["included"]],
        },
        "traffic_manager": {
            "port": 8000,
            "leading_distance": 2.5,
            "auto_lane_change": False,
        },
    }
    hybrid_path = os.path.join(output_dir, "hybrid_config.json")
    with open(hybrid_path, "w", encoding="utf-8") as stream:
        json.dump(hybrid, stream, indent=2, sort_keys=False)
        stream.write("\n")

    print("SUMO network:", network_path)
    print("SUMO configuration:", config_path)
    print("Hybrid configuration:", hybrid_path)
    print("Routes included: %d; skipped: %d" % (
        len(report["included"]), len(report["skipped"])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--xodr", help="override manifest OpenDRIVE source")
    parser.add_argument("--output")
    parser.add_argument("--carla-root", default=r"C:\carla")
    parser.add_argument("--sumo-home", default=os.environ.get("SUMO_HOME"))
    parser.add_argument("--net-file", help="use an existing SUMO .net.xml")
    parser.add_argument("--keep-existing-net", action="store_true")
    parser.add_argument("--guess-tls", action="store_true")
    parser.add_argument("--maximum-snap-distance", type=float, default=8.0)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--seed", type=int, default=103)
    parser.add_argument("--tls-manager", choices=("none", "carla", "sumo"), default="none")
    parser.add_argument("--ego-authority",
                        choices=("carla_tm", "carla_reference", "carla_replay", "carla_external"),
                        default="carla_reference")
    parser.add_argument("--critical-actor", action="append", default=[],
                        help="nuScenes actor id controlled by CARLA instead of SUMO; repeatable")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
