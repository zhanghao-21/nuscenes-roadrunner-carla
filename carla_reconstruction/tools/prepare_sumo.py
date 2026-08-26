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
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    DEFAULT_MINIMUM_TRACK_DISTANCE_M, DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
    load_manifest_tracks, validated_minimum_track_distance,
    validated_minimum_two_point_speed)


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


def _print_network_terminal_warning(report):
    rows = sorted(
        (
            item for item in report.get("included", [])
            if (isinstance(item.get("continuation"), dict) and
                item["continuation"].get("status") == "network_terminal")
        ),
        key=lambda item: str(item.get("id", "")))
    if not rows:
        return
    print(
        "WARNING: %d SUMO mover route(s) reach a source network terminal "
        "with no outgoing connection. They will drive to that road end and "
        "then leave SUMO:" % len(rows))
    for item in rows:
        print("  - actor %s (SUMO %s), terminal edge %s" % (
            item.get("id", "unknown"), item.get("sumo_id", "unknown"),
            item["continuation"].get("terminal_edge", "unknown")))


def run(args):
    if (getattr(args, "enable_prediction_risk", False) and
            abs(float(args.step_length) - 0.05) > 1.0e-9):
        raise ValueError(
            "the bundled HGT checkpoint expects 0.05 s samples; "
            "--enable-prediction-risk currently requires --step-length 0.05")
    manifest_path = os.path.abspath(args.manifest)
    minimum_track_distance = validated_minimum_track_distance(
        getattr(args, "minimum_track_distance",
                DEFAULT_MINIMUM_TRACK_DISTANCE_M))
    minimum_two_point_speed = validated_minimum_two_point_speed(
        getattr(args, "minimum_two_point_speed",
                DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS))
    static_spawn_mode = getattr(args, "static_spawn_mode", "startup")
    if static_spawn_mode not in ("startup", "recorded"):
        raise ValueError("--static-spawn-mode must be startup or recorded")
    bridge_warmup_steps = int(getattr(args, "bridge_warmup_steps", 1))
    if bridge_warmup_steps < 0:
        raise ValueError("--bridge-warmup-steps must be non-negative")
    if static_spawn_mode == "startup" and bridge_warmup_steps < 1:
        raise ValueError(
            "--bridge-warmup-steps must be at least 1 when static vehicles "
            "are staged at startup")
    if static_spawn_mode == "recorded":
        bridge_warmup_steps = 0
    moving_start_delay = getattr(args, "moving_start_delay", None)
    minimum_start_delay = (
        (bridge_warmup_steps + 2) * float(args.step_length)
        if static_spawn_mode == "startup" else 0.0)
    if moving_start_delay is None:
        moving_start_delay = minimum_start_delay
    moving_start_delay = round(float(moving_start_delay), 12)
    if moving_start_delay + 1.0e-9 < minimum_start_delay:
        raise ValueError(
            "--moving-start-delay must be at least %.3f s so CARLA actors "
            "are mirrored before SUMO traffic departs" % minimum_start_delay)
    maximum_departure_gap = getattr(
        args, "maximum_moving_departure_gap", None)
    eager_insert = not bool(getattr(args, "no_eager_insert", False))
    route_continuation_seed = getattr(
        args, "route_continuation_seed", None)
    if route_continuation_seed is None:
        route_continuation_seed = args.seed
    route_continuation_search_depth = getattr(
        args, "route_continuation_search_depth",
        getattr(args, "maximum_route_continuation_edges", 64))
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
        end_time=args.duration,
        minimum_track_distance=minimum_track_distance,
        minimum_two_point_speed=minimum_two_point_speed,
        moving_start_delay=moving_start_delay,
        maximum_departure_gap=maximum_departure_gap,
        eager_insert=eager_insert,
        moving_route_continuation=getattr(
            args, "moving_route_continuation", "terminal"),
        route_continuation_seed=route_continuation_seed,
        maximum_route_continuation_edges=route_continuation_search_depth,
        allow_continuation_uturns=getattr(
            args, "allow_continuation_uturns", False),
        moving_initial_pose=getattr(
            args, "moving_initial_pose", "recorded"),
        moving_speed_policy=getattr(
            args, "moving_speed_policy", "recorded_profile"),
        minimum_moving_speed=getattr(
            args, "minimum_moving_speed", 0.1),
        terminal_stop_policy=getattr(
            args, "terminal_stop_policy", "release_to_sumo"),
        terminal_stop_maximum_extent=getattr(
            args, "terminal_stop_maximum_extent", 1.0),
        terminal_stop_minimum_duration=getattr(
            args, "terminal_stop_minimum_duration", 2.0),
        sumo_seed=args.seed)

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
        "static_actors": [
            {
                "track_id": item["id"],
                "authority": "carla_static",
                "spawn_phase": (
                    "prestage" if static_spawn_mode == "startup"
                    else "recorded"),
                "recorded_start_time_s": item["recorded_start_time"],
            }
            for item in report["carla_static"]
        ],
        "spawn_schedule": {
            "static_spawn_mode": static_spawn_mode,
            "bridge_warmup_steps": bridge_warmup_steps,
            "moving_start_delay_s": moving_start_delay,
            "maximum_moving_departure_gap_s": (
                None if maximum_departure_gap is None
                else float(maximum_departure_gap)),
            "sumo_eager_insert": eager_insert,
        },
        "background": {
            "authority": "sumo",
            "track_ids": [item["id"] for item in report["included"]],
            "minimum_track_distance_m": minimum_track_distance,
            "minimum_two_point_speed_mps": minimum_two_point_speed,
            "route_continuation": report["moving_route_continuation"],
            "moving_initialization": report["moving_initialization"],
            "moving_speed": report["moving_speed"],
        },
        "traffic_manager": {
            "port": 8000,
            "leading_distance": 2.5,
            "auto_lane_change": False,
        },
    }
    if getattr(args, "enable_prediction_risk", False):
        model_root = os.path.join(
            REPO_ROOT, "carla_reconstruction", "HGT_model")
        hybrid["prediction_risk"] = {
            "enabled": True,
            "pace_realtime": True,
            "update_interval_s": 0.1,
            "sample_step_s": 0.05,
            "neighbor_radius_m": 50.0,
            "max_neighbors": 3,
            "mode_count": 5,
            "overlap_half_x_m": 5.0,
            "overlap_half_y_m": 2.0,
            "model": {
                "config": os.path.abspath(os.path.join(
                    model_root, "configs", "mart_carla.yaml")),
                "checkpoint": os.path.abspath(os.path.join(
                    model_root, "checkpoints", "mart_carla",
                    "carla_ckpt_best.pth")),
                "device": "auto",
                "seed": 1,
                "input_scale": 2.5,
                "output_scale": 2.5,
                "output_frame": "actor_heading",
                "heading_history_samples": 5,
                "minimum_heading_displacement_m": 0.05,
            },
            "dashboard": {
                "enabled": True,
                "column_width_px": 450,
                "history_length": 200,
                "startup_timeout_s": 10.0,
            },
            "drawing": {
                "enabled": False,
                "mode_count": 3,
                "life_time_s": 0.2,
            },
        }
    hybrid_path = os.path.join(output_dir, "hybrid_config.json")
    with open(hybrid_path, "w", encoding="utf-8") as stream:
        json.dump(hybrid, stream, indent=2, sort_keys=False)
        stream.write("\n")

    print("SUMO network:", network_path)
    print("SUMO configuration:", config_path)
    print("Hybrid configuration:", hybrid_path)
    print("SUMO routes: %d; CARLA static: %d; skipped: %d" % (
        len(report["included"]), len(report["carla_static"]),
        len(report["skipped"])))
    _print_network_terminal_warning(report)
    included_departures = [item["depart"] for item in report["included"]]
    if included_departures:
        print("Moving departures: %.3f-%.3f s; maximum source gap: %s" % (
            min(included_departures), max(included_departures),
            ("recorded" if maximum_departure_gap is None
             else "%.3f s" % float(maximum_departure_gap))))


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
    parser.add_argument(
        "--minimum-track-distance", type=float,
        default=DEFAULT_MINIMUM_TRACK_DISTANCE_M,
        help="vehicle tracks with less spatial extent remain static in CARLA")
    parser.add_argument(
        "--minimum-two-point-speed", type=float,
        default=DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
        help="speed that identifies a moving vehicle from exactly two observations")
    parser.add_argument(
        "--static-spawn-mode", choices=("startup", "recorded"),
        default="startup",
        help="prestage all CARLA-static vehicles or use their recorded appearance times")
    parser.add_argument(
        "--bridge-warmup-steps", type=int, default=1,
        help="bridge-only ticks used to mirror prestaged static actors into SUMO (minimum 1)")
    parser.add_argument(
        "--moving-start-delay", type=float,
        help="internal SUMO delay before the first moving departure; derived by default")
    parser.add_argument(
        "--maximum-moving-departure-gap", type=float,
        help="cap long gaps between moving departure groups; omitted preserves recorded gaps")
    parser.add_argument(
        "--no-eager-insert", action="store_true",
        help="stop trying later same-edge SUMO insertions after the first blocked vehicle")
    parser.add_argument(
        "--moving-route-continuation", choices=("none", "terminal"),
        default="terminal",
        help="extend each recorded route through seeded valid turns to a network boundary")
    parser.add_argument(
        "--route-continuation-seed", type=int,
        help="seed for inferred route turns; defaults to --seed")
    parser.add_argument(
        "--route-continuation-search-depth",
        "--maximum-route-continuation-edges",
        dest="route_continuation_search_depth", type=int, default=64,
        help="terminal search depth and minimum cyclic tail; duration coverage may add edges")
    parser.add_argument(
        "--allow-continuation-uturns", action="store_true",
        help="allow explicit SUMO turnaround connections in inferred route tails")
    parser.add_argument(
        "--moving-initial-pose", choices=("recorded", "route"),
        default="recorded",
        help="relocate each inserted SUMO actor to its first matched recorded pose")
    parser.add_argument(
        "--moving-speed-policy",
        choices=("recorded_profile", "recorded_mean", "unbounded"),
        default="recorded_profile",
        help="use a safe recorded speed target, a mean-speed cap, or generic type limits")
    parser.add_argument(
        "--minimum-moving-speed", type=float, default=0.1,
        help="positive lower bound for data-seeded SUMO maximum speed")
    parser.add_argument(
        "--terminal-stop-policy",
        choices=("release_to_sumo", "preserve_recorded"),
        default="release_to_sumo",
        help="release a moving actor's terminal stationary tail to SUMO or replay it")
    parser.add_argument(
        "--terminal-stop-maximum-extent", type=float, default=1.0,
        help="maximum remaining recorded extent classified as a terminal stop")
    parser.add_argument(
        "--terminal-stop-minimum-duration", type=float, default=2.0,
        help="minimum stationary-tail duration before autonomous SUMO release")
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--seed", type=int, default=103)
    parser.add_argument("--tls-manager", choices=("none", "carla", "sumo"), default="none")
    parser.add_argument("--ego-authority",
                        choices=("carla_tm", "carla_reference", "carla_replay", "carla_external"),
                        default="carla_reference")
    parser.add_argument("--critical-actor", action="append", default=[],
                        help="nuScenes actor id controlled by CARLA instead of SUMO; repeatable")
    parser.add_argument(
        "--enable-prediction-risk", action="store_true",
        help="enable the optional HGT prediction-risk dashboard in hybrid_config.json")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
