#!/usr/bin/env python3
"""Generate all-CARLA Traffic Manager safety variants for one scene."""

import argparse
import os
import random
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.safety_variants import (  # noqa: E402
    CARLA_BEHAVIOR_LIMITS, CARLA_PIPELINE, CARLA_SCOPE,
    VARIANT_SCHEMA_VERSION, latin_hypercube, parse_range,
    prune_stale_numbered_variants, validate_carla_variant, validate_range,
    write_json)
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    DEFAULT_MINIMUM_TRACK_DISTANCE_M, DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
    classify_vehicle_motion, load_manifest_tracks,
    validated_minimum_track_distance, validated_minimum_two_point_speed)


DEFAULT_RANGES = {
    "desired_speed_scale": (0.75, 1.35),
    "leading_distance_m": (0.50, 4.00),
    "random_left_lane_change_percentage": (0.0, 30.0),
    "random_right_lane_change_percentage": (0.0, 30.0),
    "keep_right_rule_percentage": (0.0, 30.0),
}


def _round(value):
    return round(float(value), 6)


def _variant_document(
        manifest_path, variant_id, kind, generation_seed, simulation_seed,
        minimum_track_distance, minimum_two_point_speed, eligible_ids,
        retained_vehicle_ids, behavior):
    document = {
        "schema_version": VARIANT_SCHEMA_VERSION,
        "pipeline": CARLA_PIPELINE,
        "variant_id": variant_id,
        "kind": kind,
        "manifest": manifest_path,
        "simulation_seed": int(simulation_seed),
        "selection": {
            "scope": CARLA_SCOPE,
            "minimum_track_distance_m": float(minimum_track_distance),
            "minimum_two_point_speed_mps": float(minimum_two_point_speed),
            "eligible_track_ids": list(eligible_ids),
            "retained_vehicle_track_ids": list(retained_vehicle_ids),
        },
        "behavior": behavior,
        "scenario": {
            "name": variant_id,
            "pipeline": CARLA_PIPELINE,
            "generator": "latin_hypercube" if kind == "sample" else "baseline",
            "generation_seed": int(generation_seed),
            "parameters": behavior,
        },
    }
    return validate_carla_variant(document)


def run(args):
    if (isinstance(args.count, bool) or not isinstance(args.count, int) or
            args.count <= 0):
        raise ValueError("--count must be positive")
    manifest_path = os.path.abspath(args.manifest)
    manifest, bundle = load_manifest_tracks(manifest_path)
    minimum_track_distance = validated_minimum_track_distance(
        args.minimum_track_distance)
    minimum_two_point_speed = validated_minimum_two_point_speed(
        args.minimum_two_point_speed)
    moving_ids = []
    static_ids = []
    single_observation_ids = []
    for track in bundle.vehicle_tracks.values():
        motion = classify_vehicle_motion(
            track, minimum_track_distance, minimum_two_point_speed)
        if motion == "moving":
            moving_ids.append(track.actor_id)
        elif motion == "static":
            static_ids.append(track.actor_id)
        else:
            # With only one observation there is no defensible trajectory to
            # control.  The CARLA baseline still renders it as a fixed actor.
            single_observation_ids.append(track.actor_id)
    moving_ids.sort()
    static_ids.sort()
    single_observation_ids.sort()
    retained_vehicle_ids = sorted(bundle.vehicle_tracks)
    if not moving_ids:
        raise ValueError("manifest contains no moving surrounding vehicles")

    output_dir = os.path.abspath(args.output or os.path.join(
        os.path.dirname(manifest_path), "carla_safety_variants"))
    os.makedirs(output_dir, exist_ok=True)
    bounds = [
        ("desired_speed_scale", validate_range(
            "desired speed scale", args.desired_speed_scale,
            CARLA_BEHAVIOR_LIMITS["desired_speed_scale"])),
        ("leading_distance_m", validate_range(
            "leading distance", args.leading_distance,
            CARLA_BEHAVIOR_LIMITS["leading_distance_m"])),
        ("random_left_lane_change_percentage", validate_range(
            "left lane-change percentage", args.random_left_lane_change,
            CARLA_BEHAVIOR_LIMITS[
                "random_left_lane_change_percentage"])),
        ("random_right_lane_change_percentage", validate_range(
            "right lane-change percentage", args.random_right_lane_change,
            CARLA_BEHAVIOR_LIMITS[
                "random_right_lane_change_percentage"])),
        ("keep_right_rule_percentage", validate_range(
            "keep-right percentage", args.keep_right,
            CARLA_BEHAVIOR_LIMITS["keep_right_rule_percentage"])),
    ]
    samples = latin_hypercube(
        args.count, bounds, random.Random(args.seed))

    baseline_behavior = {
        "desired_speed_scale": 1.0,
        "leading_distance_m": float(args.baseline_leading_distance),
        "auto_lane_change": False,
        "random_left_lane_change_percentage": 0.0,
        "random_right_lane_change_percentage": 0.0,
        "keep_right_rule_percentage": 0.0,
    }
    baseline = _variant_document(
        manifest_path, "carla_tm_baseline", "baseline", args.seed,
        args.simulation_seed, minimum_track_distance,
        minimum_two_point_speed, moving_ids, retained_vehicle_ids,
        baseline_behavior)
    prepared_variants = []
    for number, sample in enumerate(samples):
        behavior = {key: _round(value) for key, value in sample.items()}
        behavior["auto_lane_change"] = True
        # Preserve a stable, reader-friendly order in generated JSON.
        behavior = {
            "desired_speed_scale": behavior["desired_speed_scale"],
            "leading_distance_m": behavior["leading_distance_m"],
            "auto_lane_change": behavior["auto_lane_change"],
            "random_left_lane_change_percentage": behavior[
                "random_left_lane_change_percentage"],
            "random_right_lane_change_percentage": behavior[
                "random_right_lane_change_percentage"],
            "keep_right_rule_percentage": behavior[
                "keep_right_rule_percentage"],
        }
        variant_id = "carla_tm_%03d" % number
        document = _variant_document(
            manifest_path, variant_id, "sample", args.seed,
            args.simulation_seed, minimum_track_distance,
            minimum_two_point_speed, moving_ids, retained_vehicle_ids,
            behavior)
        prepared_variants.append((
            number, variant_id, behavior, document))
    removed = prune_stale_numbered_variants(output_dir, args.count)
    if removed:
        print("Removed stale generated CARLA variants:", ", ".join(removed))
    baseline_path = os.path.abspath(os.path.join(output_dir, "baseline.json"))
    write_json(baseline_path, baseline)

    index = {
        "schema_version": VARIANT_SCHEMA_VERSION,
        "pipeline": CARLA_PIPELINE,
        "manifest": manifest_path,
        "scene": manifest.get("scene"),
        "generation_seed": int(args.seed),
        "simulation_seed": int(args.simulation_seed),
        "selection": baseline["selection"],
        "population": {
            "recorded_vehicle_count": len(bundle.vehicle_tracks),
            "moving_variant_target_count": len(moving_ids),
            "recorded_static_count": len(static_ids),
            "single_observation_static_count": len(single_observation_ids),
            "static_track_ids": static_ids,
            "single_observation_static_track_ids": single_observation_ids,
        },
        "baseline_config": baseline_path,
        "variants": [],
    }
    for number, variant_id, behavior, document in prepared_variants:
        path = os.path.abspath(os.path.join(
            output_dir, "variant_%03d.json" % number))
        write_json(path, document)
        index["variants"].append({
            "variant_id": variant_id,
            "config": path,
            "behavior": behavior,
        })
    index_path = os.path.abspath(os.path.join(output_dir, "index.json"))
    write_json(index_path, index)
    print("Generated matched CARLA baseline and %d variants in %s" %
          (args.count, output_dir))
    print("Moving CARLA Traffic Manager vehicles per experiment:",
          len(moving_ids))
    print("Fixed CARLA vehicles retained:",
          len(static_ids) + len(single_observation_ids))
    print("Index:", index_path)
    return index


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed", type=int, default=103,
                        help="Latin-hypercube sampling seed")
    parser.add_argument("--simulation-seed", type=int, default=103,
                        help="fixed CARLA Traffic Manager seed for every experiment")
    parser.add_argument(
        "--minimum-track-distance", type=float,
        default=DEFAULT_MINIMUM_TRACK_DISTANCE_M)
    parser.add_argument(
        "--minimum-two-point-speed", type=float,
        default=DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS)
    parser.add_argument("--desired-speed-scale", type=parse_range,
                        default=DEFAULT_RANGES["desired_speed_scale"],
                        metavar="MIN,MAX")
    parser.add_argument("--leading-distance", type=parse_range,
                        default=DEFAULT_RANGES["leading_distance_m"],
                        metavar="MIN,MAX")
    parser.add_argument("--random-left-lane-change", type=parse_range,
                        default=DEFAULT_RANGES[
                            "random_left_lane_change_percentage"],
                        metavar="MIN,MAX")
    parser.add_argument("--random-right-lane-change", type=parse_range,
                        default=DEFAULT_RANGES[
                            "random_right_lane_change_percentage"],
                        metavar="MIN,MAX")
    parser.add_argument("--keep-right", type=parse_range,
                        default=DEFAULT_RANGES[
                            "keep_right_rule_percentage"],
                        metavar="MIN,MAX")
    parser.add_argument("--baseline-leading-distance", type=float, default=2.5)
    parser.add_argument("--output")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.count <= 0:
        parser.error("--count must be positive")
    run(args)


if __name__ == "__main__":
    main()
