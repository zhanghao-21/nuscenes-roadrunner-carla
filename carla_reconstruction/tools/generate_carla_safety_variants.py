#!/usr/bin/env python3
"""Generate separate ego or surrounding Traffic Manager behavior experiments."""

import argparse
import os
import random
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.safety_variants import (  # noqa: E402
    CARLA_BEHAVIOR_LIMITS, CARLA_EGO_SCOPE, CARLA_PIPELINE, CARLA_SCOPE,
    CARLA_VARIANT_SCHEMA_VERSION, latin_hypercube, load_json, parse_range,
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
        retained_vehicle_ids, behavior, target, moving_ids, baseline_behavior):
    document = {
        "schema_version": CARLA_VARIANT_SCHEMA_VERSION,
        "pipeline": CARLA_PIPELINE,
        "variant_id": variant_id,
        "kind": kind,
        "manifest": manifest_path,
        "simulation_seed": int(simulation_seed),
        "ego_mode": "tm",
        "selection": {
            "scope": CARLA_EGO_SCOPE if target == "ego" else CARLA_SCOPE,
            "minimum_track_distance_m": float(minimum_track_distance),
            "minimum_two_point_speed_mps": float(minimum_two_point_speed),
            "eligible_track_ids": list(eligible_ids),
            "moving_surrounding_track_ids": list(moving_ids),
            "retained_vehicle_track_ids": list(retained_vehicle_ids),
        },
        "behavior": behavior,
        "baseline_behavior": baseline_behavior,
        "scenario": {
            "name": variant_id,
            "pipeline": CARLA_PIPELINE,
            "generator": "latin_hypercube" if kind == "sample" else "baseline",
            "generation_seed": int(generation_seed),
            "perturbation_target": target,
            "parameters": behavior,
        },
    }
    return validate_carla_variant(document)


def _validate_output_scope(output_dir, baseline):
    """Do not overwrite another experiment family through an explicit --output."""
    for name in ("index.json", "baseline.json"):
        path = os.path.join(output_dir, name)
        if not os.path.isfile(path):
            continue
        existing = load_json(path)
        if (existing.get("pipeline") != CARLA_PIPELINE or
                existing.get("schema_version") != CARLA_VARIANT_SCHEMA_VERSION or
                existing.get("selection", {}).get("scope") !=
                baseline["selection"]["scope"] or
                os.path.normcase(os.path.abspath(existing.get("manifest", ""))) !=
                os.path.normcase(baseline["manifest"])):
            raise ValueError(
                "output contains a different scene, target, or legacy experiment; "
                "choose a separate --output directory: %s" % output_dir)


def run(args):
    if (isinstance(args.count, bool) or not isinstance(args.count, int) or
            args.count <= 0):
        raise ValueError("--count must be positive")
    target = getattr(args, "target", "surrounding")
    if target not in ("ego", "surrounding"):
        raise ValueError("--target must be ego or surrounding")
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
    if target == "surrounding" and not moving_ids:
        raise ValueError("manifest contains no moving surrounding vehicles")
    if target == "ego" and len(bundle.ego.points) < 2:
        raise ValueError("ego perturbations require an ego path with at least two points")
    eligible_ids = ["ego"] if target == "ego" else moving_ids

    output_dir = os.path.abspath(args.output or os.path.join(
        os.path.dirname(manifest_path), "carla_safety_variants", target))
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
        manifest_path, "carla_tm_%s_baseline" % target, "baseline", args.seed,
        args.simulation_seed, minimum_track_distance,
        minimum_two_point_speed, eligible_ids, retained_vehicle_ids,
        baseline_behavior, target, moving_ids, baseline_behavior)
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
        variant_id = "carla_tm_%s_%03d" % (target, number)
        document = _variant_document(
            manifest_path, variant_id, "sample", args.seed,
            args.simulation_seed, minimum_track_distance,
            minimum_two_point_speed, eligible_ids, retained_vehicle_ids,
            behavior, target, moving_ids, baseline_behavior)
        prepared_variants.append((
            number, variant_id, behavior, document))
    _validate_output_scope(output_dir, baseline)
    os.makedirs(output_dir, exist_ok=True)
    removed = prune_stale_numbered_variants(output_dir, args.count)
    if removed:
        print("Removed stale generated CARLA variants:", ", ".join(removed))
    baseline_path = os.path.abspath(os.path.join(output_dir, "baseline.json"))
    write_json(baseline_path, baseline)

    index = {
        "schema_version": CARLA_VARIANT_SCHEMA_VERSION,
        "pipeline": CARLA_PIPELINE,
        "manifest": manifest_path,
        "scene": manifest.get("scene"),
        "generation_seed": int(args.seed),
        "simulation_seed": int(args.simulation_seed),
        "ego_mode": "tm",
        "perturbation_target": target,
        "baseline_behavior": baseline["baseline_behavior"],
        "selection": baseline["selection"],
        "population": {
            "recorded_vehicle_count": len(bundle.vehicle_tracks),
            "moving_surrounding_count": len(moving_ids),
            "moving_variant_target_count": len(eligible_ids),
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
    print("Generated matched CARLA %s baseline and %d variants in %s" %
          (target, args.count, output_dir))
    print("Perturbation targets:", len(eligible_ids), "(" + target + ")")
    print("Moving CARLA Traffic Manager vehicles per experiment:",
          len(moving_ids))
    print("Fixed CARLA vehicles retained:",
          len(static_ids) + len(single_observation_ids))
    print("Index:", index_path)
    return index


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument(
        "--target", choices=("ego", "surrounding"), default="surrounding",
        help="perturb only the ego or all moving surrounding vehicles; "
             "both groups use Traffic Manager in every experiment")
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
    parser.add_argument("--output", help="exact output folder; default: "
                        "<scene>/carla_safety_variants/<target>")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.count <= 0:
        parser.error("--count must be positive")
    run(args)


if __name__ == "__main__":
    main()
