#!/usr/bin/env python3
"""Generate separate ego or surrounding Traffic Manager behavior experiments."""

import argparse
import hashlib
import os
import random
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.safety_variants import (  # noqa: E402
    CARLA_AGGRESSION_LIMITS, CARLA_BEHAVIOR_LIMITS, CARLA_EGO_SCOPE, CARLA_PIPELINE, CARLA_SCOPE,
    CARLA_VARIANT_SCHEMA_VERSION, latin_hypercube, load_json, parse_range,
    prune_stale_numbered_variants, validate_carla_variant, validate_range,
    write_json)
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    DEFAULT_MINIMUM_TRACK_DISTANCE_M, DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
    classify_vehicle_motion, load_manifest_tracks,
    validated_minimum_track_distance, validated_minimum_two_point_speed)


MILD_RANGES = {
    "desired_speed_scale": (0.75, 1.35),
    "leading_distance_m": (0.50, 4.00),
    "random_left_lane_change_percentage": (0.0, 30.0),
    "random_right_lane_change_percentage": (0.0, 30.0),
    "keep_right_rule_percentage": (0.0, 30.0),
    "target_speed_floor_kmh": (0.0, 0.0),
    "ignore_vehicles_percentage": (0.0, 0.0),
    "ignore_lights_percentage": (0.0, 0.0),
    "ignore_signs_percentage": (0.0, 0.0),
}
PROFILE_RANGES = {
    "mild": MILD_RANGES,
    "aggressive": {
        "desired_speed_scale": (1.4, 2.6),
        "leading_distance_m": (0.2, 0.8),
        "random_left_lane_change_percentage": (40.0, 80.0),
        "random_right_lane_change_percentage": (40.0, 80.0),
        "keep_right_rule_percentage": (0.0, 0.0),
        "target_speed_floor_kmh": (30.0, 45.0),
        "ignore_vehicles_percentage": (70.0, 100.0),
        "ignore_lights_percentage": (80.0, 100.0),
        "ignore_signs_percentage": (80.0, 100.0),
    },
    "stress": {
        "desired_speed_scale": (2.0, 3.5),
        "leading_distance_m": (0.0, 0.2),
        "random_left_lane_change_percentage": (70.0, 100.0),
        "random_right_lane_change_percentage": (70.0, 100.0),
        "keep_right_rule_percentage": (0.0, 0.0),
        "target_speed_floor_kmh": (45.0, 65.0),
        "ignore_vehicles_percentage": (100.0, 100.0),
        "ignore_lights_percentage": (100.0, 100.0),
        "ignore_signs_percentage": (100.0, 100.0),
    },
}
PROFILE_SPEED_CEILINGS = {"mild": 0.0, "aggressive": 80.0, "stress": 100.0}
DEFAULT_RANGES = PROFILE_RANGES["aggressive"]
RANGE_OPTIONS = {
    "desired_speed_scale": "desired_speed_scale",
    "leading_distance_m": "leading_distance",
    "random_left_lane_change_percentage": "random_left_lane_change",
    "random_right_lane_change_percentage": "random_right_lane_change",
    "keep_right_rule_percentage": "keep_right",
    "target_speed_floor_kmh": "target_speed_floor",
    "ignore_vehicles_percentage": "ignore_vehicles",
    "ignore_lights_percentage": "ignore_lights",
    "ignore_signs_percentage": "ignore_signs",
}


def resolve_sampling(args):
    profile = getattr(args, "profile", "aggressive")
    if profile not in PROFILE_RANGES:
        raise ValueError("unknown CARLA aggression profile")
    limits = dict(CARLA_BEHAVIOR_LIMITS, **CARLA_AGGRESSION_LIMITS)
    bounds = []
    for key, option in RANGE_OPTIONS.items():
        override = getattr(args, option, None)
        value = PROFILE_RANGES[profile][key] if override is None else override
        bounds.append((key, validate_range(key, value, limits[key])))
    ceiling = getattr(args, "maximum_speed_kmh", None)
    ceiling = PROFILE_SPEED_CEILINGS[profile] if ceiling is None else ceiling
    ceiling = validate_range("maximum speed", (ceiling, ceiling),
                             CARLA_AGGRESSION_LIMITS["target_speed_ceiling_kmh"])[0]
    if ceiling and dict(bounds)["target_speed_floor_kmh"][1] > ceiling:
        raise ValueError("target speed floor range cannot exceed --maximum-speed-kmh")
    return profile, bounds, ceiling


def sampled_actor_behaviors(count, bounds, seed, eligible_ids, profile, ceiling):
    """Separate reproducible speed/compliance variation for each selected driver.

    Mild keeps the previous homogeneous sampling. Aggressive/stress use stable
    per-track seeds so targets do not all accelerate/change lanes identically.
    """
    columns = {}
    for actor_id in eligible_ids:
        actor_seed = seed if profile == "mild" else int.from_bytes(
            hashlib.sha256((str(seed) + ":" + actor_id).encode("utf-8")).digest()[:8], "big")
        columns[actor_id] = latin_hypercube(count, bounds, random.Random(actor_seed))
    return [{actor_id: dict({key: _round(value) for key, value in columns[actor_id][number].items()},
                           auto_lane_change=True, target_speed_ceiling_kmh=ceiling)
             for actor_id in eligible_ids} for number in range(count)]


def _round(value):
    return round(float(value), 6)


def _variant_document(
        manifest_path, variant_id, kind, generation_seed, simulation_seed,
        minimum_track_distance, minimum_two_point_speed, eligible_ids,
        retained_vehicle_ids, behavior, target, moving_ids, baseline_behavior,
        profile, actor_behaviors):
    document = {
        "schema_version": CARLA_VARIANT_SCHEMA_VERSION,
        "pipeline": CARLA_PIPELINE,
        "variant_id": variant_id,
        "kind": kind,
        "aggression_profile": profile,
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
        "actor_behaviors": actor_behaviors,
        "baseline_behavior": baseline_behavior,
        "scenario": {
            "name": variant_id,
            "pipeline": CARLA_PIPELINE,
            "generator": "latin_hypercube" if kind == "sample" else "baseline",
            "generation_seed": int(generation_seed),
            "perturbation_target": target,
            "parameters": behavior,
            "parameters_describe": "first selected actor; actor_behaviors contains every target",
            "aggression_profile": profile,
            "sampling": "shared" if profile == "mild" else "independent_per_track",
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
                existing.get("aggression_profile") != baseline.get("aggression_profile") or
                existing.get("selection", {}).get("scope") !=
                baseline["selection"]["scope"] or
                os.path.normcase(os.path.abspath(existing.get("manifest", ""))) !=
                os.path.normcase(baseline["manifest"])):
            raise ValueError(
                "output contains a different scene, target, profile, or legacy experiment; "
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

    profile, bounds, ceiling = resolve_sampling(args)
    output_dir = os.path.abspath(args.output or os.path.join(
        os.path.dirname(manifest_path), "carla_safety_variants", target, profile))
    samples = sampled_actor_behaviors(args.count, bounds, args.seed, eligible_ids, profile, ceiling)

    baseline_behavior = {
        "desired_speed_scale": 1.0,
        "leading_distance_m": float(args.baseline_leading_distance),
        "auto_lane_change": False,
        "random_left_lane_change_percentage": 0.0,
        "random_right_lane_change_percentage": 0.0,
        "keep_right_rule_percentage": 0.0,
        "target_speed_floor_kmh": 0.0,
        "target_speed_ceiling_kmh": 0.0,
        "ignore_vehicles_percentage": 0.0,
        "ignore_lights_percentage": 0.0,
        "ignore_signs_percentage": 0.0,
    }
    baseline = _variant_document(
        manifest_path, "carla_tm_%s_baseline" % target, "baseline", args.seed,
        args.simulation_seed, minimum_track_distance,
        minimum_two_point_speed, eligible_ids, retained_vehicle_ids,
        baseline_behavior, target, moving_ids, baseline_behavior,
        profile, {actor_id: dict(baseline_behavior) for actor_id in eligible_ids})
    prepared_variants = []
    for number, actor_behaviors in enumerate(samples):
        behavior = actor_behaviors[eligible_ids[0]]
        variant_id = "carla_tm_%s_%03d" % (target, number)
        document = _variant_document(
            manifest_path, variant_id, "sample", args.seed,
            args.simulation_seed, minimum_track_distance,
            minimum_two_point_speed, eligible_ids, retained_vehicle_ids,
            behavior, target, moving_ids, baseline_behavior, profile, actor_behaviors)
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
        "aggression_profile": profile,
        "parameter_ranges": dict(bounds),
        "target_speed_ceiling_kmh": ceiling,
        "sampling": "shared" if profile == "mild" else "independent_per_track",
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
            "actor_behaviors": document["actor_behaviors"],
        })
    index_path = os.path.abspath(os.path.join(output_dir, "index.json"))
    write_json(index_path, index)
    print("Generated matched CARLA %s baseline and %d variants in %s" %
          (target, args.count, output_dir))
    print("Perturbation targets:", len(eligible_ids), "(" + target + ")")
    print("Aggression profile:", profile, "(collision physics remains enabled)")
    if any(dict(bounds)[key][1] > 0 for key in (
            "ignore_vehicles_percentage", "ignore_lights_percentage", "ignore_signs_percentage")):
        print("Selected drivers intentionally bypass some TM hazard/rule checks; collisions are possible, not guaranteed.")
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
    parser.add_argument("--profile", choices=tuple(PROFILE_RANGES), default="aggressive",
                        help="aggressive (default), stress (100%% vehicle/rule ignore), or old mild ranges")
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
    for key, option in RANGE_OPTIONS.items():
        parser.add_argument("--" + option.replace("_", "-"), type=parse_range, default=None,
                            metavar="MIN,MAX", help="override profile range for " + key)
    parser.add_argument("--maximum-speed-kmh", type=float, default=None,
                        help="target-speed ceiling: mild=0 (off), aggressive=80, stress=100")
    parser.add_argument("--baseline-leading-distance", type=float, default=2.5)
    parser.add_argument("--output", help="exact output folder; default: "
                        "<scene>/carla_safety_variants/<target>/<profile>")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.count <= 0:
        parser.error("--count must be positive")
    run(args)


if __name__ == "__main__":
    main()
