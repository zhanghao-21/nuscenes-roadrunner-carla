#!/usr/bin/env python3
"""Generate all-SUMO moving-traffic safety variants for hybrid simulation."""

import argparse
import copy
import os
import random
import sys
import xml.etree.ElementTree as ET


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.safety_variants import (  # noqa: E402
    SUMO_LANE_CHANGE_LIMITS, SUMO_PIPELINE, SUMO_SCOPE,
    VARIANT_SCHEMA_VERSION, latin_hypercube, load_json, parse_range,
    prune_stale_numbered_variants, resolve_path, validate_range,
    validate_sumo_variant, write_json)


FACTOR_LIMITS = (0.25, 3.0)
DEFAULT_RANGES = {
    "tau_factor": (0.65, 1.40),
    "min_gap_factor": (0.60, 1.50),
    "accel_factor": (0.75, 1.30),
    "decel_factor": (0.75, 1.30),
    "lc_strategic": (0.30, 2.00),
    "lc_cooperative": (0.10, 1.00),
    "lc_speed_gain": (0.20, 2.00),
    "lc_keep_right": (0.00, 1.20),
    "lc_assertive": (0.70, 1.80),
}


def _hybrid_child_path(config, config_path, *keys):
    value = config
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return resolve_path(value, os.path.dirname(config_path))


def _canonicalize_hybrid_paths(config, config_path):
    """Make source-config-relative paths stable in child experiment files."""
    value = copy.deepcopy(config)
    parent = os.path.dirname(config_path)
    if value.get("manifest"):
        value["manifest"] = resolve_path(value["manifest"], parent)
    sumo = value.get("sumo")
    if isinstance(sumo, dict):
        for key in ("config", "network", "route_report"):
            if sumo.get(key):
                sumo[key] = resolve_path(sumo[key], parent)
    prediction = value.get("prediction_risk")
    model = prediction.get("model") if isinstance(prediction, dict) else None
    if isinstance(model, dict):
        for key in ("config", "checkpoint"):
            if model.get(key):
                model[key] = resolve_path(model[key], parent)
    return value


def _route_type_definitions(config, config_path):
    sumo_config = _hybrid_child_path(config, config_path, "sumo", "config")
    if not sumo_config or not os.path.isfile(sumo_config):
        raise ValueError("hybrid configuration has no readable SUMO config")
    root = ET.parse(sumo_config).getroot()
    route_node = root.find("./input/route-files")
    if route_node is None or not route_node.get("value"):
        raise ValueError("SUMO config has no route-files entry")
    route_name = route_node.get("value").split(",")[0].strip()
    route_path = resolve_path(route_name, os.path.dirname(sumo_config))
    if not route_path or not os.path.isfile(route_path):
        raise ValueError("SUMO route file is unavailable: %s" % route_path)
    route_root = ET.parse(route_path).getroot()
    types = {
        item.get("id"): dict(item.attrib)
        for item in route_root.findall("./vType")
        if item.get("id")
    }
    vehicle_types = {
        item.get("id"): item.get("type")
        for item in route_root.findall("./vehicle")
        if item.get("id") and item.get("type")
    }
    return route_path, types, vehicle_types


def _float_attribute(attributes, name, default=None):
    value = attributes.get(name, default)
    if value is None:
        raise ValueError("SUMO vType is missing %s" % name)
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError("SUMO vType %s is not numeric: %r" % (name, value))


def _baseline_from_type(type_id, attributes):
    return {
        "car_following": {
            "tau_s": _float_attribute(attributes, "tau"),
            "min_gap_m": _float_attribute(attributes, "minGap"),
            "accel_mps2": _float_attribute(attributes, "accel"),
            "decel_mps2": _float_attribute(attributes, "decel"),
            "apparent_decel_mps2": _float_attribute(
                attributes, "apparentDecel",
                _float_attribute(attributes, "decel")),
            "emergency_decel_mps2": _float_attribute(
                attributes, "emergencyDecel", 9.0),
        },
        "lane_changing": {
            "lc_strategic": _float_attribute(
                attributes, "lcStrategic", 1.0),
            "lc_cooperative": _float_attribute(
                attributes, "lcCooperative", 1.0),
            "lc_speed_gain": _float_attribute(
                attributes, "lcSpeedGain", 1.0),
            "lc_keep_right": _float_attribute(
                attributes, "lcKeepRight", 1.0),
            "lc_assertive": _float_attribute(
                attributes, "lcAssertive", 1.0),
        },
        "sumo_type_id": type_id,
    }


def audit_hybrid_base(config, config_path):
    """Return the exact moving-SUMO population and its explicit baselines."""
    critical = config.get("critical_actors") or []
    if critical:
        raise ValueError(
            "SUMO safety variants do not select CARLA critical vehicles. "
            "Rerun prepare_sumo.py without --critical-actor before generating "
            "this pipeline.")
    background = config.get("background")
    if not isinstance(background, dict) or background.get("authority") != "sumo":
        raise ValueError("hybrid background.authority must be sumo")
    moving_speed = background.get("moving_speed") or {}
    if moving_speed.get("policy") != "unbounded":
        raise ValueError(
            "SUMO behavior variants require a base prepared with "
            "--moving-speed-policy unbounded so routes do not retain hidden "
            "per-track maxSpeed caps")
    if config.get("sumo_behavior_variant") is not None:
        raise ValueError("generate variants from a baseline hybrid config, not a variant")

    report_path = _hybrid_child_path(
        config, config_path, "sumo", "route_report")
    if not report_path or not os.path.isfile(report_path):
        raise ValueError("hybrid route_report is unavailable: %s" % report_path)
    report = load_json(report_path)
    included = report.get("included")
    if not isinstance(included, list) or not included:
        raise ValueError("route report contains no moving SUMO vehicles")
    report_speed = report.get("moving_speed") or {}
    if report_speed.get("policy") != "unbounded":
        raise ValueError(
            "route report was not prepared with --moving-speed-policy "
            "unbounded; changing only hybrid_config.json is insufficient")
    capped_ids = sorted(
        str(item.get("id")) for item in included
        if item.get("sumo_max_speed_mps") is not None)
    if capped_ids:
        raise ValueError(
            "route report still contains recorded maxSpeed caps for %s; "
            "rerun prepare_sumo.py with --moving-speed-policy unbounded" %
            capped_ids)
    legacy_exclusions = [
        item.get("id") for item in report.get("skipped", [])
        if item.get("reason") == "CARLA authority"
    ]
    if legacy_exclusions:
        raise ValueError(
            "route report still excludes CARLA-authority moving vehicles %s; "
            "rerun prepare_sumo.py without --critical-actor" %
            sorted(legacy_exclusions))

    sumo_ids = [str(item.get("sumo_id", "")) for item in included]
    source_ids = [str(item.get("id", "")) for item in included]
    if (any(not value for value in sumo_ids + source_ids) or
            len(set(sumo_ids)) != len(sumo_ids) or
            len(set(source_ids)) != len(source_ids)):
        raise ValueError("route report moving IDs must be present and unique")
    configured_sources = background.get("track_ids")
    if (not isinstance(configured_sources, list) or
            set(str(value) for value in configured_sources) != set(source_ids)):
        raise ValueError(
            "hybrid background.track_ids does not match route_report included IDs")
    static_ids = {
        str(item.get("track_id"))
        for item in config.get("static_actors", [])
        if item.get("track_id")
    }
    overlap = sorted(static_ids.intersection(source_ids))
    if overlap:
        raise ValueError(
            "moving SUMO and CARLA-static actor partitions overlap: %s" % overlap)
    reported_static_ids = {
        str(item.get("id"))
        for item in report.get("carla_static", [])
        if item.get("id")
    }
    if static_ids != reported_static_ids:
        raise ValueError(
            "hybrid static_actors does not match route_report carla_static; "
            "missing=%s extra=%s" %
            (sorted(reported_static_ids - static_ids),
             sorted(static_ids - reported_static_ids)))
    unrouted = [
        (str(item.get("id")), str(item.get("reason")))
        for item in report.get("skipped", [])
        if item.get("reason") not in ("not a vehicle", "CARLA authority")
    ]
    if unrouted:
        raise ValueError(
            "route report contains moving vehicles that SUMO could not route: %s" %
            unrouted)

    route_path, type_definitions, vehicle_types = _route_type_definitions(
        config, config_path)

    rows = []
    for item in included:
        sumo_id = str(item["sumo_id"])
        type_id = item.get("sumo_type_id") or vehicle_types.get(sumo_id)
        attributes = type_definitions.get(type_id)
        if attributes is None:
            raise ValueError(
                "SUMO route file has no vType %s for %s" %
                (type_id, sumo_id))
        if attributes.get("maxSpeed") is not None:
            raise ValueError(
                "SUMO route vType %s still has maxSpeed=%s; rerun "
                "prepare_sumo.py with --moving-speed-policy unbounded" %
                (type_id, attributes["maxSpeed"]))
        baseline = item.get("sumo_behavior_baseline")
        if not isinstance(baseline, dict):
            baseline = _baseline_from_type(type_id, attributes)
        else:
            baseline = copy.deepcopy(baseline)
            baseline["sumo_type_id"] = type_id
        rows.append({
            "sumo_id": sumo_id,
            "source_track_id": str(item["id"]),
            "sumo_type_id": type_id,
            "baseline": baseline,
        })
    return {
        "report_path": report_path,
        "route_path": route_path,
        "rows": rows,
        "sumo_ids": sumo_ids,
        "source_ids": source_ids,
    }


def _round(value):
    return round(float(value), 6)


def _resolved_vehicle(row, profile=None):
    baseline = row["baseline"]
    base_cf = baseline["car_following"]
    base_lc = baseline["lane_changing"]
    if profile is None:
        car_following = {key: _round(value)
                         for key, value in base_cf.items()}
        lane_changing = {key: _round(value)
                         for key, value in base_lc.items()}
    else:
        car_following = {
            "tau_s": _round(base_cf["tau_s"] * profile["tau_factor"]),
            "min_gap_m": _round(
                base_cf["min_gap_m"] * profile["min_gap_factor"]),
            "accel_mps2": _round(
                base_cf["accel_mps2"] * profile["accel_factor"]),
            "decel_mps2": _round(
                base_cf["decel_mps2"] * profile["decel_factor"]),
            "apparent_decel_mps2": _round(
                base_cf.get("apparent_decel_mps2", base_cf["decel_mps2"]) *
                profile["decel_factor"]),
            "emergency_decel_mps2": _round(
                base_cf["emergency_decel_mps2"]),
        }
        lane_changing = {
            key: _round(profile[key])
            for key in SUMO_LANE_CHANGE_LIMITS
        }
    if car_following["decel_mps2"] > car_following[
            "emergency_decel_mps2"]:
        raise ValueError(
            "resolved comfortable deceleration exceeds emergency deceleration "
            "for %s" % row["sumo_id"])
    return {
        "source_track_id": row["source_track_id"],
        "sumo_type_id": row["sumo_type_id"],
        "car_following": car_following,
        "lane_changing": lane_changing,
    }


def _variant_section(audit, variant_id, kind, generation_seed, profile=None):
    document = {
        "schema_version": VARIANT_SCHEMA_VERSION,
        "pipeline": SUMO_PIPELINE,
        "variant_id": variant_id,
        "kind": kind,
        "scope": {
            "policy": SUMO_SCOPE,
            "sumo_vehicle_ids": list(audit["sumo_ids"]),
            "source_track_ids": list(audit["source_ids"]),
        },
        "generation": {
            "method": "latin_hypercube" if profile is not None else "baseline",
            "seed": int(generation_seed),
        },
        "sampled_profile": (
            copy.deepcopy(profile) if profile is not None else {
                "car_following_multipliers": {
                    "tau_factor": 1.0,
                    "min_gap_factor": 1.0,
                    "accel_factor": 1.0,
                    "decel_factor": 1.0,
                },
                "lane_changing": "per_vehicle_vtype_defaults",
            }),
        "vehicles": {
            row["sumo_id"]: _resolved_vehicle(row, profile)
            for row in audit["rows"]
        },
    }
    return validate_sumo_variant(document, audit["sumo_ids"])


def _experiment_config(base, section, source_speed_policy):
    value = copy.deepcopy(base)
    background = value["background"]
    moving_speed = copy.deepcopy(background.get("moving_speed") or {})
    moving_speed["policy"] = "unbounded"
    background["moving_speed"] = moving_speed
    value["sumo_behavior_variant"] = section
    value["scenario"] = {
        "name": section["variant_id"],
        "pipeline": SUMO_PIPELINE,
        "generator": section["generation"]["method"],
        "source_moving_speed_policy": source_speed_policy,
        "effective_moving_speed_policy": "unbounded",
        "parameters": section["sampled_profile"],
    }
    return value


def run(args):
    if (isinstance(args.count, bool) or not isinstance(args.count, int) or
            args.count <= 0):
        raise ValueError("--count must be positive")
    config_path = os.path.abspath(args.config)
    base = _canonicalize_hybrid_paths(load_json(config_path), config_path)
    audit = audit_hybrid_base(base, config_path)
    source_speed_policy = str(
        ((base.get("background") or {}).get("moving_speed") or {}).get(
            "policy", "unbounded"))
    output_dir = os.path.abspath(args.output or os.path.join(
        os.path.dirname(config_path), "sumo_safety_variants"))
    os.makedirs(output_dir, exist_ok=True)

    bounds = [
        ("tau_factor", validate_range(
            "tau factor", args.tau_factor, FACTOR_LIMITS)),
        ("min_gap_factor", validate_range(
            "minimum-gap factor", args.min_gap_factor, FACTOR_LIMITS)),
        ("accel_factor", validate_range(
            "acceleration factor", args.accel_factor, FACTOR_LIMITS)),
        ("decel_factor", validate_range(
            "deceleration factor", args.decel_factor, FACTOR_LIMITS)),
        ("lc_strategic", validate_range(
            "lcStrategic", args.lc_strategic,
            SUMO_LANE_CHANGE_LIMITS["lc_strategic"])),
        ("lc_cooperative", validate_range(
            "lcCooperative", args.lc_cooperative,
            SUMO_LANE_CHANGE_LIMITS["lc_cooperative"])),
        ("lc_speed_gain", validate_range(
            "lcSpeedGain", args.lc_speed_gain,
            SUMO_LANE_CHANGE_LIMITS["lc_speed_gain"])),
        ("lc_keep_right", validate_range(
            "lcKeepRight", args.lc_keep_right,
            SUMO_LANE_CHANGE_LIMITS["lc_keep_right"])),
        ("lc_assertive", validate_range(
            "lcAssertive", args.lc_assertive,
            SUMO_LANE_CHANGE_LIMITS["lc_assertive"])),
    ]
    samples = latin_hypercube(
        args.count, bounds, random.Random(args.seed))

    baseline_section = _variant_section(
        audit, "sumo_hybrid_baseline", "baseline", args.seed)
    prepared_variants = []
    for number, profile in enumerate(samples):
        profile = {key: _round(value) for key, value in profile.items()}
        variant_id = "sumo_hybrid_%03d" % number
        section = _variant_section(
            audit, variant_id, "sample", args.seed, profile)
        prepared_variants.append((
            number, variant_id, profile,
            _experiment_config(base, section, source_speed_policy)))
    removed = prune_stale_numbered_variants(output_dir, args.count)
    if removed:
        print("Removed stale generated SUMO variants:", ", ".join(removed))
    baseline_path = os.path.abspath(os.path.join(output_dir, "baseline.json"))
    write_json(
        baseline_path,
        _experiment_config(
            base, baseline_section, source_speed_policy))

    index = {
        "schema_version": VARIANT_SCHEMA_VERSION,
        "pipeline": SUMO_PIPELINE,
        "source_config": config_path,
        "route_report": audit["report_path"],
        "generation_seed": int(args.seed),
        "simulation_seed": int(base.get("seed", 103)),
        "source_moving_speed_policy": source_speed_policy,
        "effective_moving_speed_policy": "unbounded",
        "scope": baseline_section["scope"],
        "baseline_config": baseline_path,
        "variants": [],
    }
    for number, variant_id, profile, variant in prepared_variants:
        path = os.path.abspath(os.path.join(
            output_dir, "variant_%03d.json" % number))
        write_json(path, variant)
        index["variants"].append({
            "variant_id": variant_id,
            "config": path,
            "sampled_profile": profile,
        })
    index_path = os.path.abspath(os.path.join(output_dir, "index.json"))
    write_json(index_path, index)
    print("Generated matched SUMO baseline and %d variants in %s" %
          (args.count, output_dir))
    print("Moving SUMO vehicles per experiment:", len(audit["sumo_ids"]))
    print("Index:", index_path)
    return index


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True,
                        help="baseline hybrid_config.json prepared with no critical actor")
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed", type=int, default=103,
                        help="Latin-hypercube seed; the SUMO simulation seed is preserved")
    parser.add_argument("--tau-factor", type=parse_range,
                        default=DEFAULT_RANGES["tau_factor"], metavar="MIN,MAX")
    parser.add_argument("--min-gap-factor", type=parse_range,
                        default=DEFAULT_RANGES["min_gap_factor"], metavar="MIN,MAX")
    parser.add_argument("--accel-factor", type=parse_range,
                        default=DEFAULT_RANGES["accel_factor"], metavar="MIN,MAX")
    parser.add_argument("--decel-factor", type=parse_range,
                        default=DEFAULT_RANGES["decel_factor"], metavar="MIN,MAX")
    parser.add_argument("--lc-strategic", type=parse_range,
                        default=DEFAULT_RANGES["lc_strategic"], metavar="MIN,MAX")
    parser.add_argument("--lc-cooperative", type=parse_range,
                        default=DEFAULT_RANGES["lc_cooperative"], metavar="MIN,MAX")
    parser.add_argument("--lc-speed-gain", type=parse_range,
                        default=DEFAULT_RANGES["lc_speed_gain"], metavar="MIN,MAX")
    parser.add_argument("--lc-keep-right", type=parse_range,
                        default=DEFAULT_RANGES["lc_keep_right"], metavar="MIN,MAX")
    parser.add_argument("--lc-assertive", type=parse_range,
                        default=DEFAULT_RANGES["lc_assertive"], metavar="MIN,MAX")
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
