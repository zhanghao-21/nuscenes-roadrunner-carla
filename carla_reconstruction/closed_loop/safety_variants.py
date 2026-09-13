"""Shared schemas and deterministic sampling for safety-variant pipelines.

This module deliberately has no CARLA, SUMO, or TraCI imports.  Generators and
offline tests can therefore validate experiment artifacts without either
simulator being installed.
"""

import argparse
import copy
import json
import math
import os
import re


VARIANT_SCHEMA_VERSION = 1
CARLA_VARIANT_SCHEMA_VERSION = 3
SUMO_PIPELINE = "sumo_hybrid"
CARLA_PIPELINE = "carla_tm"
SUMO_SCOPE = "all_moving_background"
CARLA_SCOPE = "all_moving_surrounding"
CARLA_EGO_SCOPE = "ego_only"
MAX_CARLA_RANDOM_SEED = (1 << 64) - 1


SUMO_CAR_FOLLOW_LIMITS = {
    "tau_s": (0.1, 5.0),
    "min_gap_m": (0.0, 20.0),
    "accel_mps2": (0.1, 10.0),
    "decel_mps2": (0.1, 10.0),
    "apparent_decel_mps2": (0.1, 20.0),
    "emergency_decel_mps2": (0.1, 20.0),
}
SUMO_LANE_CHANGE_LIMITS = {
    "lc_strategic": (0.0, 5.0),
    "lc_cooperative": (0.0, 1.0),
    "lc_speed_gain": (0.0, 5.0),
    "lc_keep_right": (0.0, 5.0),
    "lc_assertive": (0.5, 3.0),
}
CARLA_BEHAVIOR_LIMITS = {
    "desired_speed_scale": (0.25, 4.0),
    "leading_distance_m": (0.0, 20.0),
    "random_left_lane_change_percentage": (0.0, 100.0),
    "random_right_lane_change_percentage": (0.0, 100.0),
    "keep_right_rule_percentage": (0.0, 100.0),
}
CARLA_AGGRESSION_LIMITS = {
    "target_speed_floor_kmh": (0.0, 100.0),
    "target_speed_ceiling_kmh": (0.0, 140.0),
    "ignore_vehicles_percentage": (0.0, 100.0),
    "ignore_lights_percentage": (0.0, 100.0),
    "ignore_signs_percentage": (0.0, 100.0),
}


def load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=False)
        stream.write("\n")


def resolve_path(path, parent):
    if not path:
        return None
    if os.path.isabs(path):
        return os.path.abspath(path)
    return os.path.abspath(os.path.join(parent, path))


def prune_stale_numbered_variants(output_dir, count):
    """Remove only obsolete files owned by a numbered variant generator."""
    if not os.path.isdir(output_dir):
        return []
    expected = {"variant_%03d.json" % index for index in range(count)}
    pattern = re.compile(r"^variant_[0-9]+\.json$")
    removed = []
    for name in os.listdir(output_dir):
        if pattern.match(name) and name not in expected:
            os.remove(os.path.join(output_dir, name))
            removed.append(name)
    return sorted(removed)


def parse_range(value):
    try:
        parts = [float(item.strip()) for item in value.split(",")]
    except (AttributeError, TypeError, ValueError):
        raise argparse.ArgumentTypeError("range must be MIN,MAX")
    if (len(parts) != 2 or not all(math.isfinite(item) for item in parts) or
            parts[0] > parts[1]):
        raise argparse.ArgumentTypeError("range must be finite MIN,MAX")
    return tuple(parts)


def validate_range(name, value, limits=None):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError("%s must contain MIN,MAX" % name)
    lower = _finite_number(value[0], name + " minimum")
    upper = _finite_number(value[1], name + " maximum")
    if lower > upper:
        raise ValueError("%s must be ordered MIN,MAX" % name)
    if limits is not None and (lower < limits[0] or upper > limits[1]):
        raise ValueError(
            "%s must stay inside %.3f..%.3f" %
            (name, limits[0], limits[1]))
    return lower, upper


def latin_hypercube(count, bounds, rng):
    """Return deterministic stratified samples for independently shuffled axes."""
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("variant count must be a positive integer")
    columns = []
    for name, value in bounds:
        lower, upper = validate_range(name, value)
        samples = []
        for index in range(count):
            ratio = (index + rng.random()) / count
            samples.append(lower + ratio * (upper - lower))
        rng.shuffle(samples)
        columns.append(samples)
    return [dict(zip((name for name, _value in bounds), row))
            for row in zip(*columns)]


def _finite_number(value, label):
    if isinstance(value, bool):
        raise ValueError("%s must be a finite number" % label)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be a finite number" % label)
    if not math.isfinite(result):
        raise ValueError("%s must be a finite number" % label)
    return result


def _bounded_number(mapping, key, limits, label):
    if key not in mapping:
        raise ValueError("%s requires %s" % (label, key))
    value = _finite_number(mapping[key], "%s.%s" % (label, key))
    if value < limits[0] or value > limits[1]:
        raise ValueError(
            "%s.%s must be inside %.3f..%.3f" %
            (label, key, limits[0], limits[1]))
    return value


def _string_list(value, label, allow_empty=False):
    if not isinstance(value, list):
        raise ValueError("%s must be a list" % label)
    result = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            raise ValueError("%s entries must be non-empty strings" % label)
        result.append(item)
    if not allow_empty and not result:
        raise ValueError("%s must not be empty" % label)
    if len(set(result)) != len(result):
        raise ValueError("%s contains duplicate IDs" % label)
    return result


def _validate_header(document, pipeline, versions=(VARIANT_SCHEMA_VERSION,)):
    if not isinstance(document, dict):
        raise ValueError("variant configuration must be a JSON object")
    if document.get("schema_version") not in versions:
        raise ValueError(
            "variant schema_version must be one of %s" % (versions,))
    if document.get("pipeline") != pipeline:
        raise ValueError(
            "variant pipeline must be %s" % pipeline)
    variant_id = document.get("variant_id")
    if not isinstance(variant_id, str) or not variant_id.strip():
        raise ValueError("variant_id must be a non-empty string")


def validate_carla_seed(value, label="CARLA Traffic Manager seed"):
    if (isinstance(value, bool) or not isinstance(value, int) or
            value < 0 or value > MAX_CARLA_RANDOM_SEED):
        raise ValueError(
            "%s must be an integer inside 0..%d" %
            (label, MAX_CARLA_RANDOM_SEED))
    return value


def validate_sumo_variant(document, expected_sumo_ids=None):
    """Validate and normalize one embedded SUMO behavior variant."""
    if document is None:
        return None
    _validate_header(document, SUMO_PIPELINE)
    scope = document.get("scope")
    if not isinstance(scope, dict) or scope.get("policy") != SUMO_SCOPE:
        raise ValueError(
            "SUMO variant scope.policy must be %s" % SUMO_SCOPE)
    sumo_ids = _string_list(
        scope.get("sumo_vehicle_ids"), "scope.sumo_vehicle_ids")
    source_ids = _string_list(
        scope.get("source_track_ids"), "scope.source_track_ids")
    if len(source_ids) != len(sumo_ids):
        raise ValueError(
            "SUMO variant source and SUMO ID lists must have equal length")
    if expected_sumo_ids is not None and set(sumo_ids) != set(expected_sumo_ids):
        missing = sorted(set(expected_sumo_ids) - set(sumo_ids))
        extra = sorted(set(sumo_ids) - set(expected_sumo_ids))
        raise ValueError(
            "SUMO variant must target every moving background vehicle; "
            "missing=%s extra=%s" % (missing, extra))

    vehicles = document.get("vehicles")
    if not isinstance(vehicles, dict) or set(vehicles) != set(sumo_ids):
        raise ValueError(
            "SUMO variant vehicles must match scope.sumo_vehicle_ids exactly")
    normalized = copy.deepcopy(document)
    mapped_source_ids = []
    for sumo_id in sumo_ids:
        spec = vehicles[sumo_id]
        if not isinstance(spec, dict):
            raise ValueError("SUMO vehicle %s specification must be an object" % sumo_id)
        source_id = spec.get("source_track_id")
        if source_id not in source_ids:
            raise ValueError(
                "SUMO vehicle %s has an unknown source_track_id" % sumo_id)
        mapped_source_ids.append(source_id)
        type_id = spec.get("sumo_type_id")
        if not isinstance(type_id, str) or not type_id:
            raise ValueError("SUMO vehicle %s requires sumo_type_id" % sumo_id)
        car_following = spec.get("car_following")
        lane_changing = spec.get("lane_changing")
        if not isinstance(car_following, dict):
            raise ValueError(
                "SUMO vehicle %s requires car_following parameters" % sumo_id)
        if not isinstance(lane_changing, dict):
            raise ValueError(
                "SUMO vehicle %s requires lane_changing parameters" % sumo_id)
        normalized_cf = {}
        for key, limits in SUMO_CAR_FOLLOW_LIMITS.items():
            normalized_cf[key] = _bounded_number(
                car_following, key, limits,
                "vehicles.%s.car_following" % sumo_id)
        if normalized_cf["decel_mps2"] > normalized_cf[
                "emergency_decel_mps2"]:
            raise ValueError(
                "SUMO vehicle %s comfortable deceleration cannot exceed "
                "emergency deceleration" % sumo_id)
        if normalized_cf["apparent_decel_mps2"] > normalized_cf[
                "emergency_decel_mps2"]:
            raise ValueError(
                "SUMO vehicle %s apparent deceleration cannot exceed "
                "emergency deceleration" % sumo_id)
        normalized_lc = {}
        for key, limits in SUMO_LANE_CHANGE_LIMITS.items():
            normalized_lc[key] = _bounded_number(
                lane_changing, key, limits,
                "vehicles.%s.lane_changing" % sumo_id)
        normalized["vehicles"][sumo_id]["car_following"] = normalized_cf
        normalized["vehicles"][sumo_id]["lane_changing"] = normalized_lc
    if (len(set(mapped_source_ids)) != len(mapped_source_ids) or
            set(mapped_source_ids) != set(source_ids)):
        raise ValueError(
            "SUMO variant vehicles must map one-to-one onto "
            "scope.source_track_ids")
    normalized["scope"]["sumo_vehicle_ids"] = list(sumo_ids)
    normalized["scope"]["source_track_ids"] = list(source_ids)
    return normalized


def _validate_carla_behavior(behavior, label="behavior", extended=False):
    if not isinstance(behavior, dict):
        raise ValueError("CARLA variant requires %s parameters" % label)
    if extended:
        unknown = set(behavior) - (set(CARLA_BEHAVIOR_LIMITS) | set(CARLA_AGGRESSION_LIMITS) | {"auto_lane_change"})
        if unknown:
            raise ValueError(label + " has unsupported settings: " + ", ".join(sorted(unknown)))
    auto_lane_change = behavior.get("auto_lane_change")
    if not isinstance(auto_lane_change, bool):
        raise ValueError("%s.auto_lane_change must be true or false" % label)
    result = {"auto_lane_change": auto_lane_change}
    for key, limits in CARLA_BEHAVIOR_LIMITS.items():
        result[key] = _bounded_number(behavior, key, limits, label)
    for key, limits in CARLA_AGGRESSION_LIMITS.items():
        # Old version-1/2 artifacts retain their original behavior: no new
        # speed floor/cap and no hazard bypass unless explicitly specified.
        result[key] = (_bounded_number(behavior, key, limits, label)
                       if extended or key in behavior else 0.0)
    ceiling = result["target_speed_ceiling_kmh"]
    if ceiling and result["target_speed_floor_kmh"] > ceiling:
        raise ValueError(label + " speed floor cannot exceed its speed ceiling")
    if not auto_lane_change and any(result[key] > 0.0 for key in (
            "random_left_lane_change_percentage",
            "random_right_lane_change_percentage", "keep_right_rule_percentage")):
        raise ValueError(
            "lane-change percentages must be zero when auto_lane_change is false")
    return result


def validate_carla_variant(document):
    """Validate and normalize one standalone CARLA Traffic Manager variant."""
    _validate_header(document, CARLA_PIPELINE, (1, 2, CARLA_VARIANT_SCHEMA_VERSION))
    scoped = document["schema_version"] >= 2
    extended = document["schema_version"] >= 3
    manifest = document.get("manifest")
    if not isinstance(manifest, str) or not manifest.strip():
        raise ValueError("CARLA variant requires manifest")
    selection = document.get("selection")
    scopes = (CARLA_SCOPE, CARLA_EGO_SCOPE) if scoped else (CARLA_SCOPE,)
    if not isinstance(selection, dict) or selection.get("scope") not in scopes:
        raise ValueError(
            "CARLA variant selection.scope must be one of %s" % (scopes,))
    eligible = _string_list(
        selection.get("eligible_track_ids"),
        "selection.eligible_track_ids")
    retained = _string_list(
        selection.get("retained_vehicle_track_ids"),
        "selection.retained_vehicle_track_ids",
        allow_empty=selection["scope"] == CARLA_EGO_SCOPE)
    if "ego" in retained:
        raise ValueError("retained_vehicle_track_ids contains only surrounding vehicles")
    if selection["scope"] == CARLA_EGO_SCOPE:
        if eligible != ["ego"]:
            raise ValueError("ego_only variants must target exactly ['ego']")
    elif "ego" in eligible or not set(eligible).issubset(set(retained)):
        raise ValueError(
            "eligible CARLA variant tracks must be retained vehicles")
    minimum_distance = _finite_number(
        selection.get("minimum_track_distance_m"),
        "selection.minimum_track_distance_m")
    minimum_speed = _finite_number(
        selection.get("minimum_two_point_speed_mps"),
        "selection.minimum_two_point_speed_mps")
    if minimum_distance < 0.0 or minimum_speed < 0.0:
        raise ValueError("CARLA variant motion thresholds must be non-negative")
    simulation_seed = validate_carla_seed(
        document.get("simulation_seed"), "CARLA variant simulation_seed")
    normalized_behavior = _validate_carla_behavior(document.get("behavior"), extended=extended)
    normalized = copy.deepcopy(document)
    if scoped:
        if document.get("ego_mode") != "tm":
            raise ValueError("scoped CARLA variants require ego_mode=tm")
        moving = _string_list(
            selection.get("moving_surrounding_track_ids"),
            "selection.moving_surrounding_track_ids", allow_empty=True)
        if not set(moving).issubset(set(retained)):
            raise ValueError("moving surrounding tracks must be retained vehicles")
        if selection["scope"] == CARLA_SCOPE and set(eligible) != set(moving):
            raise ValueError("surrounding variants must target every moving surrounding vehicle")
        baseline = _validate_carla_behavior(
            document.get("baseline_behavior"), "baseline_behavior", extended=extended)
        if document.get("kind") not in ("baseline", "sample"):
            raise ValueError("CARLA variant kind must be baseline or sample")
        if document["kind"] == "baseline" and normalized_behavior != baseline:
            raise ValueError("matched baseline behavior must equal baseline_behavior")
        normalized["baseline_behavior"] = baseline
        normalized["selection"]["moving_surrounding_track_ids"] = list(moving)
    if extended:
        if (baseline["desired_speed_scale"] != 1.0 or baseline["auto_lane_change"] or
                any(baseline[key] != 0.0 for key in CARLA_AGGRESSION_LIMITS) or
                any(baseline[key] != 0.0 for key in (
                    "random_left_lane_change_percentage", "random_right_lane_change_percentage", "keep_right_rule_percentage"))):
            raise ValueError("version-3 baseline_behavior must preserve normal speed, lane-change and hazard responses")
        actor_behaviors = document.get("actor_behaviors")
        if not isinstance(actor_behaviors, dict) or set(actor_behaviors) != set(eligible):
            raise ValueError("actor_behaviors must contain exactly the selected perturbation targets")
        normalized["actor_behaviors"] = {
            actor_id: _validate_carla_behavior(profile, "actor_behaviors." + actor_id, extended=True)
            for actor_id, profile in actor_behaviors.items()}
        if document["kind"] == "baseline" and any(
                profile != normalized["baseline_behavior"] for profile in normalized["actor_behaviors"].values()):
            raise ValueError("matched baseline actor_behaviors must equal baseline_behavior")
        if normalized_behavior != normalized["actor_behaviors"][eligible[0]]:
            raise ValueError("behavior must describe the first selected actor; see actor_behaviors for all targets")
    normalized["selection"]["eligible_track_ids"] = list(eligible)
    normalized["selection"]["retained_vehicle_track_ids"] = list(retained)
    normalized["selection"]["minimum_track_distance_m"] = minimum_distance
    normalized["selection"]["minimum_two_point_speed_mps"] = minimum_speed
    normalized["behavior"] = normalized_behavior
    normalized["simulation_seed"] = simulation_seed
    return normalized


def carla_behavior_for_track(document, track_id):
    """Choose a profile from a validated config without crossing target scopes.

    Version 1 keeps its original surrounding-only semantics. Versions 2/3 also
    explicitly configures every unaffected TM actor with the matched baseline.
    Static vehicles and pedestrians receive no TM behavior in either case.
    """
    if document is None:
        return None
    selection = document["selection"]
    if track_id in selection["eligible_track_ids"]:
        if document["schema_version"] >= 3:
            return document["actor_behaviors"][track_id]
        return document["behavior"]
    if (document["schema_version"] >= 2 and
            (track_id == "ego" or
             track_id in selection["moving_surrounding_track_ids"])):
        return document["baseline_behavior"]
    return None


def load_carla_variant(path):
    path = os.path.abspath(path)
    document = validate_carla_variant(load_json(path))
    document["manifest"] = resolve_path(document["manifest"], os.path.dirname(path))
    return document
