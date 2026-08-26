#!/usr/bin/env python3
"""Generate deterministic critical-braking variants of a hybrid scenario."""

import argparse
import copy
import json
import math
import os
import random
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.tracks import load_manifest_tracks  # noqa: E402


def load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def parse_range(value):
    parts = [float(item.strip()) for item in value.split(",")]
    if len(parts) != 2 or parts[0] > parts[1]:
        raise argparse.ArgumentTypeError("range must be MIN,MAX")
    return tuple(parts)


def latin_hypercube(count, bounds, rng):
    """Return one stratified, independently shuffled column per bound."""
    columns = []
    for lower, upper in bounds:
        values = []
        for index in range(count):
            ratio = (index + rng.random()) / count
            values.append(lower + ratio * (upper - lower))
        rng.shuffle(values)
        columns.append(values)
    return list(zip(*columns))


def critical_track_window(base, config_path, actor_id):
    """Return the finite recorded control window for a critical actor."""
    manifest_path = base.get("manifest")
    if not manifest_path:
        raise ValueError("hybrid configuration has no manifest")
    if not os.path.isabs(manifest_path):
        manifest_path = os.path.join(
            os.path.dirname(config_path), manifest_path)
    _manifest, bundle = load_manifest_tracks(os.path.abspath(manifest_path))
    track = bundle.actors.get(actor_id)
    if track is None:
        raise ValueError(
            "critical actor %s is absent from the scene manifest" % actor_id)
    duration = float(track.end_time - track.start_time)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError(
            "critical actor %s has no positive recorded control window" %
            actor_id)
    return {
        "start_time_s": float(track.start_time),
        "end_time_s": float(track.end_time),
        "controllable_duration_s": duration,
    }


def bounded_brake_ranges(track_duration, start_range, duration_range):
    """Clip requested timing ranges so every event can start on the track."""
    values = (track_duration,) + tuple(start_range) + tuple(duration_range)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("brake timing values must be finite")
    track_duration = float(track_duration)
    start_lower, start_upper = map(float, start_range)
    duration_lower, duration_upper = map(float, duration_range)
    if track_duration <= 0.0:
        raise ValueError("critical track duration must be positive")
    if start_lower > start_upper:
        raise ValueError("brake start range must be ordered")
    if duration_lower <= 0.0 or duration_lower > duration_upper:
        raise ValueError("brake duration range must be positive and ordered")
    if duration_lower > track_duration:
        raise ValueError(
            "minimum brake duration %.3f s exceeds the critical track's "
            "%.3f s controllable window" %
            (duration_lower, track_duration))

    effective_duration = (
        duration_lower, min(duration_upper, track_duration))
    latest_start = track_duration - duration_lower
    effective_start = (
        max(0.0, min(start_lower, latest_start)),
        max(0.0, min(start_upper, latest_start)),
    )
    return effective_start, effective_duration


def fit_brake_event(start, duration, track_duration):
    """Keep the complete braking event inside the finite control window."""
    start = max(0.0, min(float(start), float(track_duration)))
    duration = min(float(duration), max(0.0, float(track_duration) - start))
    return start, duration


def run(args):
    config_path = os.path.abspath(args.config)
    base = load_json(config_path)
    critical = {
        item["track_id"]: item for item in base.get("critical_actors", [])
    }
    if args.critical_actor not in critical:
        raise ValueError(
            "actor %s is not CARLA-authority in this configuration; rerun "
            "prepare_sumo.py with --critical-actor %s first" %
            (args.critical_actor, args.critical_actor))

    output_dir = os.path.abspath(args.output or os.path.join(
        os.path.dirname(config_path), "variants_%s" % args.critical_actor[:8]))
    os.makedirs(output_dir, exist_ok=True)
    track_window = critical_track_window(
        base, config_path, args.critical_actor)
    effective_brake_start, effective_brake_duration = bounded_brake_ranges(
        track_window["controllable_duration_s"],
        args.brake_start, args.brake_duration)
    timing_ranges_adjusted = (
        tuple(args.brake_start) != effective_brake_start or
        tuple(args.brake_duration) != effective_brake_duration)
    if timing_ranges_adjusted:
        print("Adjusted braking ranges to the critical actor's finite %.3f s "
              "control window: start %.3f..%.3f s, duration %.3f..%.3f s" %
              ((track_window["controllable_duration_s"],) +
               effective_brake_start + effective_brake_duration))
    rng = random.Random(args.seed)
    samples = latin_hypercube(args.count, [
        effective_brake_start, effective_brake_duration, args.brake_intensity,
        args.speed_scale, args.time_headway,
    ], rng)
    index = {
        "source_config": config_path,
        "critical_actor": args.critical_actor,
        "seed": args.seed,
        "critical_track_window": track_window,
        "requested_brake_start_range_s": list(args.brake_start),
        "effective_brake_start_range_s": list(effective_brake_start),
        "requested_brake_duration_range_s": list(args.brake_duration),
        "effective_brake_duration_range_s": list(effective_brake_duration),
        "variants": [],
    }
    adjusted_event_count = 0

    for number, sample in enumerate(samples):
        start, duration, intensity, speed_scale, time_headway = sample
        original_duration = duration
        start, duration = fit_brake_event(
            start, duration, track_window["controllable_duration_s"])
        if abs(duration - original_duration) > 1.0e-9:
            adjusted_event_count += 1
        variant = copy.deepcopy(base)
        spec = next(item for item in variant["critical_actors"]
                    if item["track_id"] == args.critical_actor)
        spec["authority"] = "carla_reference"
        spec["desired_speed_scale"] = speed_scale
        spec["time_headway"] = time_headway
        spec["events"] = [{
            "type": "brake",
            "start": start,
            "duration": duration,
            "intensity": intensity,
        }]
        parameters = {
            "brake_start_s": start,
            "brake_duration_s": duration,
            "brake_end_s": start + duration,
            "brake_intensity": intensity,
            "desired_speed_scale": speed_scale,
            "time_headway_s": time_headway,
        }
        variant["scenario"] = {
            "name": "critical_brake_%03d" % number,
            "generator": "latin_hypercube",
            "parameters": parameters,
        }
        filename = "variant_%03d.json" % number
        path = os.path.join(output_dir, filename)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(variant, stream, indent=2, sort_keys=False)
            stream.write("\n")
        index["variants"].append({"config": os.path.abspath(path), **parameters})

    index["adjusted_event_count"] = adjusted_event_count
    index_path = os.path.join(output_dir, "index.json")
    with open(index_path, "w", encoding="utf-8") as stream:
        json.dump(index, stream, indent=2, sort_keys=False)
        stream.write("\n")
    print("Generated %d variants in %s" % (args.count, output_dir))
    print("Index:", index_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--critical-actor", required=True)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed", type=int, default=103)
    parser.add_argument("--brake-start", type=parse_range, default=(4.0, 12.0),
                        metavar="MIN,MAX")
    parser.add_argument("--brake-duration", type=parse_range, default=(0.5, 2.5),
                        metavar="MIN,MAX")
    parser.add_argument("--brake-intensity", type=parse_range, default=(0.4, 1.0),
                        metavar="MIN,MAX")
    parser.add_argument("--speed-scale", type=parse_range, default=(0.8, 1.2),
                        metavar="MIN,MAX")
    parser.add_argument("--time-headway", type=parse_range, default=(0.8, 2.0),
                        metavar="MIN,MAX")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    run(args)


if __name__ == "__main__":
    main()
