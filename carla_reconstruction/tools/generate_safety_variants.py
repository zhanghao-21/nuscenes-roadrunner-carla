#!/usr/bin/env python3
"""Generate deterministic critical-braking variants of a hybrid scenario."""

import argparse
import copy
import json
import os
import random


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
    rng = random.Random(args.seed)
    samples = latin_hypercube(args.count, [
        args.brake_start, args.brake_duration, args.brake_intensity,
        args.speed_scale, args.time_headway,
    ], rng)
    index = {
        "source_config": config_path,
        "critical_actor": args.critical_actor,
        "seed": args.seed,
        "variants": [],
    }

    for number, sample in enumerate(samples):
        start, duration, intensity, speed_scale, time_headway = sample
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
