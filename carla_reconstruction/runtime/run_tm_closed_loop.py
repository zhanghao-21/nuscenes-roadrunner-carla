#!/usr/bin/env python3
"""Run data-seeded interactive traffic with CARLA Traffic Manager."""

import argparse
import json
import os
import sys

import carla


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.carla_runtime import (  # noqa: E402
    CollisionMonitor, GroundProjector, actor_state, chase_transform,
    configure_tm_actor, default_run_output, place_from_point, spawn_track_actor)
from carla_reconstruction.closed_loop.metrics import SafetyMetrics  # noqa: E402
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    load_manifest_tracks, point_at)


def run(args):
    manifest_path = os.path.abspath(args.manifest)
    manifest, bundle = load_manifest_tracks(manifest_path)
    map_name = args.map or manifest["map"]["runtime_name"]
    output = os.path.abspath(args.output) if args.output else default_run_output(
        REPO_ROOT, manifest["scene"], "tm")

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world() if args.reuse_world else client.load_world(map_name)
    carla_map = world.get_map()
    projector = GroundProjector(carla_map)
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.step_length
    settings.no_rendering_mode = args.no_rendering
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_random_device_seed(args.seed)
    traffic_manager.set_global_distance_to_leading_vehicle(args.leading_distance)
    traffic_manager.set_osm_mode(True)
    traffic_manager.set_hybrid_physics_mode(False)

    spawned = {}
    offsets = {}
    authority = {}
    metrics = SafetyMetrics()
    clock = [0.0]
    collision_monitor = None
    spectator = world.get_spectator()

    ego_physics = args.ego_mode != "replay"
    ego, ego_offset = spawn_track_actor(
        world, bundle.ego, projector, "hero", physics=ego_physics)
    if ego is None:
        raise RuntimeError("could not spawn ego vehicle")
    spawned["ego"] = ego
    offsets["ego"] = ego_offset
    authority["ego"] = "carla_%s" % args.ego_mode
    if args.ego_mode == "tm":
        configure_tm_actor(
            traffic_manager, ego, bundle.ego, args.tm_port,
            args.path_spacing, args.leading_distance, args.auto_lane_change,
            args.minimum_speed_kmh)
    collision_monitor = CollisionMonitor(world, ego, metrics, lambda: clock[0])

    vehicle_tracks = [track for track in bundle.vehicle_tracks.values()
                      if len(track.points) >= 2]
    vehicle_tracks.sort(key=lambda track: (track.start_time, track.actor_id))
    if args.max_vehicles > 0:
        vehicle_tracks = vehicle_tracks[:args.max_vehicles]
    pedestrian_tracks = ([track for track in bundle.actors.values()
                          if not track.is_vehicle]
                         if args.replay_pedestrians else [])
    waiting = {track.actor_id: track for track in vehicle_tracks + pedestrian_tracks}
    live_tracks = {}

    def spawn_due(sim_time):
        for actor_id, track in list(waiting.items()):
            if track.start_time > sim_time + 1.0e-9:
                continue
            moving = track.is_vehicle and track.distance >= args.minimum_track_distance
            actor, offset = spawn_track_actor(
                world, track, projector, "nuscenes_agent", physics=moving)
            del waiting[actor_id]
            if actor is None:
                print("WARNING: could not spawn", actor_id, track.category)
                continue
            spawned[actor_id], offsets[actor_id] = actor, offset
            live_tracks[actor_id] = track
            if moving:
                authority[actor_id] = "carla_tm"
                configure_tm_actor(
                    traffic_manager, actor, track, args.tm_port,
                    args.path_spacing, args.leading_distance,
                    args.auto_lane_change, args.minimum_speed_kmh)
            else:
                authority[actor_id] = ("replay_pedestrian" if not track.is_vehicle
                                       else "static_recorded")

    duration = args.duration if args.duration is not None else bundle.duration
    print("CARLA server:", client.get_server_version())
    print("Map:", carla_map.name)
    print("Ego authority:", authority["ego"])
    print("Candidate surrounding vehicles:", len(vehicle_tracks))
    print("Duration %.2fs at %.3fs/tick" % (duration, args.step_length))

    termination_reason = "duration_reached"
    try:
        while clock[0] <= duration + 1.0e-9:
            sim_time = clock[0]
            spawn_due(sim_time)
            if args.ego_mode == "replay":
                place_from_point(
                    ego, point_at(bundle.ego, sim_time), projector, ego_offset)

            for actor_id, track in list(live_tracks.items()):
                actor = spawned[actor_id]
                if authority[actor_id] == "replay_pedestrian":
                    place_from_point(
                        actor, point_at(track, sim_time), projector, offsets[actor_id])
                if args.despawn_at_track_end and sim_time > track.end_time:
                    try:
                        actor.set_autopilot(False, args.tm_port)
                    except Exception:
                        pass
                    actor.destroy()
                    del spawned[actor_id]
                    del live_tracks[actor_id]

            world.tick()
            if not ego.is_alive:
                termination_reason = "ego_destroyed"
                print("WARNING: ego actor was destroyed; ending this run")
                break
            spectator.set_transform(chase_transform(ego))
            others = {
                actor_id: actor_state(actor)
                for actor_id, actor in spawned.items()
                if actor_id != "ego" and actor.is_alive
            }
            metrics.update(sim_time, actor_state(ego), others)
            clock[0] += args.step_length
    finally:
        run_metadata = {
            "mode": "traffic_manager",
            "manifest": manifest_path,
            "map": map_name,
            "seed": args.seed,
            "step_length": args.step_length,
            "duration": duration,
            "termination_reason": termination_reason,
            "ego_mode": args.ego_mode,
            "actor_authority": authority,
        }
        csv_path, summary_path = metrics.write(output, run_metadata)
        with open(os.path.join(output, "run_config.json"), "w", encoding="utf-8") as stream:
            json.dump(run_metadata, stream, indent=2, sort_keys=False)
            stream.write("\n")
        if collision_monitor:
            collision_monitor.destroy()
        for actor in list(spawned.values()):
            try:
                if actor.type_id.startswith("vehicle."):
                    actor.set_autopilot(False, args.tm_port)
                actor.destroy()
            except Exception:
                pass
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print("Metrics:", csv_path)
        print("Summary:", summary_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--map", help="CARLA map path; defaults to manifest runtime_name")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--seed", type=int, default=103)
    parser.add_argument("--ego-mode", choices=("replay", "tm", "external"), default="replay")
    parser.add_argument("--max-vehicles", type=int, default=0,
                        help="maximum moving/static surrounding vehicles; 0 means all")
    parser.add_argument("--minimum-track-distance", type=float, default=2.0,
                        help="shorter vehicle tracks remain static instead of entering TM")
    parser.add_argument("--path-spacing", type=float, default=2.0)
    parser.add_argument("--leading-distance", type=float, default=2.5)
    parser.add_argument("--minimum-speed-kmh", type=float, default=3.0)
    parser.add_argument("--auto-lane-change", action="store_true")
    parser.add_argument("--replay-pedestrians", action="store_true")
    parser.add_argument("--despawn-at-track-end", action="store_true")
    parser.add_argument("--reuse-world", action="store_true")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--output")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
