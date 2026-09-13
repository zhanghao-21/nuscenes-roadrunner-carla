#!/usr/bin/env python3
"""Run data-seeded interactive traffic with CARLA Traffic Manager."""

import argparse
import json
import os
import sys
import time

import carla


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from carla_reconstruction.closed_loop.carla_runtime import (  # noqa: E402
    CollisionMonitor, GroundProjector, actor_state, chase_transform,
    configure_tm_actor, default_run_output, place_from_point, spawn_track_actor)
from carla_reconstruction.closed_loop.metrics import SafetyMetrics  # noqa: E402
from carla_reconstruction.closed_loop.safety_variants import (  # noqa: E402
    carla_behavior_for_track, load_carla_variant, validate_carla_seed)
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    DEFAULT_MINIMUM_TRACK_DISTANCE_M, DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
    classify_vehicle_motion, load_manifest_tracks, point_at)
from carla_reconstruction.scene_overrides.tm_ego_route_0103 import (  # noqa: E402
    prepare_ego_route)
from carla_reconstruction.closed_loop.carla_prediction import (  # noqa: E402
    CarlaPredictionObserver, add_prediction_arguments)
from carla_reconstruction.closed_loop.environment_scenarios import (  # noqa: E402
    EnvironmentScenario, add_environment_arguments, validate_environment_arguments)
from carla_reconstruction.visualization.tm_capture import (  # noqa: E402
    TMSensorCapture, add_capture_arguments, capture_enabled, validate_capture_arguments)


def _resolve_ego_mode(requested, variant):
    required = variant.get("ego_mode") if variant is not None else None
    if required is not None and requested is not None and requested != required:
        raise ValueError(
            "this CARLA perturbation experiment requires --ego-mode tm "
            "for both the baseline and variants")
    return requested or required or "replay"


def run(args):
    validate_environment_arguments(args)
    variant_path = getattr(args, "variant_config", None)
    variant_path = os.path.abspath(variant_path) if variant_path else None
    behavior_variant = (
        load_carla_variant(variant_path) if variant_path else None)
    ego_mode = _resolve_ego_mode(getattr(args, "ego_mode", None), behavior_variant)
    if capture_enabled(args) and ego_mode != "tm":
        raise ValueError("TM sensor visualization requires --ego-mode tm")
    requested_manifest = getattr(args, "manifest", None)
    if requested_manifest:
        requested_manifest = os.path.abspath(requested_manifest)
    if behavior_variant is not None:
        variant_manifest = os.path.abspath(behavior_variant["manifest"])
        if (requested_manifest is not None and
                os.path.normcase(requested_manifest) !=
                os.path.normcase(variant_manifest)):
            raise ValueError(
                "--manifest does not match the manifest recorded by "
                "--variant-config")
        manifest_path = variant_manifest
    elif requested_manifest is not None:
        manifest_path = requested_manifest
    else:
        raise ValueError("either --manifest or --variant-config is required")
    manifest, bundle = load_manifest_tracks(manifest_path)
    selection = behavior_variant.get("selection", {}) if behavior_variant else {}
    minimum_track_distance = (
        float(args.minimum_track_distance)
        if getattr(args, "minimum_track_distance", None) is not None else
        float(selection.get(
            "minimum_track_distance_m", DEFAULT_MINIMUM_TRACK_DISTANCE_M)))
    minimum_two_point_speed = (
        float(args.minimum_two_point_speed)
        if getattr(args, "minimum_two_point_speed", None) is not None else
        float(selection.get(
            "minimum_two_point_speed_mps",
            DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS)))
    simulation_seed = (
        int(args.seed) if getattr(args, "seed", None) is not None else
        int(behavior_variant["simulation_seed"])
        if behavior_variant is not None else 103)
    simulation_seed = validate_carla_seed(
        simulation_seed, "Traffic Manager --seed")
    map_name = args.map or manifest["map"]["runtime_name"]
    output = os.path.abspath(args.output) if args.output else default_run_output(
        REPO_ROOT, manifest["scene"], "tm")
    duration = args.duration if args.duration is not None else bundle.duration
    validate_capture_arguments(args, output, duration)

    # Resolve and validate the complete experiment population before touching
    # the CARLA world. This keeps invalid variant/CLI combinations side-effect
    # free (no world load, synchronous-mode change, or actor spawn).
    vehicle_tracks = list(bundle.vehicle_tracks.values())
    vehicle_tracks.sort(key=lambda track: (track.start_time, track.actor_id))
    if args.max_vehicles > 0:
        vehicle_tracks = vehicle_tracks[:args.max_vehicles]
    moving_track_ids = sorted(
        track.actor_id for track in vehicle_tracks
        if classify_vehicle_motion(
            track, minimum_track_distance,
            minimum_two_point_speed) == "moving")
    if behavior_variant is not None:
        expected_retained_ids = set(
            selection["retained_vehicle_track_ids"])
        effective_retained_ids = {
            track.actor_id for track in vehicle_tracks}
        if expected_retained_ids != effective_retained_ids:
            raise ValueError(
                "CARLA variant must retain every recorded surrounding "
                "vehicle; missing=%s extra=%s. Do not combine a generated "
                "variant with --max-vehicles or a changed manifest." %
                (sorted(expected_retained_ids - effective_retained_ids),
                 sorted(effective_retained_ids - expected_retained_ids)))
        expected_variant_ids = set(selection.get(
            "moving_surrounding_track_ids", selection["eligible_track_ids"]))
        effective_variant_ids = set(moving_track_ids)
        if expected_variant_ids != effective_variant_ids:
            raise ValueError(
                "CARLA variant must preserve every moving surrounding vehicle; "
                "missing=%s extra=%s. Do not combine a generated variant with "
                "a different --max-vehicles or motion threshold." %
                (sorted(expected_variant_ids - effective_variant_ids),
                 sorted(effective_variant_ids - expected_variant_ids)))
    pedestrian_tracks = ([track for track in bundle.actors.values()
                          if not track.is_vehicle]
                         if args.replay_pedestrians else [])

    prediction_observer = CarlaPredictionObserver(
        args, manifest_path, moving_track_ids,
        sorted({track.actor_id for track in vehicle_tracks} - set(moving_track_ids)))

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world() if args.reuse_world else client.load_world(map_name)
    carla_map = world.get_map()
    # A scene-local exception, validated against the actual loaded map before
    # changing world settings or spawning actors. Other scenes/modes are no-ops.
    ego_route = prepare_ego_route(
        manifest, bundle.ego, carla_map, ego_mode,
        disabled=getattr(args, "disable_0103_ego_route", False))
    projector = GroundProjector(carla_map)
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = args.step_length
    settings.no_rendering_mode = args.no_rendering
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_random_device_seed(simulation_seed)
    traffic_manager.set_global_distance_to_leading_vehicle(args.leading_distance)
    traffic_manager.set_osm_mode(True)
    traffic_manager.set_hybrid_physics_mode(False)

    spawned = {}
    offsets = {}
    authority = {}
    variant_applied = {}
    tm_behavior_applied = {}
    metrics = SafetyMetrics()
    clock = [0.0]
    collision_monitor = None
    environment = EnvironmentScenario(args, world)
    sensor_capture = None
    spectator = world.get_spectator()

    def configure_track(actor, track, sim_time):
        applied = configure_tm_actor(
            traffic_manager, actor, track, args.tm_port,
            args.path_spacing, args.leading_distance, args.auto_lane_change,
            args.minimum_speed_kmh,
            behavior_variant=carla_behavior_for_track(
                behavior_variant, track.actor_id))
        targeted = track.actor_id in selection.get("eligible_track_ids", [])
        record = {
            "carla_actor_id": int(actor.id),
            "behavior": applied,
            "applied_at_s": float(sim_time),
            "profile_role": "target" if targeted else "baseline",
        }
        tm_behavior_applied[track.actor_id] = record
        if targeted:
            variant_applied[track.actor_id] = record

    # Keep every recorded vehicle in the CARLA-only baseline. A vehicle with a
    # single observation has no defensible route, so it is rendered as fixed;
    # only positively classified movers receive Traffic Manager behavior.
    waiting = {track.actor_id: track for track in vehicle_tracks + pedestrian_tracks}
    live_tracks = {}

    def spawn_due(sim_time):
        for actor_id, track in list(waiting.items()):
            if track.start_time > sim_time + 1.0e-9:
                continue
            moving = (
                classify_vehicle_motion(
                    track, minimum_track_distance,
                    minimum_two_point_speed) == "moving")
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
                configure_track(actor, track, sim_time)
            else:
                authority[actor_id] = ("replay_pedestrian" if not track.is_vehicle
                                       else "static_recorded")

    print("CARLA server:", client.get_server_version())
    print("Map:", carla_map.name)
    print("Ego authority:", "carla_%s" % ego_mode)
    print("Recorded surrounding vehicles retained:", len(vehicle_tracks))
    print("Moving Traffic Manager vehicles:", len(moving_track_ids))
    if behavior_variant is not None:
        print("CARLA behavior experiment: %s; scope=%s; %d target(s)" % (
            behavior_variant["variant_id"], selection["scope"],
            len(selection["eligible_track_ids"])))
        print("Perturbation profile: %s; kind=%s" % (
            behavior_variant.get("aggression_profile", "legacy"), behavior_variant.get("kind", "legacy")))
        if behavior_variant.get("kind") == "sample" and any(
                (carla_behavior_for_track(behavior_variant, actor_id) or {}).get(key, 0.0) > 0.0
                for actor_id in selection["eligible_track_ids"]
                for key in ("ignore_vehicles_percentage", "ignore_lights_percentage", "ignore_signs_percentage")):
            print("Selected drivers intentionally relax TM vehicle/rule checks; physical collisions remain enabled.")
    print("Duration %.2fs at %.3fs/tick" % (duration, args.step_length))

    termination_reason = "duration_reached"
    run_error = None
    capture_close_error = None
    try:
        environment.apply_weather()
        ego_physics = ego_mode != "replay"
        ego, ego_offset = spawn_track_actor(
            world, bundle.ego, projector, "hero", physics=ego_physics)
        if ego is None:
            raise RuntimeError("could not spawn ego vehicle")
        spawned["ego"] = ego
        offsets["ego"] = ego_offset
        authority["ego"] = "carla_%s" % ego_mode
        if ego_mode == "tm":
            configure_track(ego, bundle.ego, 0.0)
            if ego_route is not None:
                # Replace only route coordinates once, before the first tick.
                # Keep all baseline/variant speed and safety settings intact.
                traffic_manager.set_path(ego, list(ego_route.locations), True)
                ego_route.metadata["applied"] = True
                print("Ego route correction:", ego_route.metadata["override_id"])
        collision_monitor = CollisionMonitor(world, ego, metrics, lambda: clock[0])
        # Retain the recorded population before adding a synthetic road obstacle.
        if getattr(args, "obstacle_distance", None) is not None:
            spawn_due(0.0)
            first = bundle.ego.points[0]
            obstacle = environment.spawn_obstacle(
                carla.Location(x=first.x, y=-first.y, z=projector.z(first.x, -first.y)),
                carla_map)
            spawned["scenario_obstacle"] = obstacle
            authority["scenario_obstacle"] = "synthetic_static_obstacle"
            print("Lane obstacle:", environment.metadata["obstacle"])
        if capture_enabled(args):
            sensor_capture = TMSensorCapture(
                args, manifest, manifest_path, output, duration, environment)
            sensor_capture.start(world, ego)
        prediction_observer.start(args.step_length)
        while clock[0] <= duration + 1.0e-9:
            wall_start = time.monotonic()
            sim_time = clock[0]
            spawn_due(sim_time)
            if ego_mode == "replay":
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

            frame_id = world.tick()
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
            ego_state = actor_state(ego)
            if sensor_capture is not None:
                sensor_capture.observe(frame_id, sim_time + args.step_length, ego, world)
            metrics.update(sim_time, ego_state, others)
            prediction_observer.observe(
                sim_time, ego_state, others, ego, spawned, world)
            clock[0] += args.step_length
            prediction_observer.pace(wall_start)
        if sensor_capture is not None and termination_reason == "duration_reached":
            sensor_capture.finish()
    except KeyboardInterrupt:
        termination_reason = "interrupted"
        raise
    except Exception as exc:
        termination_reason = "error"
        run_error = str(exc)
        raise
    finally:
        prediction_observer.close()
        if sensor_capture is not None:
            try:
                sensor_capture.close(termination_reason, run_error)
            except Exception as exc:
                # A failed sensor/index cleanup must not bypass vehicle,
                # Traffic Manager, weather, or synchronous-world cleanup.
                capture_close_error = exc
                sensor_capture.audit["complete"] = False
                termination_reason = "error"
                run_error = run_error or str(exc)
        run_metadata = {
            "mode": "traffic_manager",
            "manifest": manifest_path,
            "map": map_name,
            "seed": simulation_seed,
            "step_length": args.step_length,
            "duration": duration,
            "termination_reason": termination_reason,
            "error": run_error,
            "ego_mode": ego_mode,
            "scene_ego_route": ego_route.metadata if ego_route is not None else None,
            "prediction_risk": prediction_observer.metadata(),
            "actor_authority": authority,
            "environment_scenario": environment.metadata,
            "sensor_capture": ({"index": str(sensor_capture.root / "capture_index.json"),
                                "complete": sensor_capture.audit["complete"],
                                "frame_count": len(sensor_capture.audit["frames"])}
                               if sensor_capture is not None else None),
            "carla_behavior_variant": {
                "enabled": behavior_variant is not None,
                "config": variant_path,
                "configuration": behavior_variant,
                "scope": selection.get("scope"),
                "ego_mode": ego_mode,
                "eligible_track_ids": (
                    list(selection.get("eligible_track_ids", []))
                    if behavior_variant is not None else []),
                "applied_track_ids": sorted(variant_applied),
                "unapplied_track_ids": (
                    sorted(set(selection.get("eligible_track_ids", [])) -
                           set(variant_applied))
                    if behavior_variant is not None else []),
                "applied": dict(sorted(variant_applied.items())),
                "all_tm_actor_settings": dict(sorted(tm_behavior_applied.items())),
            },
        }
        try:
            csv_path, summary_path = metrics.write(output, run_metadata)
            with open(os.path.join(output, "run_config.json"), "w", encoding="utf-8") as stream:
                json.dump(run_metadata, stream, indent=2, sort_keys=False)
                stream.write("\n")
            print("Metrics:", csv_path)
            print("Summary:", summary_path)
        finally:
            try:
                if collision_monitor:
                    collision_monitor.destroy()
            finally:
                for actor in list(spawned.values()):
                    try:
                        if actor.type_id.startswith("vehicle."):
                            actor.set_autopilot(False, args.tm_port)
                        actor.destroy()
                    except Exception:
                        pass
                try:
                    traffic_manager.set_synchronous_mode(False)
                finally:
                    try:
                        environment.close()
                    finally:
                        world.apply_settings(original_settings)

    if capture_close_error is not None:
        raise RuntimeError("sensor capture cleanup failed") from capture_close_error
    if getattr(args, "build_sim_panels", False):
        from carla_reconstruction.visualization.sim_panels import build_sim_panels
        build_sim_panels(os.path.join(output, "capture"), os.path.join(output, "sim_panels"))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest",
                        help="scene manifest (optional when --variant-config supplies it)")
    parser.add_argument(
        "--variant-config",
        help="CARLA behavior baseline/variant generated by generate_carla_safety_variants.py")
    parser.add_argument("--map", help="CARLA map path; defaults to manifest runtime_name")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--step-length", type=float, default=0.05)
    parser.add_argument("--duration", type=float)
    parser.add_argument(
        "--seed", type=int,
        help="Traffic Manager seed override; defaults to variant seed or 103")
    parser.add_argument("--ego-mode", choices=("replay", "tm", "external"),
                        help="defaults to tm for new perturbation configs, "
                             "otherwise replay")
    parser.add_argument(
        "--disable-0103-ego-route", action="store_true",
        help="disable the scene-0103-only TM ego exit-lane correction "
             "and use the original raw recorded path (diagnostic rollback)")
    parser.add_argument("--max-vehicles", type=int, default=0,
                        help="maximum moving/static surrounding vehicles; 0 means all")
    parser.add_argument(
        "--minimum-track-distance", type=float,
        default=None,
        help="shorter vehicle tracks remain static instead of entering TM")
    parser.add_argument(
        "--minimum-two-point-speed", type=float,
        default=None,
        help="speed that identifies a moving vehicle from exactly two observations")
    parser.add_argument("--path-spacing", type=float, default=2.0)
    parser.add_argument("--leading-distance", type=float, default=2.5)
    parser.add_argument("--minimum-speed-kmh", type=float, default=3.0)
    parser.add_argument("--auto-lane-change", action="store_true")
    parser.add_argument("--replay-pedestrians", action="store_true")
    parser.add_argument("--despawn-at-track-end", action="store_true")
    parser.add_argument("--reuse-world", action="store_true")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--output")
    add_prediction_arguments(parser)
    add_environment_arguments(parser)
    add_capture_arguments(parser)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not args.manifest and not args.variant_config:
        parser.error("either --manifest or --variant-config is required")
    run(args)


if __name__ == "__main__":
    main()
