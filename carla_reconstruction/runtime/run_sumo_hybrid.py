#!/usr/bin/env python3
"""Run SUMO background traffic with CARLA-authority ego/critical vehicles."""

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
    CollisionMonitor, GroundProjector, actor_state, carla_path, chase_transform,
    configure_tm_actor, default_run_output, place_from_point, spawn_track_actor)
from carla_reconstruction.closed_loop.controllers import ReferencePathController  # noqa: E402
from carla_reconstruction.closed_loop.metrics import SafetyMetrics  # noqa: E402
from carla_reconstruction.closed_loop.tracks import load_manifest_tracks, point_at  # noqa: E402


def _load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _find_sumo_home(explicit):
    candidates = [explicit, os.environ.get("SUMO_HOME"),
                  r"C:\Program Files (x86)\Eclipse\Sumo",
                  r"C:\Program Files\Eclipse\Sumo"]
    for candidate in candidates:
        if candidate and os.path.isdir(os.path.join(candidate, "tools")):
            return os.path.abspath(candidate)
    raise RuntimeError("SUMO_HOME does not identify an installation containing a tools folder")


def _load_cosimulation(carla_root, sumo_home):
    os.environ["SUMO_HOME"] = sumo_home
    tools = os.path.join(sumo_home, "tools")
    integration = os.path.join(carla_root, "Co-Simulation", "Sumo")
    for path in (tools, integration):
        if path not in sys.path:
            sys.path.insert(0, path)
    if not os.path.isfile(os.path.join(integration, "run_synchronization.py")):
        raise FileNotFoundError("CARLA SUMO integration was not found under " + integration)
    from run_synchronization import SimulationSynchronization
    from sumo_integration.carla_simulation import CarlaSimulation
    from sumo_integration.sumo_simulation import SumoSimulation
    return SimulationSynchronization, CarlaSimulation, SumoSimulation


def _authority_from_mode(mode):
    return {
        "tm": "carla_tm",
        "reference": "carla_reference",
        "replay": "carla_replay",
        "external": "carla_external",
    }[mode]


def run(args):
    config_path = os.path.abspath(args.config)
    config = _load_json(config_path)
    manifest_path = os.path.abspath(config["manifest"])
    manifest, bundle = load_manifest_tracks(manifest_path)
    tracks = dict(bundle.actors)
    tracks["ego"] = bundle.ego
    sumo_home = _find_sumo_home(args.sumo_home)
    SimulationSynchronization, CarlaSimulation, SumoSimulation = _load_cosimulation(
        os.path.abspath(args.carla_root), sumo_home)

    map_name = args.map or config["map"]
    step_length = args.step_length or float(config.get("step_length", 0.05))
    duration = args.duration if args.duration is not None else float(config["duration"])
    output = os.path.abspath(args.output) if args.output else default_run_output(
        REPO_ROOT, manifest["scene"], "sumo_hybrid")

    bootstrap_client = carla.Client(args.host, args.port)
    bootstrap_client.set_timeout(args.timeout)
    world = (bootstrap_client.get_world() if args.reuse_world
             else bootstrap_client.load_world(map_name))
    original_settings = world.get_settings()

    sumo_simulation = SumoSimulation(
        config["sumo"]["config"], step_length,
        host=args.sumo_host, port=args.sumo_port,
        sumo_gui=args.sumo_gui, client_order=args.client_order)
    carla_simulation = CarlaSimulation(args.host, args.port, step_length)
    synchronization = SimulationSynchronization(
        sumo_simulation, carla_simulation,
        tls_manager=args.tls_manager or config["sumo"].get("tls_manager", "none"),
        sync_vehicle_color=args.sync_vehicle_color,
        sync_vehicle_lights=args.sync_vehicle_lights)
    world = carla_simulation.world
    projector = GroundProjector(world.get_map())
    traffic_manager_settings = config.get("traffic_manager", {})
    tm_port = int(traffic_manager_settings.get("port", args.tm_port))
    traffic_manager = bootstrap_client.get_trafficmanager(tm_port)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_random_device_seed(int(config.get("seed", 103)))
    traffic_manager.set_osm_mode(True)
    traffic_manager.set_hybrid_physics_mode(False)

    ego_spec = dict(config.get("ego", {}))
    if args.ego_mode:
        ego_spec["authority"] = _authority_from_mode(args.ego_mode)
    ego_spec.setdefault("track_id", "ego")
    ego_spec.setdefault("authority", "carla_reference")
    specs = [ego_spec] + list(config.get("critical_actors", []))
    pending = {spec["track_id"]: spec for spec in specs}
    spawned = {}
    offsets = {}
    controllers = {}
    authority = {}
    metrics = SafetyMetrics()
    clock = [0.0]
    collision_monitor = None
    spectator = world.get_spectator()

    def spawn_spec(track_id, spec):
        track = tracks.get(track_id)
        if track is None:
            print("WARNING: configured CARLA-authority track is absent:", track_id)
            return
        actor_authority = spec.get("authority", "carla_reference")
        physics = actor_authority != "carla_replay"
        actor, offset = spawn_track_actor(
            world, track, projector,
            "hero" if track_id == "ego" else "critical_actor", physics=physics)
        if actor is None:
            print("WARNING: could not spawn CARLA-authority actor", track_id)
            return
        spawned[track_id], offsets[track_id] = actor, offset
        authority[track_id] = actor_authority
        if actor_authority == "carla_tm":
            configure_tm_actor(
                traffic_manager, actor, track, tm_port,
                leading_distance=float(traffic_manager_settings.get(
                    "leading_distance", 2.5)),
                auto_lane_change=bool(traffic_manager_settings.get(
                    "auto_lane_change", False)))
        elif actor_authority == "carla_reference":
            path = carla_path(track)
            if len(path) < 2:
                print("WARNING: reference track is too short:", track_id)
            else:
                controllers[track_id] = ReferencePathController(
                    path,
                    desired_speed=max(
                        0.5, track.mean_speed * float(spec.get("desired_speed_scale", 1.0))),
                    time_headway=float(spec.get("time_headway", 1.4)),
                    minimum_gap=float(spec.get("minimum_gap", 2.5)),
                    events=spec.get("events", []))

    def spawn_due(sim_time):
        nonlocal collision_monitor
        for track_id, spec in list(pending.items()):
            track = tracks.get(track_id)
            if track is None or track.start_time <= sim_time + 1.0e-9:
                del pending[track_id]
                spawn_spec(track_id, spec)
                if track_id == "ego" and track_id in spawned:
                    collision_monitor = CollisionMonitor(
                        world, spawned[track_id], metrics, lambda: clock[0])

    print("CARLA server:", bootstrap_client.get_server_version())
    print("Map:", world.get_map().name)
    print("SUMO config:", config["sumo"]["config"])
    print("Duration %.2fs at %.3fs/tick" % (duration, step_length))

    termination_reason = "duration_reached"
    try:
        while clock[0] <= duration + 1.0e-9:
            wall_start = time.time()
            sim_time = clock[0]
            spawn_due(sim_time)
            vehicles = list(world.get_actors().filter("vehicle.*"))
            for track_id, actor in list(spawned.items()):
                if not actor.is_alive:
                    continue
                track = tracks[track_id]
                actor_authority = authority[track_id]
                if actor_authority == "carla_replay":
                    place_from_point(actor, point_at(track, sim_time), projector,
                                     offsets[track_id])
                elif actor_authority == "carla_reference" and track_id in controllers:
                    actor.apply_control(controllers[track_id].run_step(
                        actor, vehicles, elapsed=sim_time - track.start_time))

            synchronization.tick()
            if "ego" in spawned:
                ego = spawned["ego"]
                if not ego.is_alive:
                    termination_reason = "ego_destroyed"
                    print("WARNING: ego actor was destroyed; ending this run")
                    break
                spectator.set_transform(chase_transform(ego))
                reverse_sumo = {
                    carla_id: "sumo:%s" % sumo_id
                    for sumo_id, carla_id in synchronization.sumo2carla_ids.items()
                }
                others = {}
                for actor in world.get_actors().filter("vehicle.*"):
                    if actor.id == ego.id:
                        continue
                    label = reverse_sumo.get(actor.id, "carla:%s" % actor.id)
                    others[label] = actor_state(actor)
                metrics.update(sim_time, actor_state(ego), others)

            clock[0] += step_length
            if args.realtime:
                elapsed = time.time() - wall_start
                if elapsed < step_length:
                    time.sleep(step_length - elapsed)
    finally:
        run_metadata = {
            "mode": "sumo_hybrid",
            "config": config_path,
            "manifest": manifest_path,
            "map": map_name,
            "sumo_config": config["sumo"]["config"],
            "step_length": step_length,
            "duration": duration,
            "termination_reason": termination_reason,
            "actor_authority": authority,
        }
        csv_path, summary_path = metrics.write(output, run_metadata)
        if collision_monitor:
            collision_monitor.destroy()
        for actor in list(spawned.values()):
            try:
                if actor.type_id.startswith("vehicle."):
                    actor.set_autopilot(False, tm_port)
                actor.destroy()
            except Exception:
                pass
        synchronization.close()
        try:
            traffic_manager.set_synchronous_mode(False)
            world.apply_settings(original_settings)
        except Exception:
            pass
        print("Metrics:", csv_path)
        print("Summary:", summary_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True,
                        help="hybrid_config.json produced by prepare_sumo.py")
    parser.add_argument("--carla-root", default=r"C:\carla")
    parser.add_argument("--sumo-home")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--map")
    parser.add_argument("--step-length", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--ego-mode", choices=("tm", "reference", "replay", "external"))
    parser.add_argument("--tls-manager", choices=("none", "carla", "sumo"))
    parser.add_argument("--sumo-host")
    parser.add_argument("--sumo-port", type=int)
    parser.add_argument("--sumo-gui", action="store_true")
    parser.add_argument("--client-order", type=int, default=1)
    parser.add_argument("--sync-vehicle-color", action="store_true")
    parser.add_argument("--sync-vehicle-lights", action="store_true")
    parser.add_argument("--reuse-world", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--output")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
