#!/usr/bin/env python3
"""Run SUMO moving traffic with CARLA-authority ego, critical, and static vehicles."""

import argparse
import json
import math
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
from carla_reconstruction.closed_loop.hybrid_authority import (  # noqa: E402
    STATIC_AUTHORITY, actor_spawn_policy, configured_carla_actor_specs)
from carla_reconstruction.closed_loop.metrics import SafetyMetrics  # noqa: E402
from carla_reconstruction.closed_loop.sumo_runtime import (  # noqa: E402
    SumoRouteContinuator)
from carla_reconstruction.closed_loop.tracks import (  # noqa: E402
    load_manifest_tracks, point_at, recorded_speed_at)


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


def _sumo_front_bumper_xy(pose, vehicle_length):
    """Convert a recorded center pose to SUMO's front-bumper reference."""
    reference_point = pose.get("reference_point", "vehicle_center")
    x = float(pose["x"])
    y = float(pose["y"])
    if reference_point == "front_center_bumper":
        return x, y
    if reference_point != "vehicle_center":
        raise ValueError(
            "unsupported initial_sumo_pose reference_point: %s" %
            reference_point)
    heading = math.radians(90.0 - float(pose["angle_degrees"]))
    half_length = 0.5 * float(vehicle_length)
    return (
        x + math.cos(heading) * half_length,
        y + math.sin(heading) * half_length,
    )


def _prediction_risk_actor_states(vehicle_rows, static_carla_actor_ids):
    """Return non-static vehicle states for prediction and dashboard risk.

    ``carla_static`` is a recorded ownership/classification decision, rather
    than an instantaneous speed test.  This keeps dynamic vehicles eligible
    while they wait at a signal or otherwise stop temporarily.
    """
    static_carla_actor_ids = set(static_carla_actor_ids)
    return {
        label: state
        for carla_actor_id, label, state in vehicle_rows
        if carla_actor_id not in static_carla_actor_ids
    }


def _detach_carla_static_sumo_proxies(
        synchronization, static_carla_actor_ids):
    """Remove SUMO shadows of CARLA-owned recorded parked vehicles.

    CARLA keeps rendering the original actors at their recorded roadside
    poses. The stock bridge otherwise maps every CARLA vehicle onto a SUMO
    lane with ``moveToXY``; a parked off-lane actor can then become a false
    car-following leader and hold real SUMO movers at zero speed. Removing the
    bridge mapping after a successful SUMO removal prevents respawn on later
    ticks while leaving the CARLA actor untouched.
    """
    detached = {}
    failures = {}
    mapping = synchronization.carla2sumo_ids
    for raw_carla_actor_id in sorted(set(static_carla_actor_ids)):
        carla_actor_id = int(raw_carla_actor_id)
        sumo_actor_id = mapping.get(carla_actor_id)
        if sumo_actor_id is None:
            continue
        unsubscribed = False
        try:
            synchronization.sumo.unsubscribe(sumo_actor_id)
            unsubscribed = True
            synchronization.sumo.destroy_actor(sumo_actor_id)
        except Exception as exc:
            restore_error = None
            if unsubscribed:
                try:
                    synchronization.sumo.subscribe(sumo_actor_id)
                except Exception as restore_exc:
                    restore_error = str(restore_exc)
            failures[str(carla_actor_id)] = (
                str(exc) if restore_error is None else
                "%s; subscription restore failed: %s" %
                (exc, restore_error))
            continue
        mapping.pop(carla_actor_id, None)
        detached[str(carla_actor_id)] = str(sumo_actor_id)
    return detached, failures


def _install_carla_to_sumo_spawn_exclusion(
        synchronization, excluded_carla_actor_ids):
    """Prevent selected CARLA actors from being announced to SUMO.

    The stock synchronizer creates SUMO shadows from
    ``carla.spawned_actors`` immediately after ``carla.tick``. Filtering that
    one-tick notification is sufficient: the underlying CARLA actor remains
    active and visible, while the bridge never creates an in-lane proxy.
    ``excluded_carla_actor_ids`` is a mutable set so actors spawned later in
    the scenario are covered before their first synchronization tick.
    """
    original_tick = synchronization.carla.tick
    state = {
        "filtered_carla_actor_ids": set(),
    }

    def tick_without_excluded_spawn_notifications():
        original_tick()
        filtered = set(synchronization.carla.spawned_actors).intersection(
            excluded_carla_actor_ids)
        if filtered:
            synchronization.carla.spawned_actors.difference_update(filtered)
            state["filtered_carla_actor_ids"].update(filtered)

    synchronization.carla.tick = tick_without_excluded_spawn_notifications
    return state


def _terminal_stationary_tail_start(
        track, maximum_extent=1.0, minimum_duration=2.0):
    """Return the earliest persistent stationary-tail time, if one exists.

    This uses the remaining spatial extent instead of individual segment
    speeds, which are noisy for annotated parked vehicles. A temporary stop in
    the middle of a track is not a terminal tail because later observations
    leave the allowed radius.
    """
    maximum_extent = float(maximum_extent)
    minimum_duration = float(minimum_duration)
    if (not math.isfinite(maximum_extent) or maximum_extent <= 0.0 or
            not math.isfinite(minimum_duration) or minimum_duration <= 0.0):
        raise ValueError(
            "terminal stationary-tail extent and duration must be positive")
    points = list(track.points)
    if len(points) < 2:
        return None
    final_time = float(points[-1].time)
    extent = 0.0
    earliest = None
    for index in range(len(points) - 2, -1, -1):
        point = points[index]
        for right in points[index + 1:]:
            extent = max(
                extent,
                math.hypot(right.x - point.x, right.y - point.y))
        # Adding earlier observations cannot reduce the tail extent, so no
        # still-earlier point can qualify after this boundary is crossed.
        if extent > maximum_extent + 1.0e-9:
            break
        if final_time - float(point.time) + 1.0e-9 >= minimum_duration:
            earliest = float(point.time)
    return earliest


def _ordered_runtime_route_events(continuator, vehicle_id=None):
    """Merge runtime route mutations in deterministic chronological order."""
    decorated = []
    # The runtime invokes terminal/stall recovery before ordinary continuation
    # at a given simulation time. Preserve that ordering when no shared event
    # sequence is available, then preserve each producer's append order.
    event_streams = (
        getattr(continuator, "terminal_release_events", ()),
        getattr(continuator, "stall_recovery_events", ()),
        getattr(continuator, "events", ()),
    )
    for stream_index, events in enumerate(event_streams):
        for event_index, event in enumerate(events or ()):
            if (vehicle_id is not None and
                    event.get("vehicle_id") != vehicle_id):
                continue
            event_time = event.get("simulation_time_s")
            try:
                event_time = float(event_time)
            except (TypeError, ValueError, OverflowError):
                event_time = None
            if event_time is not None and not math.isfinite(event_time):
                event_time = None
            sequence = event.get("sequence")
            try:
                sequence = int(sequence)
            except (TypeError, ValueError, OverflowError):
                sequence = None
            decorated.append((
                event_time is None,
                0.0 if event_time is None else event_time,
                sequence is None,
                0 if sequence is None else sequence,
                stream_index,
                event_index,
                event,
            ))
    decorated.sort(key=lambda item: item[:-1])
    return [item[-1] for item in decorated]


def _install_sumo_route_continuation(
        sumo_simulation, continuator, mover_fidelity):
    """Run route-tail repair immediately before and after each SUMO tick."""
    import traci

    original_tick = sumo_simulation.tick

    def extend_and_report():
        now = float(traci.simulation.getTime())
        # Observe every eligible mover. Active recorded speed authority is
        # released first after a guarded persistent lane stop; route intent is
        # changed only if autonomous SUMO then remains stopped.
        if mover_fidelity["speed_policy"] == "recorded_profile":
            speed_authority_ids = set(
                mover_fidelity.get("_active_profile_ids", ()))
            autonomous_ids = (
                set(continuator.eligible_vehicle_ids) - speed_authority_ids)
        else:
            speed_authority_ids = set()
            autonomous_ids = set(continuator.eligible_vehicle_ids)
        for event in continuator.recover_persistent_stops(
                autonomous_ids, now,
                speed_authority_vehicle_ids=speed_authority_ids,
                release_speed_authority=mover_fidelity.get(
                    "_release_profile")):
            if event["mutation_type"] == "persistent_stop_suffix_recovery":
                print("SUMO mover %s recovered from a persistent stop while "
                      "preserving %s -> %s; selected a new viable suffix "
                      "ending at %s at %.3f s" % (
                          event["vehicle_id"], event["from_edge"],
                          event["preserved_next_edge"], event["to_edge"],
                          now))
            else:
                print("SUMO mover %s recovered from a persistent stop: "
                      "planned %s -> %s, selected viable alternative %s "
                      "with continuation ending at %s at %.3f s" % (
                          event["vehicle_id"], event["from_edge"],
                          event["original_next_edge"],
                          event.get("selected_next_edge", event["to_edge"]),
                          event["to_edge"], now))
        for event in continuator.extend_active_routes(now):
            print("SUMO mover %s extended route %s -> %s at %.3f s" % (
                event["vehicle_id"], event["from_edge"], event["to_edge"],
                now))

    def tick_with_route_continuation():
        extend_and_report()
        original_tick()
        extend_and_report()

    sumo_simulation.tick = tick_with_route_continuation


def _apply_reference_track_control(
        actor, controller, track, other_vehicles, sim_time):
    """Follow a finite reference track, then approach and hold its endpoint.

    The recorded end time starts a spatial terminal approach rather than an
    immediate stop. This lets a lagging physics actor reach its recorded final
    position. The controller slows near that position and latches a
    straight full-brake hold when it reaches or passes the endpoint, preventing
    a pure-pursuit orbit. Returning ``True`` identifies a hold tick.
    """
    elapsed = max(0.0, sim_time - track.start_time)
    if controller is None:
        if sim_time + 1.0e-9 < track.end_time:
            return False
        actor.apply_control(carla.VehicleControl(
            throttle=0.0, steer=0.0, brake=1.0))
        return True
    if sim_time + 1.0e-9 >= track.end_time:
        control, held = controller.run_step_to_endpoint(
            actor, other_vehicles, elapsed=elapsed)
        actor.apply_control(control)
        return held
    actor.apply_control(controller.run_step(
        actor, other_vehicles, elapsed=elapsed))
    return False


def _install_sumo_mover_fidelity(
        sumo_simulation, route_rows, tracks, position_policy, speed_policy,
        step_length, maximum_snap_distance,
        terminal_stop_policy="release_to_sumo",
        terminal_stop_maximum_extent=1.0,
        terminal_stop_minimum_duration=2.0):
    """Seed recorded mover pose/speed immediately before CARLA mirroring."""
    state = {
        "position_policy": position_policy,
        "speed_policy": speed_policy,
        "configured": 0,
        "pose_applied": set(),
        "profile_started": set(),
        "profile_released": set(),
        "pose_applied_at_s": {},
        "profile_started_at_s": {},
        "profile_released_at_s": {},
        "profile_release_reason": {},
        "terminal_stop_policy": terminal_stop_policy,
        "terminal_stop_maximum_extent_m": float(
            terminal_stop_maximum_extent),
        "terminal_stop_minimum_duration_s": float(
            terminal_stop_minimum_duration),
        "terminal_release_track_time_s": {},
        "failures": {},
        # Runtime-only coordination with guarded stop recovery. These private
        # entries are deliberately omitted from the JSON summary.
        "_active_profile_ids": set(),
        "_release_profile": None,
    }
    if position_policy not in ("recorded", "route"):
        raise ValueError(
            "background.moving_initialization.position_policy must be "
            "recorded or route")
    if speed_policy not in (
            "recorded_profile", "recorded_mean", "unbounded"):
        raise ValueError(
            "background.moving_speed.policy is not supported: %s" %
            speed_policy)
    if terminal_stop_policy not in ("release_to_sumo", "preserve_recorded"):
        raise ValueError(
            "background.moving_speed.terminal_stop_policy must be "
            "release_to_sumo or preserve_recorded")
    # Validate these even when terminal release is disabled so malformed
    # generated or hand-edited configuration fails predictably.
    terminal_stop_maximum_extent = float(terminal_stop_maximum_extent)
    terminal_stop_minimum_duration = float(terminal_stop_minimum_duration)
    if (not math.isfinite(terminal_stop_maximum_extent) or
            terminal_stop_maximum_extent <= 0.0 or
            not math.isfinite(terminal_stop_minimum_duration) or
            terminal_stop_minimum_duration <= 0.0):
        raise ValueError(
            "terminal stationary-tail extent and duration must be positive")

    specs = {}
    for sumo_id, row in route_rows.items():
        track = tracks.get(str(row.get("id", "")))
        if track is None:
            continue
        pose = row.get("initial_sumo_pose")
        if position_policy == "recorded" and not isinstance(pose, dict):
            state["failures"][sumo_id] = (
                "route report has no initial_sumo_pose")
            continue
        terminal_release_time = None
        if (speed_policy == "recorded_profile" and
                terminal_stop_policy == "release_to_sumo"):
            terminal_release_time = _terminal_stationary_tail_start(
                track,
                maximum_extent=terminal_stop_maximum_extent,
                minimum_duration=terminal_stop_minimum_duration)
            if terminal_release_time is not None:
                state["terminal_release_track_time_s"][sumo_id] = (
                    terminal_release_time)
        specs[sumo_id] = {
            "track": track,
            "first_matched_time": float(row.get(
                "first_matched_time", track.start_time)),
            "pose": pose,
            "maximum_speed": row.get("sumo_max_speed_mps"),
            "vehicle_length": row.get("sumo_vehicle_length_m"),
            "terminal_release_time": terminal_release_time,
        }
    state["configured"] = len(specs)
    if not specs or (position_policy == "route" and
                     speed_policy != "recorded_profile"):
        return state

    import traci

    original_tick = sumo_simulation.tick
    initialized = set()
    initialization_attempts = {}
    profile_start_times = {}
    restore_default_speed_mode_at = {}
    release_one_shot_speed_at = {}

    def apply_target_speed(sumo_id, target_speed):
        if target_speed is None:
            traci.vehicle.setSpeed(sumo_id, -1.0)
            return
        traci.vehicle.setSpeed(sumo_id, max(0.0, float(target_speed)))

    def release_profile(sumo_id, now, reason):
        if (sumo_id in state["profile_released"] and
                sumo_id not in state["_active_profile_ids"]):
            return False
        apply_target_speed(sumo_id, None)
        profile_start_times.pop(sumo_id, None)
        state["_active_profile_ids"].discard(sumo_id)
        if sumo_id not in state["profile_released"]:
            state["profile_released"].add(sumo_id)
            state["profile_released_at_s"][sumo_id] = now
            state["profile_release_reason"][sumo_id] = reason
            if reason == "terminal_recorded_stationary_tail":
                print("SUMO mover %s reached a terminal recorded stop at "
                      "%.3f s; releasing speed replay to autonomous SUMO" %
                      (sumo_id, now))
            elif reason in (
                    "persistent_unexplained_lane_stop",
                    "persistent_downstream_short_edge_blocker"):
                print("SUMO mover %s remained stopped without a signal, stop "
                      "command, or close leader at %.3f s; releasing recorded "
                      "speed replay to autonomous SUMO" % (sumo_id, now))
        return True

    state["_release_profile"] = release_profile

    def tick_with_recorded_initialization():
        now = float(traci.simulation.getTime())
        active_ids = set(traci.vehicle.getIDList())
        for sumo_id, restore_time in list(
                restore_default_speed_mode_at.items()):
            if sumo_id not in active_ids or now + 1.0e-9 < restore_time:
                continue
            traci.vehicle.setSpeedMode(sumo_id, 31)
            del restore_default_speed_mode_at[sumo_id]
        for sumo_id, release_time in list(release_one_shot_speed_at.items()):
            if sumo_id not in active_ids or now + 1.0e-9 < release_time:
                continue
            traci.vehicle.setSpeed(sumo_id, -1.0)
            del release_one_shot_speed_at[sumo_id]

        if speed_policy == "recorded_profile":
            for sumo_id, start_time in list(profile_start_times.items()):
                if sumo_id not in active_ids:
                    continue
                spec = specs[sumo_id]
                track_time = (
                    spec["first_matched_time"] + max(0.0, now - start_time))
                terminal_release_time = spec["terminal_release_time"]
                if (terminal_release_time is not None and
                        track_time + 1.0e-9 >= terminal_release_time):
                    release_profile(
                        sumo_id, now, "terminal_recorded_stationary_tail")
                    continue
                target_speed = recorded_speed_at(spec["track"], track_time)
                if target_speed is None:
                    release_profile(sumo_id, now, "recorded_track_ended")
                    continue
                apply_target_speed(sumo_id, target_speed)

        original_tick()

        now = float(traci.simulation.getTime())
        active_ids = set(traci.vehicle.getIDList())
        pending_ids = set()
        get_pending = getattr(traci.simulation, "getPendingVehicles", None)
        if get_pending is not None:
            pending_ids = set(get_pending())
        if position_policy == "recorded":
            candidates = (active_ids | pending_ids).intersection(specs)
        else:
            candidates = active_ids.intersection(specs)

        for sumo_id in sorted(candidates - initialized):
            attempts = initialization_attempts.get(sumo_id, 0) + 1
            initialization_attempts[sumo_id] = attempts
            spec = specs[sumo_id]
            target_speed = recorded_speed_at(
                spec["track"], spec["first_matched_time"])
            try:
                if speed_policy == "recorded_profile":
                    maximum_speed = spec["maximum_speed"]
                    if maximum_speed is None:
                        maximum_speed = spec["track"].peak_speed
                    traci.vehicle.setMaxSpeed(
                        sumo_id, max(0.1, float(maximum_speed)))

                # Keep SUMO's safe-speed, braking, signal, and right-of-way
                # checks, but waive only the acceleration bound while seeding
                # the recorded initial velocity. Default mode 31 is restored
                # two ticks later.
                traci.vehicle.setSpeedMode(sumo_id, 29)
                if target_speed is not None:
                    capped_speed = float(target_speed)
                    maximum_speed = spec["maximum_speed"]
                    if maximum_speed is not None:
                        capped_speed = min(
                            capped_speed, float(maximum_speed))
                    traci.vehicle.setPreviousSpeed(sumo_id, capped_speed)
                    apply_target_speed(sumo_id, capped_speed)

                if position_policy == "recorded":
                    pose = spec["pose"]
                    vehicle_length = spec["vehicle_length"]
                    if vehicle_length is None:
                        # Backward compatibility for route reports generated
                        # before sumo_vehicle_length_m was recorded.
                        vehicle_length = traci.vehicle.getLength(sumo_id)
                    front_x, front_y = _sumo_front_bumper_xy(
                        pose, vehicle_length)
                    traci.vehicle.moveToXY(
                        sumo_id, "", 0,
                        front_x, front_y,
                        angle=float(pose["angle_degrees"]),
                        keepRoute=1,
                        matchThreshold=max(
                            10.0, float(maximum_snap_distance) + 1.0))
                    state["pose_applied"].add(sumo_id)
                    state["pose_applied_at_s"][sumo_id] = now

                initialized.add(sumo_id)
                state["failures"].pop(sumo_id, None)
                profile_start_times[sumo_id] = now
                restore_default_speed_mode_at[sumo_id] = (
                    now + 2.0 * step_length)
                if speed_policy == "recorded_profile":
                    state["profile_started"].add(sumo_id)
                    state["profile_started_at_s"][sumo_id] = now
                    state["_active_profile_ids"].add(sumo_id)
                else:
                    release_one_shot_speed_at[sumo_id] = (
                        now + 2.0 * step_length)
            except Exception as exc:
                state["failures"][sumo_id] = str(exc)
                if attempts >= 3:
                    initialized.add(sumo_id)
                    print("WARNING: could not initialize SUMO mover %s after "
                          "%d attempts: %s" % (sumo_id, attempts, exc))

    sumo_simulation.tick = tick_with_recorded_initialization
    return state


def _authority_from_mode(mode):
    return {
        "tm": "carla_tm",
        "reference": "carla_reference",
        "replay": "carla_replay",
        "external": "carla_external",
    }[mode]


def _stop_simulations(synchronization, carla_simulation, sumo_simulation,
                      traffic_manager, bootstrap_client, world,
                      original_settings):
    """Best-effort shutdown for normal and partially initialized runtimes."""
    close_error = None
    synchronization_closed = False
    if synchronization is not None:
        try:
            synchronization.close()
            synchronization_closed = True
        except Exception as exc:
            close_error = exc
    if not synchronization_closed:
        if carla_simulation is not None:
            try:
                carla_simulation.close()
            except Exception:
                pass
        if sumo_simulation is not None:
            try:
                sumo_simulation.close()
            except Exception:
                pass
    if traffic_manager is not None:
        try:
            traffic_manager.set_synchronous_mode(False)
        except Exception:
            pass
    if bootstrap_client is not None:
        try:
            # The official synchronization helper always enables the default
            # TM, even when the experiment uses a different configured port.
            bootstrap_client.get_trafficmanager().set_synchronous_mode(False)
        except Exception:
            pass
    if world is not None and original_settings is not None:
        try:
            world.apply_settings(original_settings)
        except Exception:
            pass
    return close_error


def _start_simulations(args, config, map_name, step_length,
                       SimulationSynchronization, CarlaSimulation,
                       SumoSimulation):
    """Create external runtime resources with rollback on partial startup."""
    bootstrap_client = None
    world = None
    original_settings = None
    sumo_simulation = None
    carla_simulation = None
    synchronization = None
    traffic_manager = None
    try:
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
            tls_manager=args.tls_manager or config["sumo"].get(
                "tls_manager", "none"),
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
        return (
            bootstrap_client, world, original_settings, sumo_simulation,
            carla_simulation, synchronization, projector,
            traffic_manager_settings, tm_port, traffic_manager)
    except BaseException:
        _stop_simulations(
            synchronization, carla_simulation, sumo_simulation,
            traffic_manager, bootstrap_client, world, original_settings)
        raise


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
    import traci

    map_name = args.map or config["map"]
    step_length = args.step_length or float(config.get("step_length", 0.05))
    duration = args.duration if args.duration is not None else float(config["duration"])
    output = os.path.abspath(args.output) if args.output else default_run_output(
        REPO_ROOT, manifest["scene"], "sumo_hybrid")

    expected_sumo_ids = set()
    route_rows = {}
    route_report = {}
    route_report_path = config.get("sumo", {}).get("route_report")
    if route_report_path:
        if not os.path.isabs(route_report_path):
            route_report_path = os.path.join(
                os.path.dirname(config_path), route_report_path)
        try:
            route_report = _load_json(route_report_path)
            expected_sumo_ids = {
                str(item["sumo_id"])
                for item in route_report.get("included", [])
                if item.get("sumo_id")
            }
            route_rows = {
                str(item["sumo_id"]): item
                for item in route_report.get("included", [])
                if item.get("sumo_id")
            }
        except Exception as exc:
            print("WARNING: moving-vehicle lifecycle report is unavailable:", exc)

    spawn_schedule = config.get("spawn_schedule")
    if spawn_schedule is None:
        # Configurations generated before staged startup support keep their
        # original recorded-time spawning and start ticking immediately.
        bridge_warmup_steps = 0
    else:
        if not isinstance(spawn_schedule, dict):
            raise ValueError("spawn_schedule in hybrid_config.json must be an object")
        raw_warmup_steps = spawn_schedule.get("bridge_warmup_steps", 0)
        if (isinstance(raw_warmup_steps, bool) or
                not isinstance(raw_warmup_steps, int) or
                raw_warmup_steps < 0):
            raise ValueError(
                "spawn_schedule.bridge_warmup_steps must be a non-negative integer")
        bridge_warmup_steps = raw_warmup_steps

    prediction_section = config.get("prediction_risk") or {}
    prediction_override = getattr(args, "prediction_risk", None)
    if prediction_override is None:
        prediction_requested = (
            bool(prediction_section.get("enabled", False))
            if isinstance(prediction_section, dict)
            else bool(prediction_section))
    else:
        prediction_requested = bool(prediction_override)
    prediction_settings = None
    prediction_setup_error = None
    if prediction_requested:
        try:
            if not isinstance(prediction_section, dict):
                raise ValueError(
                    "prediction_risk in hybrid_config.json must be an object")
            from carla_reconstruction.closed_loop.prediction_risk import (  # noqa: E402
                prediction_risk_settings)
            prediction_settings = prediction_risk_settings(
                config, config_path,
                enabled_override=prediction_override,
                device_override=getattr(args, "prediction_device", None))
        except Exception as exc:
            prediction_setup_error = str(exc)
            print("WARNING: prediction-risk configuration is unavailable; "
                  "continuing the hybrid simulation without it:", exc)

    ego_spec = dict(config.get("ego", {}))
    if args.ego_mode:
        ego_spec["authority"] = _authority_from_mode(args.ego_mode)
    ego_spec.setdefault("track_id", "ego")
    ego_spec.setdefault("authority", "carla_reference")
    specs = configured_carla_actor_specs(config, ego_spec)
    pending = {spec["track_id"]: spec for spec in specs}
    spawned = {}
    offsets = {}
    controllers = {}
    authority = {}
    reference_endpoint_holds = {}
    metrics = SafetyMetrics()
    clock = [0.0]
    collision_monitor = None
    prediction_monitor = None
    prediction_metadata = {
        "requested": prediction_requested,
        "active": False,
    }
    prediction_static_track_ids = sorted(
        str(spec["track_id"])
        for spec in specs
        if spec.get("authority") == STATIC_AUTHORITY)
    prediction_metadata["actor_filter"] = {
        "policy": "exclude_recorded_carla_static",
        "excluded_authority": STATIC_AUTHORITY,
        "configured_excluded_track_count": len(prediction_static_track_ids),
        "configured_excluded_track_ids": prediction_static_track_ids,
    }
    if prediction_setup_error is not None:
        prediction_metadata["error"] = prediction_setup_error
    sumo_lifecycle = {
        "inserted": set(),
        "mirrored": set(),
        "completed": set(),
        "inserted_at": {},
        "mirrored_at": {},
        "completed_at": {},
        "last_observed_time": None,
    }
    background_settings = config.get("background") or {}
    route_continuation_settings = (
        background_settings.get("route_continuation") or
        route_report.get("moving_route_continuation") or {})
    moving_initialization = (
        background_settings.get("moving_initialization") or
        route_report.get("moving_initialization") or {})
    moving_speed = (
        background_settings.get("moving_speed") or
        route_report.get("moving_speed") or {})
    mover_position_policy = moving_initialization.get(
        "position_policy", "route")
    mover_speed_policy = moving_speed.get("policy", "unbounded")
    runtime_route_continuation_requested = bool(
        route_continuation_settings.get(
            "runtime_extend_at_route_end",
            route_continuation_settings.get("policy") == "terminal"))
    runtime_route_continuator = None
    runtime_route_continuation_metadata = {
        "enabled": False,
        "requested": runtime_route_continuation_requested,
        "policy": (
            "preserve_prepared_route_then_guarded_persistent_stop_recovery"),
    }
    mover_fidelity = {
        "position_policy": mover_position_policy,
        "speed_policy": mover_speed_policy,
        "configured": 0,
        "pose_applied": set(),
        "profile_started": set(),
        "profile_released": set(),
        "pose_applied_at_s": {},
        "profile_started_at_s": {},
        "profile_released_at_s": {},
        "profile_release_reason": {},
        "terminal_stop_policy": moving_speed.get(
            "terminal_stop_policy", "release_to_sumo"),
        "terminal_stop_maximum_extent_m": float(moving_speed.get(
            "terminal_stop_maximum_extent_m", 1.0)),
        "terminal_stop_minimum_duration_s": float(moving_speed.get(
            "terminal_stop_minimum_duration_s", 2.0)),
        "terminal_release_track_time_s": {},
        "failures": {},
    }

    def update_sumo_lifecycle(sim_time):
        """Record expected nuScenes movers as they pass through the bridge."""
        sumo_lifecycle["last_observed_time"] = sim_time
        if not expected_sumo_ids:
            return
        observed = {
            "inserted": expected_sumo_ids.intersection(
                str(actor_id) for actor_id in sumo_simulation.spawned_actors),
            "mirrored": expected_sumo_ids.intersection(
                str(actor_id) for actor_id in synchronization.sumo2carla_ids),
            "completed": expected_sumo_ids.intersection(
                str(actor_id) for actor_id in sumo_simulation.destroyed_actors),
        }
        for event_name in ("inserted", "mirrored", "completed"):
            newly_observed = observed[event_name] - sumo_lifecycle[event_name]
            for actor_id in sorted(newly_observed):
                sumo_lifecycle[event_name].add(actor_id)
                sumo_lifecycle[event_name + "_at"][actor_id] = sim_time
                suffix = ""
                if event_name == "completed":
                    runtime_events = []
                    if runtime_route_continuator is not None:
                        runtime_events = _ordered_runtime_route_events(
                            runtime_route_continuator, actor_id)
                    if runtime_events:
                        latest = runtime_events[-1]
                        suffix = " (runtime-selected terminal %s from %s)" % (
                            latest["to_edge"], latest["from_edge"])
                    else:
                        continuation = route_rows.get(
                            actor_id, {}).get("continuation", {})
                        terminal_edge = continuation.get("terminal_edge")
                        status = continuation.get("status")
                        if terminal_edge:
                            suffix = " (planned terminal %s; %s)" % (
                                terminal_edge,
                                status or "status unavailable")
                print("SUMO mover %s %s at %.3f s%s" % (
                    actor_id, event_name, sim_time, suffix))

    (bootstrap_client, world, original_settings, sumo_simulation,
     carla_simulation, synchronization, projector, traffic_manager_settings,
     tm_port, traffic_manager) = _start_simulations(
        args, config, map_name, step_length,
        SimulationSynchronization, CarlaSimulation, SumoSimulation)
    static_carla_actor_ids = set()
    carla_to_sumo_spawn_exclusion = {
        "filtered_carla_actor_ids": set(),
    }
    try:
        carla_to_sumo_spawn_exclusion = (
            _install_carla_to_sumo_spawn_exclusion(
                synchronization, static_carla_actor_ids))
        mover_fidelity = _install_sumo_mover_fidelity(
            sumo_simulation, route_rows, tracks,
            mover_position_policy, mover_speed_policy, step_length,
            route_report.get("maximum_snap_distance_m", 8.0),
            terminal_stop_policy=moving_speed.get(
                "terminal_stop_policy", "release_to_sumo"),
            terminal_stop_maximum_extent=moving_speed.get(
                "terminal_stop_maximum_extent_m", 1.0),
            terminal_stop_minimum_duration=moving_speed.get(
                "terminal_stop_minimum_duration_s", 2.0))
        if runtime_route_continuation_requested:
            if not expected_sumo_ids:
                runtime_route_continuation_metadata["reason"] = (
                    "route report has no eligible SUMO mover IDs")
            elif not config.get("sumo", {}).get("network"):
                runtime_route_continuation_metadata["reason"] = (
                    "hybrid configuration has no SUMO network path")
            else:
                try:
                    import sumolib

                    network_path = config["sumo"]["network"]
                    if not os.path.isabs(network_path):
                        network_path = os.path.join(
                            os.path.dirname(config_path), network_path)
                    runtime_route_continuator = SumoRouteContinuator(
                        sumolib.net.readNet(network_path), traci.vehicle,
                        expected_sumo_ids,
                        seed=int(route_continuation_settings.get(
                            "seed", config.get("seed", 103))),
                        allow_uturns=bool(route_continuation_settings.get(
                            "allow_uturns", False)),
                        recovery_wait_s=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "minimum_duration_s", 1.0)),
                        recovery_stop_speed_mps=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "stop_speed_mps", 0.1)),
                        recovery_resume_speed_mps=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "resume_speed_mps", 0.3)),
                        recovery_progress_distance_m=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "progress_reset_distance_m", 0.5)),
                        recovery_leader_clearance_m=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "close_leader_distance_m", 15.0)),
                        recovery_tls_distance_m=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "traffic_signal_guard_distance_m", 15.0)),
                        recovery_downstream_blocked_wait_s=float(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "downstream_blocker_minimum_duration_s",
                                    1.0)),
                        maximum_suffix_recovery_attempts=int(
                            (route_continuation_settings.get(
                                "persistent_stop_recovery") or {}).get(
                                    "maximum_suffix_recovery_attempts", 3)),
                        boundary_terminal_tolerance_m=float(
                            route_continuation_settings.get(
                                "boundary_terminal_tolerance_m", 15.0)),
                        continuation_search_depth_edges=int(
                            route_continuation_settings.get(
                                "terminal_search_depth_edges", 64)))
                    _install_sumo_route_continuation(
                        sumo_simulation, runtime_route_continuator,
                        mover_fidelity)
                    runtime_route_continuation_metadata = (
                        runtime_route_continuator.metadata())
                    runtime_route_continuation_metadata["requested"] = True
                except Exception as exc:
                    runtime_route_continuation_metadata["reason"] = str(exc)
                    print("WARNING: SUMO runtime route continuation is "
                          "unavailable; prepared routes remain active:", exc)
        spectator = world.get_spectator()
        server_version = bootstrap_client.get_server_version()
        runtime_map_name = world.get_map().name
    except BaseException:
        _stop_simulations(
            synchronization, carla_simulation, sumo_simulation,
            traffic_manager, bootstrap_client, world, original_settings)
        raise

    def spawn_spec(track_id, spec):
        track = tracks.get(track_id)
        if track is None:
            print("WARNING: configured CARLA-authority track is absent:", track_id)
            return
        actor_authority = spec.get("authority", "carla_reference")
        role_name, physics = actor_spawn_policy(track_id, actor_authority)
        actor, offset = spawn_track_actor(
            world, track, projector, role_name, physics=physics)
        if actor is None:
            print("WARNING: could not spawn CARLA-authority actor", track_id)
            return
        spawned[track_id], offsets[track_id] = actor, offset
        authority[track_id] = actor_authority
        if actor_authority == STATIC_AUTHORITY:
            static_carla_actor_ids.add(actor.id)
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
                    events=spec.get("events", []),
                    endpoint_stop_tolerance=float(spec.get(
                        "endpoint_stop_tolerance_m", 0.75)),
                    endpoint_deceleration=float(spec.get(
                        "endpoint_deceleration_mps2", 3.0)))

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

    prestage_requested = []
    prestage_spawned = []
    prestage_mirrored = []
    static_sumo_proxy_exclusion = {
        "policy": "prevent_carla_static_sumo_lane_proxy_spawn",
        "detached_by_carla_actor_id": {},
        "failures": {},
    }

    def detach_static_sumo_proxies():
        static_carla_actor_ids = {
            actor.id
            for track_id, actor in spawned.items()
            if (actor.is_alive and
                authority.get(track_id) == STATIC_AUTHORITY)
        }
        detached, failures = _detach_carla_static_sumo_proxies(
            synchronization, static_carla_actor_ids)
        static_sumo_proxy_exclusion[
            "detached_by_carla_actor_id"].update(detached)
        static_sumo_proxy_exclusion["failures"].update(failures)
        for carla_actor_id in detached:
            static_sumo_proxy_exclusion["failures"].pop(
                carla_actor_id, None)
        return detached
    termination_reason = "duration_reached"
    try:
        print("CARLA server:", server_version)
        print("Map:", runtime_map_name)
        print("SUMO config:", config["sumo"]["config"])
        print("Duration %.2fs at %.3fs/tick" % (duration, step_length))
        if mover_fidelity["configured"]:
            print("SUMO mover fidelity: %s initial pose; %s speed (%d actors)" % (
                mover_position_policy, mover_speed_policy,
                mover_fidelity["configured"]))
        if mover_fidelity["terminal_release_track_time_s"]:
            print("SUMO terminal-stop fallback: %d recorded stationary "
                  "tail(s) will release to autonomous SUMO control" %
                  len(mover_fidelity["terminal_release_track_time_s"]))
        if runtime_route_continuator is not None:
            print("SUMO route-tail fallback: seeded valid outgoing edges for "
                  "%d moving actor(s)" % len(expected_sumo_ids))
        if mover_fidelity["failures"]:
            print("WARNING: %d SUMO mover fidelity specification(s) are "
                  "unavailable; see summary.json" %
                  len(mover_fidelity["failures"]))

        for track_id, spec in list(pending.items()):
            spawn_phase = spec.get("spawn_phase")
            if spawn_phase not in (None, "prestage", "recorded"):
                raise ValueError(
                    "CARLA-authority actor %s has unsupported spawn_phase %r" %
                    (track_id, spawn_phase))
            if (spec.get("authority") != STATIC_AUTHORITY or
                    spawn_phase != "prestage"):
                continue
            prestage_requested.append(track_id)
            del pending[track_id]
            spawn_spec(track_id, spec)
            if track_id in spawned:
                prestage_spawned.append(track_id)

        if spawn_schedule is not None:
            print("Pre-staged CARLA static vehicles: %d requested, %d spawned" % (
                len(prestage_requested), len(prestage_spawned)))
            if bridge_warmup_steps:
                print("Bridge warmup: %d synchronization tick(s); "
                      "scenario clock remains at 0.000 s" %
                      bridge_warmup_steps)
            for _ in range(bridge_warmup_steps):
                synchronization.tick()

            mirrored_carla_ids = set(synchronization.carla2sumo_ids)
            prestage_mirrored = [
                track_id for track_id in prestage_spawned
                if spawned[track_id].id in mirrored_carla_ids
            ]
            print("Pre-staged CARLA static SUMO lane proxies after filtering: "
                  "%d/%d" % (
                      len(prestage_mirrored), len(prestage_spawned)))
            filtered_static_ids = carla_to_sumo_spawn_exclusion[
                "filtered_carla_actor_ids"].intersection(
                    static_carla_actor_ids)
            print("Pre-staged roadside static vehicles excluded from SUMO "
                  "lane traffic: %d/%d; all remain visible in CARLA" % (
                      len(filtered_static_ids), len(prestage_spawned)))
            if prestage_mirrored:
                print("WARNING: %d roadside static SUMO proxy/proxies escaped "
                      "spawn filtering; applying removal fallback" %
                      len(prestage_mirrored))
            detached = detach_static_sumo_proxies()
            if detached:
                print("Detached %d roadside static SUMO lane proxies; the "
                      "recorded vehicles remain visible in CARLA" %
                      len(detached))

        if prediction_settings is not None:
            try:
                from carla_reconstruction.closed_loop.prediction_risk import (  # noqa: E402
                    PredictionRiskMonitor)
                prediction_monitor = PredictionRiskMonitor(
                    prediction_settings, step_length)
                prediction_metadata.update(prediction_monitor.metadata())
                prediction_metadata["active"] = True
                print("Prediction risk: HGT monitor and dashboard started; "
                      "excluding %d recorded static vehicle(s)" %
                      len(prediction_static_track_ids))
            except Exception as exc:
                prediction_metadata["error"] = str(exc)
                print("WARNING: prediction-risk dashboard could not start; "
                      "continuing the hybrid simulation without it:", exc)

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
                elif actor_authority == "carla_reference":
                    controller = controllers.get(track_id)
                    held = _apply_reference_track_control(
                        actor, controller, track, vehicles,
                        sim_time)
                    if held and track_id not in reference_endpoint_holds:
                        location = actor.get_location()
                        velocity = actor.get_velocity()
                        controller = controllers.get(track_id)
                        endpoint = (
                            controller.path[-1]
                            if controller is not None else None)
                        endpoint_distance = (
                            math.hypot(
                                endpoint[0] - location.x,
                                endpoint[1] - location.y)
                            if endpoint is not None else None)
                        reference_endpoint_holds[track_id] = {
                            "held_at_s": sim_time,
                            "recorded_track_end_s": track.end_time,
                            "terminal_approach_duration_s": max(
                                0.0, sim_time - track.end_time),
                            "endpoint_distance_m": endpoint_distance,
                            "speed_mps": math.sqrt(
                                velocity.x ** 2 + velocity.y ** 2 +
                                velocity.z ** 2),
                            "terminal_target": (
                                {"x": endpoint[0], "y": endpoint[1]}
                                if endpoint is not None else None),
                            "recorded_endpoint": (
                                {"x": endpoint[0], "y": endpoint[1]}
                                if endpoint is not None else None),
                            "hold_position": {
                                "x": float(location.x),
                                "y": float(location.y),
                                "z": float(location.z),
                            },
                        }
                        print("CARLA reference actor %s completed its terminal "
                              "endpoint approach at %.3f s (recorded track "
                              "ended at %.3f s, endpoint error %s); braking "
                              "and holding" %
                              (track_id, sim_time, track.end_time,
                               ("%.3f m" % endpoint_distance
                                if endpoint_distance is not None else
                                "unavailable")))

            synchronization.tick()
            # A non-prestaged legacy static actor is first mirrored during the
            # CARLA half of this tick. Detach it before the next SUMO step so
            # it never becomes an in-lane car-following obstacle.
            detach_static_sumo_proxies()
            # Lifecycle diagnostics are SUMO events.  Use TraCI time rather
            # than the scenario clock, which intentionally excludes bridge
            # warmup ticks.
            update_sumo_lifecycle(float(traci.simulation.getTime()))
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
                reverse_spawned = {
                    actor.id: "track:%s" % track_id
                    for track_id, actor in spawned.items()
                    if actor.is_alive and track_id != "ego"
                }
                vehicle_rows = []
                for actor in world.get_actors().filter("vehicle.*"):
                    if actor.id == ego.id:
                        continue
                    label = reverse_sumo.get(
                        actor.id,
                        reverse_spawned.get(actor.id, "carla:%s" % actor.id))
                    state = actor_state(actor)
                    state["z"] = actor.get_location().z
                    vehicle_rows.append((actor.id, label, state))
                others = {
                    label: state
                    for _carla_actor_id, label, state in vehicle_rows
                }
                ego_state = actor_state(ego)
                ego_state["z"] = ego.get_location().z
                metrics.update(sim_time, ego_state, others)
                if prediction_monitor is not None:
                    try:
                        recorded_static_carla_ids = {
                            actor.id
                            for track_id, actor in spawned.items()
                            if (actor.is_alive and
                                authority.get(track_id) == STATIC_AUTHORITY)
                        }
                        prediction_others = _prediction_risk_actor_states(
                            vehicle_rows, recorded_static_carla_ids)
                        prediction_monitor.observe(
                            sim_time, ego_state, prediction_others, world=world)
                    except Exception as exc:
                        print("WARNING: prediction-risk observer failed; "
                              "disabling it for the remainder of this run:", exc)
                        prediction_metadata["active"] = False
                        prediction_metadata["error"] = str(exc)
                        try:
                            prediction_monitor.close()
                        except Exception:
                            pass
                        prediction_monitor = None

            clock[0] += step_length
            pace_for_dashboard = (
                prediction_monitor is not None and
                prediction_monitor.realtime_pacing)
            if args.realtime or pace_for_dashboard:
                elapsed = time.time() - wall_start
                if elapsed < step_length:
                    time.sleep(step_length - elapsed)
    except KeyboardInterrupt:
        termination_reason = "interrupted"
        print("Hybrid simulation interrupted by user")
    except Exception:
        termination_reason = "error"
        raise
    finally:
        if runtime_route_continuator is not None:
            runtime_route_continuation_metadata = (
                runtime_route_continuator.metadata())
            runtime_route_continuation_metadata["requested"] = True
        mover_fidelity_metadata = {
            "position_policy": mover_fidelity["position_policy"],
            "speed_policy": mover_fidelity["speed_policy"],
            "configured": mover_fidelity["configured"],
            "pose_applied": sorted(mover_fidelity["pose_applied"]),
            "profile_started": sorted(mover_fidelity["profile_started"]),
            "profile_released": sorted(mover_fidelity["profile_released"]),
            "pose_applied_at_s": mover_fidelity["pose_applied_at_s"],
            "profile_started_at_s": mover_fidelity["profile_started_at_s"],
            "profile_released_at_s": mover_fidelity["profile_released_at_s"],
            "profile_release_reason": mover_fidelity[
                "profile_release_reason"],
            "terminal_stop_policy": mover_fidelity[
                "terminal_stop_policy"],
            "terminal_stop_maximum_extent_m": mover_fidelity[
                "terminal_stop_maximum_extent_m"],
            "terminal_stop_minimum_duration_s": mover_fidelity[
                "terminal_stop_minimum_duration_s"],
            "terminal_release_track_time_s": mover_fidelity[
                "terminal_release_track_time_s"],
            "failures": mover_fidelity["failures"],
        }
        final_sumo_states = {}
        final_sumo_state_error = None
        try:
            def optional_vehicle_call(name, *values):
                getter = getattr(traci.vehicle, name, None)
                if getter is None:
                    return None
                try:
                    return getter(*values)
                except Exception:
                    return None

            active_sumo_ids = set(
                str(value) for value in traci.vehicle.getIDList())
            for sumo_id in sorted(expected_sumo_ids.intersection(
                    active_sumo_ids)):
                route = list(traci.vehicle.getRoute(sumo_id))
                final_sumo_states[sumo_id] = {
                    "speed_mps": float(traci.vehicle.getSpeed(sumo_id)),
                    "road_id": str(traci.vehicle.getRoadID(sumo_id)),
                    "lane_id": str(traci.vehicle.getLaneID(sumo_id)),
                    "lane_position_m": float(
                        traci.vehicle.getLanePosition(sumo_id)),
                    "route_index": int(
                        traci.vehicle.getRouteIndex(sumo_id)),
                    "route_edges": route,
                    "speed_without_traci_mps": optional_vehicle_call(
                        "getSpeedWithoutTraCI", sumo_id),
                    "waiting_time_s": optional_vehicle_call(
                        "getWaitingTime", sumo_id),
                    "stop_state": optional_vehicle_call(
                        "getStopState", sumo_id),
                    "leader": optional_vehicle_call(
                        "getLeader", sumo_id, 100.0),
                    "next_tls": optional_vehicle_call(
                        "getNextTLS", sumo_id),
                    "next_links": optional_vehicle_call(
                        "getNextLinks", sumo_id),
                }
        except Exception as exc:
            final_sumo_state_error = str(exc)
        completed_route_details = {}
        for actor_id in sorted(sumo_lifecycle["completed"]):
            detail = dict(route_rows.get(actor_id, {}).get(
                "continuation", {}))
            runtime_events = []
            if runtime_route_continuator is not None:
                runtime_events = _ordered_runtime_route_events(
                    runtime_route_continuator, actor_id)
            if runtime_events:
                detail["runtime_route_mutations"] = runtime_events
                detail["effective_terminal_edge"] = runtime_events[-1][
                    "to_edge"]
            completed_route_details[actor_id] = detail
        lifecycle_metadata = {
            "sumo_time_at_stop_s": sumo_lifecycle["last_observed_time"],
            "expected": sorted(expected_sumo_ids),
            "inserted": sorted(sumo_lifecycle["inserted"]),
            "mirrored": sorted(sumo_lifecycle["mirrored"]),
            "completed": sorted(sumo_lifecycle["completed"]),
            "not_inserted_before_stop": sorted(
                expected_sumo_ids - sumo_lifecycle["inserted"]),
            "inserted_but_never_mirrored": sorted(
                sumo_lifecycle["inserted"] - sumo_lifecycle["mirrored"]),
            "active_at_stop": sorted(
                sumo_lifecycle["inserted"] - sumo_lifecycle["completed"]),
            "inserted_at_s": sumo_lifecycle["inserted_at"],
            "mirrored_at_s": sumo_lifecycle["mirrored_at"],
            "completed_at_s": sumo_lifecycle["completed_at"],
            "completed_route_details": completed_route_details,
            "final_active_states": final_sumo_states,
        }
        if final_sumo_state_error is not None:
            lifecycle_metadata["final_state_error"] = final_sumo_state_error
        if expected_sumo_ids:
            print("SUMO mover lifecycle: expected %d, inserted %d, mirrored "
                  "%d, completed %d, active at stop %d" % (
                      len(expected_sumo_ids),
                      len(sumo_lifecycle["inserted"]),
                      len(sumo_lifecycle["mirrored"]),
                      len(sumo_lifecycle["completed"]),
                      len(lifecycle_metadata["active_at_stop"])))
        if mover_fidelity["configured"]:
            print("SUMO mover fidelity result: pose initialized %d, speed "
                  "profiles started %d, failures %d" % (
                      len(mover_fidelity["pose_applied"]),
                      len(mover_fidelity["profile_started"]),
                      len(mover_fidelity["failures"])))
        if reference_endpoint_holds:
            print("CARLA reference endpoint holds: %d actor(s)" %
                  len(reference_endpoint_holds))
        if prediction_monitor is not None:
            try:
                prediction_monitor.close()
            except Exception as exc:
                print("WARNING: prediction dashboard cleanup failed:", exc)
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
            "spawn_schedule": {
                "configuration": spawn_schedule,
                "prestage_requested": len(prestage_requested),
                "prestage_spawned": len(prestage_spawned),
                "prestage_mirrored": len(prestage_mirrored),
            },
            "sumo_static_proxy_exclusion": {
                "policy": static_sumo_proxy_exclusion["policy"],
                "tradeoff": (
                    "SUMO_does_not_model_recorded_static_vehicle_bodies"),
                "filtered_count": len(carla_to_sumo_spawn_exclusion[
                    "filtered_carla_actor_ids"]),
                "filtered_carla_actor_ids": sorted(
                    carla_to_sumo_spawn_exclusion[
                        "filtered_carla_actor_ids"]),
                "detached_count": len(static_sumo_proxy_exclusion[
                    "detached_by_carla_actor_id"]),
                "detached_by_carla_actor_id": dict(sorted(
                    static_sumo_proxy_exclusion[
                        "detached_by_carla_actor_id"].items())),
                "failures": dict(sorted(
                    static_sumo_proxy_exclusion["failures"].items())),
            },
            "sumo_mover_lifecycle": lifecycle_metadata,
            "sumo_mover_fidelity": mover_fidelity_metadata,
            "sumo_runtime_route_continuation": (
                runtime_route_continuation_metadata),
            "reference_track_end_control": {
                "policy": "spatial_recorded_endpoint_then_full_brake_hold",
                "held_actor_count": len(reference_endpoint_holds),
                "actors": reference_endpoint_holds,
            },
            "prediction_risk": prediction_metadata,
        }
        csv_path = summary_path = None
        try:
            csv_path, summary_path = metrics.write(output, run_metadata)
        except Exception as exc:
            print("WARNING: could not write run metrics:", exc)
        if collision_monitor:
            try:
                collision_monitor.destroy()
            except Exception:
                pass
        for actor in list(spawned.values()):
            try:
                if actor.type_id.startswith("vehicle."):
                    actor.set_autopilot(False, tm_port)
                actor.destroy()
            except Exception:
                pass
        close_error = _stop_simulations(
            synchronization, carla_simulation, sumo_simulation,
            traffic_manager, bootstrap_client, world, original_settings)
        if close_error is not None:
            print("WARNING: co-simulation cleanup failed:", close_error)
        if csv_path:
            print("Metrics:", csv_path)
        if summary_path:
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
    prediction_group = parser.add_mutually_exclusive_group()
    prediction_group.add_argument(
        "--prediction-risk", dest="prediction_risk", action="store_true",
        help="enable the optional HGT prediction-risk observer and dashboard")
    prediction_group.add_argument(
        "--no-prediction-risk", dest="prediction_risk", action="store_false",
        help="disable prediction risk even when hybrid_config.json enables it")
    parser.set_defaults(prediction_risk=None)
    parser.add_argument(
        "--prediction-device",
        help="Torch device override for prediction, such as cpu, cuda, or cuda:0")
    parser.add_argument("--output")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
