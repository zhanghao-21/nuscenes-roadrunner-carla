#!/usr/bin/env python3
"""Replay nuScenes actors on a persistent, source-built CARLA map."""

import argparse
import json
import math
import os
import sys

import carla


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from replay_geo import replay_geo_carla as legacy  # noqa: E402


def load_json(path):
    with open(path, "r") as stream:
        return json.load(stream)


def dynamic_annotations(frame):
    """Exclude persistent nuScenes props if metadata was made with --include-static."""
    return [item for item in frame
            if item.get("category", "").startswith(("vehicle.", "human.pedestrian."))]


def ground_z(carla_map, x, y, cache):
    key = (round(x), round(y))
    if key not in cache:
        waypoint = carla_map.get_waypoint(
            carla.Location(x=x, y=y, z=0.0), project_to_road=True,
            lane_type=carla.LaneType.Driving)
        cache[key] = waypoint.transform.location.z if waypoint else 0.0
    return cache[key]


def chase_transform(ego_transform):
    yaw = math.radians(ego_transform.rotation.yaw)
    location = carla.Location(
        x=ego_transform.location.x - 10.0 * math.cos(yaw),
        y=ego_transform.location.y - 10.0 * math.sin(yaw),
        z=ego_transform.location.z + 5.0)
    return carla.Transform(location, carla.Rotation(
        pitch=-15.0, yaw=ego_transform.rotation.yaw))


def draw_trails(world, meta, index, history_seconds, future_seconds):
    frame_dt = float(meta.get("frame_dt", 0.5))
    history = max(0, int(round(history_seconds / frame_dt)))
    future = max(0, int(round(future_seconds / frame_dt)))
    trajectory = meta["ego_trajectory_local"]
    start = max(0, index - history)
    end = min(len(trajectory) - 1, index + future)
    for offset in range(start, end):
        a = trajectory[offset]
        b = trajectory[offset + 1]
        ax, ay, _ = legacy.carla_from_opendrive(a["x"], a["y"], a["yaw"])
        bx, by, _ = legacy.carla_from_opendrive(b["x"], b["y"], b["yaw"])
        color = carla.Color(170, 70, 35) if offset < index else carla.Color(255, 185, 35)
        world.debug.draw_line(carla.Location(ax, ay, 0.35), carla.Location(bx, by, 0.35),
                              thickness=0.08, color=color, life_time=0.7)


def run(args):
    manifest = load_json(os.path.abspath(args.manifest))
    meta = load_json(manifest["trajectories"]["meta"])
    map_name = args.map or manifest["map"]["runtime_name"]

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    print("CARLA server:", client.get_server_version())
    print("Loading persistent map:", map_name)
    world = client.load_world(map_name)
    carla_map = world.get_map()
    trajectory = meta["ego_trajectory_local"]
    frames = meta.get("agent_frames", [])
    if not trajectory:
        raise RuntimeError("metadata contains no ego trajectory")

    first = trajectory[0]
    ex, ey, eyaw = legacy.carla_from_opendrive(first["x"], first["y"], first["yaw"])
    cache = {}
    ego_bp = legacy._pick_blueprint(world, "car", 0)
    ego_tf = carla.Transform(
        carla.Location(ex, ey, ground_z(carla_map, ex, ey, cache) + 1.0),
        carla.Rotation(yaw=eyaw))
    ego = world.try_spawn_actor(ego_bp, ego_tf)
    if ego is None:
        ego_tf.location.z += 2.0
        ego = world.try_spawn_actor(ego_bp, ego_tf)
    if ego is None:
        raise RuntimeError("could not spawn ego")
    ego.set_simulate_physics(False)
    ego_z = legacy._z_offset(ego)

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    frame_dt = float(meta.get("frame_dt", 0.5)) / max(args.replay_speed, 1.0e-3)
    substeps = max(1, args.substeps)
    settings.fixed_delta_seconds = max(frame_dt / substeps, 0.01)
    world.apply_settings(settings)

    actors = {}
    actor_offsets = {}
    hidden = carla.Location(z=-200.0)
    spectator = world.get_spectator()
    rig = None
    recorder = None
    output = os.path.abspath(args.output) if args.output else os.path.join(
        REPO_ROOT, "replay_geo", "results", manifest["scene"])
    if args.cameras:
        rig = legacy._CamRig(world, ego, args.cam_width, args.cam_height)
        for name, *_ in legacy.NUSCENES_CAMS:
            os.makedirs(os.path.join(output, "carla_cams", name), exist_ok=True)
    if args.record:
        recorder = legacy._ChaseRecorder(world, args.rec_width, args.rec_height)
        os.makedirs(os.path.join(output, "carla_frames"), exist_ok=True)

    def spawn_agent(annotation):
        actor_id = annotation["id"]
        if actor_id in actors:
            return actors[actor_id]
        kind = legacy._category_kind(annotation["category"])
        bp = legacy._pick_blueprint(world, kind, sum(bytearray(actor_id.encode("utf-8"))))
        x, y, yaw = legacy.carla_from_opendrive(
            annotation["x"], annotation["y"], annotation["yaw"])
        transform = carla.Transform(
            carla.Location(x, y, ground_z(carla_map, x, y, cache) + 1.0),
            carla.Rotation(yaw=yaw))
        actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            transform.location.z += 2.0
            actor = world.try_spawn_actor(bp, transform)
        if actor is None:
            return None
        try:
            actor.set_simulate_physics(False)
        except Exception:
            pass
        actors[actor_id] = actor
        actor_offsets[actor_id] = legacy._z_offset(actor)
        return actor

    def place(actor, x, y, yaw, z_offset):
        cx, cy, cyaw = legacy.carla_from_opendrive(x, y, yaw)
        actor.set_transform(carla.Transform(
            carla.Location(cx, cy, ground_z(carla_map, cx, cy, cache) + z_offset),
            carla.Rotation(yaw=cyaw)))

    def play_once():
        for index in range(len(trajectory) - 1):
            current_frame = frames[index] if index < len(frames) else []
            next_frame = frames[index + 1] if index + 1 < len(frames) else []
            current_agents = {a["id"]: a for a in dynamic_annotations(current_frame)}
            next_agents = {a["id"]: a for a in dynamic_annotations(next_frame)}
            for annotation in current_agents.values():
                spawn_agent(annotation)
            for substep in range(substeps):
                ratio = float(substep) / substeps
                a, b = trajectory[index], trajectory[index + 1]
                ego_x = legacy._lerp(a["x"], b["x"], ratio)
                ego_y = legacy._lerp(a["y"], b["y"], ratio)
                ego_yaw = legacy._lerp_angle(a["yaw"], b["yaw"], ratio)
                place(ego, ego_x, ego_y, ego_yaw, ego_z)

                present = set()
                for actor_id in set(current_agents) | set(next_agents):
                    left = current_agents.get(actor_id)
                    right = next_agents.get(actor_id)
                    annotation = left or right
                    actor = spawn_agent(annotation)
                    if actor is None:
                        continue
                    if left and right:
                        x = legacy._lerp(left["x"], right["x"], ratio)
                        y = legacy._lerp(left["y"], right["y"], ratio)
                        yaw = legacy._lerp_angle(left["yaw"], right["yaw"], ratio)
                    else:
                        x, y, yaw = annotation["x"], annotation["y"], annotation["yaw"]
                    place(actor, x, y, yaw, actor_offsets[actor_id])
                    present.add(actor_id)
                for actor_id, actor in actors.items():
                    if actor_id not in present:
                        actor.set_location(hidden)

                ego_transform = ego.get_transform()
                chase = chase_transform(ego_transform)
                spectator.set_transform(chase)
                if recorder:
                    recorder.place(chase)
                if not args.no_trails and substep == 0:
                    draw_trails(world, meta, index, args.history_seconds, args.future_seconds)
                world.tick()
                save_keyframe = substep == 0
                if rig:
                    rig.drain(os.path.join(output, "carla_cams") if save_keyframe else None,
                              index if save_keyframe else None)
                if recorder:
                    recorder.drain(os.path.join(output, "carla_frames", "%03d.png" % index)
                                   if save_keyframe else None)

    try:
        while True:
            play_once()
            if not args.loop:
                break
    finally:
        if rig:
            rig.destroy()
        if recorder:
            recorder.destroy()
        for actor in actors.values():
            actor.destroy()
        ego.destroy()
        world.apply_settings(original_settings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--map", help="CARLA map name/path; overrides manifest")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--substeps", type=int, default=10)
    parser.add_argument("--replay-speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--no-trails", action="store_true")
    parser.add_argument("--history-seconds", type=float, default=2.0)
    parser.add_argument("--future-seconds", type=float, default=6.0)
    parser.add_argument("--cameras", action="store_true")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--cam-width", type=int, default=800)
    parser.add_argument("--cam-height", type=int, default=450)
    parser.add_argument("--rec-width", type=int, default=1280)
    parser.add_argument("--rec-height", type=int, default=720)
    parser.add_argument("--output",
                        help="scene output folder; defaults to replay_geo/results/<scene>")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
