"""Capture aligned recorded-keyframe views on an imported persistent map.

This is a separate open-loop reconstruction comparison, not a TM/SUMO runner.
Actors are held at each recorded keyframe while sensors settle. Camera/LiDAR
measurements must share an actual CARLA frame ID before the keyframe is saved.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import queue
import time


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def selected_indices(count, start=0, stride=1, limit=None):
    if count < 1 or start < 0 or start >= count or stride < 1 or (limit is not None and limit < 1):
        raise ValueError("require a valid --start-frame, positive --frame-stride/--max-frames")
    indices = list(range(start, count, stride))
    return indices if limit is None else indices[:limit]


class SensorFrameBuffer:
    """Bounded frame-ID matching; never assign the next queued image by index."""

    def __init__(self, names):
        self.names = frozenset(names)
        if not self.names:
            raise ValueError("at least one sensor is required")
        self.queue = queue.Queue()
        self.pending = {}

    def callback(self, name):
        if name not in self.names:
            raise ValueError("unknown sensor: " + name)
        return lambda measurement: self.queue.put((name, measurement))

    def drain(self, timeout=0.0):
        try:
            name, measurement = self.queue.get(timeout=timeout)
        except queue.Empty:
            return
        while True:
            self.pending.setdefault(int(measurement.frame), {})[name] = measurement
            try:
                name, measurement = self.queue.get_nowait()
            except queue.Empty:
                break
        # GPU callbacks may trail simulation ticks. Bound retained old frames
        # even if one sensor never publishes or a capture fails part-way.
        for frame in sorted(self.pending)[:-32]:
            del self.pending[frame]

    def pop_aligned(self, minimum_frame):
        eligible = [frame for frame, items in self.pending.items()
                    if frame >= minimum_frame and self.names.issubset(items)]
        if not eligible:
            return None
        frame = max(eligible)
        result = self.pending[frame]
        for old in list(self.pending):
            if old <= frame:
                del self.pending[old]
        return frame, result

    def capture(self, world, warmup_ticks=5, timeout=10.0):
        if (not isinstance(warmup_ticks, int) or warmup_ticks < 1 or
                not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("require positive integer warmup_ticks and finite positive timeout")
        # Hold all poses throughout this method. Extra ticks allow delayed GPU
        # callbacks to complete without associating a stale view with a new pose.
        for _ in range(warmup_ticks):
            minimum_frame = int(world.tick())
            self.drain()
        deadline = time.monotonic() + timeout
        while True:
            self.drain(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            result = self.pop_aligned(minimum_frame)
            if result is not None:
                return result
            if time.monotonic() >= deadline:
                available = {frame: sorted(items) for frame, items in self.pending.items()
                             if frame >= minimum_frame}
                raise TimeoutError("camera/LiDAR frame matching timed out; received %r" % available)
            world.tick()


def map_geometry(carla_map, environment):
    """Sample actual CARLA lanes, including curved junction connectors."""
    groups = {}
    points = list(carla_map.generate_waypoints(1.0))
    for first, last in carla_map.get_topology():
        points.extend((first, last))
    for wp in points:
        if str(wp.lane_type) != "Driving":
            continue
        key = (wp.road_id, wp.lane_id, wp.section_id)
        groups.setdefault(key, {})[round(wp.s, 5)] = wp
    roads = []
    for (road, lane, section), stations in sorted(groups.items()):
        left, right = [], []
        for _, wp in sorted(stations.items()):
            transform = wp.transform
            center = transform.location
            direction = transform.get_right_vector()
            half = 0.5 * wp.lane_width
            # Export right-handed patch-local coordinates to match the manifest.
            left.append([center.x - half * direction.x, -(center.y - half * direction.y)])
            right.append([center.x + half * direction.x, -(center.y + half * direction.y)])
        if len(left) >= 2:
            roads.append({"road_id": road, "lane_id": lane, "section_id": section,
                          "polygon": left + list(reversed(right))})
    if not roads:
        raise ValueError("loaded CARLA map has no driving-lane geometry")
    return {"coordinate_frame": "opendrive_patch_local_metres", "roads": roads,
            "environment": environment}


def _set_attribute(blueprint, name, value):
    if blueprint.has_attribute(name):
        blueprint.set_attribute(name, str(value))


def capture_scene(scene, args, output):
    # Only capture needs the CARLA API; panel building and --help stay offline.
    import carla
    import numpy as np
    from carla_reconstruction.closed_loop.carla_runtime import (
        GroundProjector, spawn_track_actor, place_from_point)
    from carla_reconstruction.closed_loop.tracks import ActorTrack, TrackPoint
    from replay_geo.replay_geo_carla import NUSCENES_CAMS

    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("capture output is not empty: %s; choose a new --output" % output)
    indices = selected_indices(len(scene["frames"]), args.start_frame,
                               args.frame_stride, args.max_frames)
    manifest, meta = scene["manifest"], scene["meta"]
    requested_map = manifest["map"]["runtime_name"]
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.load_world(requested_map)
    carla_map = world.get_map()
    expected_map = requested_map.replace("\\", "/").removeprefix("/Game/").strip("/")
    actual_map = carla_map.name.replace("\\", "/").removeprefix("/Game/").strip("/")
    if actual_map != expected_map:
        raise ValueError("loaded map %s differs from manifest %s" % (carla_map.name, requested_map))
    original_settings = world.get_settings()
    actors, offsets, sensors = {}, {}, []
    audit = {
        "schema_version": 1, "scene": scene["scene"],
        "manifest_path": scene["manifest_path"],
        "manifest_sha256": file_sha256(scene["manifest_path"]),
        "meta_sha256": file_sha256(scene["meta_path"]),
        "mode": "recorded_reconstruction_replay", "runtime_map": carla_map.name,
        "frame_dt_s": float(meta.get("frame_dt", 0.5)),
        "complete": False, "map_geometry_file": "map_geometry.json", "frames": [],
        "camera_rig": "approximate_nuscenes",
        "camera_rig_parameters": [list(row) for row in NUSCENES_CAMS],
        "capture_parameters": {
            "start_frame": args.start_frame, "frame_stride": args.frame_stride,
            "max_frames": args.max_frames, "requested_indices": indices,
            "camera_width": args.cam_width, "camera_height": args.cam_height,
            "simulation_step_s": 0.05, "warmup_ticks_per_keyframe": args.warmup_ticks,
            "lidar_enabled": not args.no_lidar,
            "timing": "recorded keyframe poses held until matching CARLA sensor frames arrive",
            "camera_calibration": "approximate six-camera rig, not pixel-calibrated ground truth",
        },
    }
    projector = GroundProjector(carla_map)

    def point(annotation, index):
        return TrackPoint(index, scene["frames"][index]["relative_time_s"],
                          float(annotation["x"]), float(annotation["y"]),
                          float(annotation["yaw"]), tuple(annotation.get("wlh", ())))

    def place(track_id, category, annotation, index, is_ego=False):
        current = point(annotation, index)
        if track_id not in actors:
            track = ActorTrack(track_id, category, [current], is_ego=is_ego)
            actor, offset = spawn_track_actor(
                world, track, projector, "hero" if is_ego else "reconstruction_capture",
                physics=False)
            if actor is None:
                raise RuntimeError("cannot spawn recorded actor %s; capture aborted, not silently omitted" % track_id)
            actors[track_id], offsets[track_id] = actor, offset
        place_from_point(actors[track_id], current, projector, offsets[track_id])

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        output.mkdir(parents=True, exist_ok=True)
        write_json(output / "capture_index.json", audit)
        write_json(output / "map_geometry.json", map_geometry(carla_map, manifest.get("environment", {})))
        (output / "loaded_map.xodr").write_text(carla_map.to_opendrive(), encoding="utf-8")
        audit["loaded_xodr_sha256"] = file_sha256(output / "loaded_map.xodr")
        place("ego", "vehicle.car", meta["ego_trajectory_local"][indices[0]], indices[0], True)
        ego = actors["ego"]
        names = [row[0] for row in NUSCENES_CAMS]
        if not args.no_lidar:
            names.append("LIDAR_TOP")
        buffer = SensorFrameBuffer(names)
        library = world.get_blueprint_library()
        for name, x, y, z, yaw, fov in NUSCENES_CAMS:
            blueprint = library.find("sensor.camera.rgb")
            for attribute, value in (("image_size_x", args.cam_width), ("image_size_y", args.cam_height),
                                     ("fov", fov), ("sensor_tick", 0.0), ("motion_blur_intensity", 0.0)):
                _set_attribute(blueprint, attribute, value)
            sensor = world.spawn_actor(blueprint, carla.Transform(
                carla.Location(x, y, z), carla.Rotation(yaw=yaw)), attach_to=ego)
            sensors.append(sensor)
            sensor.listen(buffer.callback(name))
            (output / "carla_cams" / name).mkdir(parents=True, exist_ok=True)
        if not args.no_lidar:
            blueprint = library.find("sensor.lidar.ray_cast")
            for attribute, value in (("channels", 32), ("range", 70), ("upper_fov", 10),
                                     ("lower_fov", -30), ("rotation_frequency", 20),
                                     ("points_per_second", 640000), ("sensor_tick", 0.0)):
                _set_attribute(blueprint, attribute, value)
            sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(0.9, 0.0, 1.84)), attach_to=ego)
            sensors.append(sensor)
            sensor.listen(buffer.callback("LIDAR_TOP"))
            (output / "lidar").mkdir(parents=True, exist_ok=True)
            audit["lidar_rig_parameters"] = {
                "mount_carla_m": [0.9, 0.0, 1.84], "channels": 32, "range_m": 70,
                "upper_fov_deg": 10, "lower_fov_deg": -30,
                "rotation_frequency_hz": 20, "points_per_second": 640000}
        for number, index in enumerate(indices, 1):
            source = scene["frames"][index]
            place("ego", "vehicle.car", meta["ego_trajectory_local"][index], index, True)
            categories = {"ego": "vehicle.car"}
            for annotation in meta["agent_frames"][index]:
                category = annotation.get("category", "")
                if not category.startswith(("vehicle.", "human.pedestrian.")):
                    continue
                track_id = str(annotation["id"])
                place(track_id, category, annotation, index)
                categories[track_id] = category
            for track_id, actor in actors.items():
                if track_id not in categories:
                    actor.set_location(carla.Location(z=-1000.0))
            ego_transform = ego.get_transform()
            world.get_spectator().set_transform(carla.Transform(
                carla.Location(ego_transform.location.x, ego_transform.location.y,
                               ego_transform.location.z + 60.0),
                carla.Rotation(pitch=-90.0, yaw=-90.0)))
            frame, measurements = buffer.capture(world, args.warmup_ticks, args.sensor_timeout)
            row = {key: source[key] for key in ("index", "sample_token", "timestamp_us", "relative_time_s")}
            row.update(carla_frame=frame, cameras={}, actors=[])
            for name, *_ in NUSCENES_CAMS:
                relative = "carla_cams/%s/%03d.png" % (name, index)
                measurements[name].save_to_disk(str(output / relative))
                row["cameras"][name] = relative
            ego_location = ego.get_location()
            row["ego_carla_xyz"] = [ego_location.x, ego_location.y, ego_location.z]
            row["ego_map_yaw_rad"] = -math.radians(ego.get_transform().rotation.yaw)
            if not args.no_lidar:
                lidar = measurements["LIDAR_TOP"]
                relative = "lidar/%03d.npy" % index
                cloud = np.frombuffer(lidar.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                np.save(str(output / relative), cloud, allow_pickle=False)
                row["lidar_file"] = relative
                row["lidar_sensor_world_matrix"] = lidar.transform.get_matrix()
            for track_id, category in categories.items():
                actor = actors[track_id]
                transform = actor.get_transform()
                bbox = actor.bounding_box
                # Bounding-box local center need not equal the blueprint pivot.
                center = transform.transform(bbox.location)
                row["actors"].append({
                    "track_id": track_id, "category": category, "is_ego": track_id == "ego",
                    "x": center.x, "y": -center.y,
                    "yaw_rad": -math.radians(transform.rotation.yaw + bbox.rotation.yaw),
                    "length": 2.0 * bbox.extent.x, "width": 2.0 * bbox.extent.y})
            audit["frames"].append(row)
            write_json(output / "capture_index.json", audit)
            print("[%s] captured %d/%d: sample %d, CARLA frame %d" %
                  (scene["scene"], number, len(indices), index, frame), flush=True)
        audit["complete"] = True
    except BaseException as exc:
        audit["error"] = "%s: %s" % (type(exc).__name__, exc)
        raise
    finally:
        try:
            if output.is_dir():
                write_json(output / "capture_index.json", audit)
        finally:
            for sensor in reversed(sensors):
                try:
                    sensor.stop()
                except Exception:
                    pass
                try:
                    sensor.destroy()
                except Exception:
                    pass
            for actor in reversed(list(actors.values())):
                try:
                    actor.destroy()
                except Exception:
                    pass
            world.apply_settings(original_settings)
    print("Capture index:", output / "capture_index.json")
    return output / "capture_index.json"


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    selector = parser.add_mutually_exclusive_group(required=True)
    selector.add_argument("--manifest", help="scene_manifest.json")
    selector.add_argument("--scene", help="scene ID/query or all prepared manifests")
    parser.add_argument("--generated-root", help="manifest discovery root; defaults to carla_reconstruction/generated")
    parser.add_argument("--dataroot", help="override nuScenes data root from manifest")
    parser.add_argument("--version", help="override nuScenes table version")
    parser.add_argument("--output", help="new capture directory; single-scene requests only")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--sensor-timeout", type=float, default=10.0)
    parser.add_argument("--warmup-ticks", type=int, default=5)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=360)
    parser.add_argument("--no-lidar", action="store_true", help="camera/topdown capture only; full panels require LiDAR")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, help="optional explicit subset, useful for a short check")
    return parser


def main():
    from carla_reconstruction.visualization.scene_data import load_scene, read_manifest_selection
    parser = build_parser()
    args = parser.parse_args()
    if (args.cam_width < 32 or args.cam_height < 32 or args.warmup_ticks < 1 or
            not math.isfinite(args.timeout) or args.timeout <= 0 or
            not math.isfinite(args.sensor_timeout) or args.sensor_timeout <= 0):
        parser.error("require camera dimensions >=32, positive warmup ticks and finite positive timeouts")
    paths = read_manifest_selection(args.manifest, args.scene, args.generated_root)
    if args.output and len(paths) != 1:
        parser.error("--output is only valid for one scene")
    for path in paths:
        scene = load_scene(path, dataroot=args.dataroot, version=args.version)
        generated = Path(args.generated_root) if args.generated_root else Path(__file__).resolve().parents[1] / "generated"
        output = (Path(args.output) if args.output else
                  generated / "visualizations" / scene["scene"] / "capture")
        capture_scene(scene, args, output)


if __name__ == "__main__":
    main()
