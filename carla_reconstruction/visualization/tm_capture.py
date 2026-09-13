"""Synchronized sensors observing continuous TM driving; never ticks the world."""

import math
from pathlib import Path
import time

from .capture import SensorFrameBuffer, _set_attribute, file_sha256, write_json


def add_capture_arguments(parser):
    parser.add_argument("--capture-sensors", action="store_true",
                        help="save six RGB cameras and LiDAR under <output>/capture")
    parser.add_argument("--build-sim-panels", action="store_true",
                        help="capture sensors and build simulation-only PNG panels/GIF after the run")
    parser.add_argument("--capture-interval", type=float, default=0.5)
    parser.add_argument("--capture-start", type=float, default=0.5,
                        help="first sensor sample time; driving continues during sensor warmup")
    parser.add_argument("--capture-width", type=int, default=640)
    parser.add_argument("--capture-height", type=int, default=360)
    parser.add_argument("--capture-timeout", type=float, default=15.0)


def capture_enabled(args):
    return getattr(args, "capture_sensors", False) or getattr(args, "build_sim_panels", False)


def validate_capture_arguments(args, output, duration):
    if not capture_enabled(args):
        return
    if args.no_rendering:
        raise ValueError("camera capture requires rendering; remove --no-rendering")
    for name in ("capture_interval", "capture_start", "capture_timeout", "step_length"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be finite and positive")
    if args.capture_interval < args.step_length:
        raise ValueError("capture interval must be at least one simulation step")
    if min(args.capture_width, args.capture_height) < 32:
        raise ValueError("capture image dimensions must be at least 32")
    if not math.isfinite(duration) or args.capture_start > duration - 3 * args.step_length:
        raise ValueError("duration must leave three sensor pipeline ticks after capture-start")
    for directory in (Path(output) / "capture", Path(output) / "sim_panels"):
        if directory.exists() and (not directory.is_dir() or any(directory.iterdir())):
            raise ValueError("capture output is not empty; choose a new --output: " + str(directory))
    # Fail for missing optional dependencies before loading a world.
    import numpy  # noqa: F401
    import PIL  # noqa: F401


class TMSensorCapture:
    def __init__(self, args, manifest, manifest_path, output, duration, scenario):
        self.args = args
        self.root = Path(output) / "capture"
        self.duration = duration
        self.scenario = scenario
        self.sensors = []
        self.buffer = None
        self.requested = {}
        self.next_time = args.capture_start
        self.audit = {
            "schema_version": 1, "mode": "traffic_manager_sensor_capture",
            "scene": manifest["scene"], "manifest_path": str(Path(manifest_path).resolve()),
            "manifest_sha256": file_sha256(manifest_path),
            "camera_rig": "approximate_nuscenes", "complete": False, "frames": [],
            "frame_dt_s": args.capture_interval,
            "capture_parameters": {
                "camera_width": args.capture_width, "camera_height": args.capture_height,
                "simulation_step_s": args.step_length, "interval_s": args.capture_interval,
                "start_s": args.capture_start, "end_margin_s": 3 * args.step_length,
                "timing": "continuous TM driving; camera/LiDAR/ego state joined by CARLA frame ID",
                "lidar_weather_model": "standard CARLA ray-cast; no added rain scattering model",
            },
        }

    def start(self, world, ego):
        import carla
        from replay_geo.replay_geo_carla import NUSCENES_CAMS
        self.root.mkdir(parents=True, exist_ok=True)
        names = [row[0] for row in NUSCENES_CAMS] + ["LIDAR_TOP"]
        self.buffer = SensorFrameBuffer(names)
        self.audit["runtime_map"] = world.get_map().name
        self.audit["camera_rig_parameters"] = [list(row) for row in NUSCENES_CAMS]
        write_json(self.root / "capture_index.json", self.audit)
        library = world.get_blueprint_library()
        for name, x, y, z, yaw, fov in NUSCENES_CAMS:
            bp = library.find("sensor.camera.rgb")
            for key, value in (("image_size_x", self.args.capture_width),
                               ("image_size_y", self.args.capture_height), ("fov", fov),
                               ("sensor_tick", 0.0), ("motion_blur_intensity", 0.0)):
                _set_attribute(bp, key, value)
            sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x, y, z),
                                      carla.Rotation(yaw=yaw)), attach_to=ego)
            self.sensors.append(sensor)
            sensor.listen(self.buffer.callback(name))
            (self.root / "carla_cams" / name).mkdir(parents=True)
        bp = library.find("sensor.lidar.ray_cast")
        for key, value in (("channels", 32), ("range", 70), ("upper_fov", 10),
                           ("lower_fov", -30), ("rotation_frequency", 1.0 / self.args.step_length),
                           ("points_per_second", 640000), ("sensor_tick", 0.0), ("noise_stddev", 0.0)):
            _set_attribute(bp, key, value)
        sensor = world.spawn_actor(bp, carla.Transform(carla.Location(0.9, 0.0, 1.84)), attach_to=ego)
        self.sensors.append(sensor)
        sensor.listen(self.buffer.callback("LIDAR_TOP"))
        (self.root / "lidar").mkdir()
        self.audit["lidar_rig_parameters"] = {
            "mount_carla_m": [0.9, 0.0, 1.84], "channels": 32, "range_m": 70,
            "rotation_frequency_hz": 1.0 / self.args.step_length, "points_per_second": 640000}

    def observe(self, frame_id, elapsed, ego, world):
        # Reserve actual current-tick state before any delayed GPU callback is used.
        if elapsed + 1e-7 >= self.next_time and elapsed <= self.duration - 3 * self.args.step_length + 1e-7:
            transform, velocity = ego.get_transform(), ego.get_velocity()
            snapshot = world.get_snapshot()
            if int(snapshot.frame) != int(frame_id):
                raise RuntimeError("world advanced outside the TM runner during sensor capture")
            self.requested[int(frame_id)] = {
                "carla_frame": int(frame_id), "relative_time_s": float(elapsed),
                "timestamp_us": round(elapsed * 1e6),
                "world_timestamp_s": snapshot.timestamp.elapsed_seconds,
                "ego_carla_xyz": [transform.location.x, transform.location.y, transform.location.z],
                "ego_map_yaw_rad": -math.radians(transform.rotation.yaw),
                "ego_speed_kmh": 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2),
                "ego_control": {key: float(getattr(ego.get_control(), key)) for key in ("throttle", "brake", "steer")},
            }
            self.next_time += self.args.capture_interval
        # Wait for this tick's GPU work before the runner advances again. On
        # CARLA 0.9.15 some camera headers otherwise acquire the following tick's
        # timestamp even while retaining the original frame ID. This is only a
        # wall-clock wait: the TM runner remains the sole owner of world.tick().
        deadline = time.monotonic() + self.args.capture_timeout
        while not self.buffer.names.issubset(self.buffer.pending.get(int(frame_id), {})):
            self.buffer.drain(timeout=0.01)
            if time.monotonic() >= deadline:
                raise TimeoutError("sensors did not finish simulation frame %s" % frame_id)
        self.flush()
        if self.requested and int(frame_id) - min(self.requested) > 24:
            raise TimeoutError("sensor callbacks did not complete requested frame %s" % min(self.requested))

    def flush(self, timeout=0.0):
        import numpy as np
        self.buffer.drain(timeout)
        for frame in sorted(self.requested):
            measurements = self.buffer.pending.get(frame, {})
            if not self.buffer.names.issubset(measurements):
                break  # keep output chronologically ordered despite delayed callbacks
            row = self.requested.pop(frame)
            index = len(self.audit["frames"])
            row.update(index=index, cameras={}, sensor_frame_ids={}, sensor_timestamps_s={})
            for name, measurement in measurements.items():
                row["sensor_frame_ids"][name] = int(measurement.frame)
                row["sensor_timestamps_s"][name] = float(measurement.timestamp)
                if name == "LIDAR_TOP":
                    relative = "lidar/%04d.npy" % index
                    cloud = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4).copy()
                    np.save(str(self.root / relative), cloud, allow_pickle=False)
                    row["lidar_file"] = relative
                    row["lidar_point_count"] = len(cloud)
                    row["lidar_sensor_world_matrix"] = measurement.transform.get_matrix()
                else:
                    relative = "carla_cams/%s/%04d.png" % (name, index)
                    measurement.save_to_disk(str(self.root / relative))
                    row["cameras"][name] = relative
            self.audit["frames"].append(row)
            write_json(self.root / "capture_index.json", self.audit)
            print("Sensor capture: t=%.2fs, CARLA frame %d, %d LiDAR points" %
                  (row["relative_time_s"], frame, row["lidar_point_count"]), flush=True)
        # Drop unrequested sensor frames; preserve future callbacks and outstanding requests.
        boundary = min(self.requested) if self.requested else max(self.buffer.pending, default=0)
        for frame in list(self.buffer.pending):
            if frame < boundary:
                del self.buffer.pending[frame]

    def finish(self):
        deadline = time.monotonic() + self.args.capture_timeout
        while self.requested and time.monotonic() < deadline:
            self.flush(timeout=0.1)
        if self.requested:
            raise TimeoutError("missing synchronized sensor frames: %s" % sorted(self.requested))
        if not self.audit["frames"]:
            raise RuntimeError("TM run produced no sensor frames")
        self.audit["complete"] = True

    def close(self, termination_reason, error=None):
        try:
            cleanup_errors = []
            for sensor in reversed(self.sensors):
                try:
                    sensor.stop()
                except Exception as exc:
                    cleanup_errors.append(str(exc))
                try:
                    sensor.destroy()
                except Exception as exc:
                    cleanup_errors.append(str(exc))
            if cleanup_errors:
                self.audit["sensor_cleanup_errors"] = cleanup_errors
        finally:
            self.audit["scenario"] = self.scenario.metadata
            self.audit["termination_reason"] = termination_reason
            self.audit["error"] = error
            self.audit["pending_frame_ids"] = sorted(self.requested)
            if termination_reason != "duration_reached":
                self.audit["complete"] = False
            if self.root.is_dir():
                write_json(self.root / "capture_index.json", self.audit)
