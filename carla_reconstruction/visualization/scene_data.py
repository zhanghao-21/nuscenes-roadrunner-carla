"""Read and align reconstruction metadata with original nuScenes keyframes.

This module needs no nuScenes devkit and never imports CARLA. Sensor timestamps
are retained: camera exposures and the LiDAR scan belonging to a keyframe are
not represented as if they were captured at precisely the same instant.
"""

import hashlib
import json
import math
from pathlib import Path, PureWindowsPath

import numpy as np


CAMERA_GRID = (
    ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"),
    ("CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"),
)
CAMERA_CHANNELS = tuple(channel for row in CAMERA_GRID for channel in row)
_TABLES = ("scene", "sample", "sample_data", "calibrated_sensor", "ego_pose", "sensor")


def _json(path):
    with Path(path).open("r", encoding="utf-8-sig") as stream:
        return json.load(stream)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_manifest_selection(manifest=None, scene=None, generated_root=None):
    """Resolve an explicit manifest or unambiguous scene name (including all).

    Only immediate scene directories are scanned, so simulation variants and
    reports under generated/closed_loop cannot be accidentally selected.
    """
    if manifest is not None:
        if scene is not None:
            raise ValueError("Choose --manifest or --scene, not both.")
        path = Path(manifest).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError("Scene manifest not found: {}".format(path))
        return [path]
    if scene is None or not str(scene).strip():
        raise ValueError("Specify --manifest or --scene (use --scene all for all prepared scenes).")
    root = (Path(generated_root) if generated_root is not None
            else Path(__file__).resolve().parents[1] / "generated").expanduser().resolve()
    paths = sorted(root.glob("*/scene_manifest.json"))
    if str(scene).lower() == "all":
        if not paths:
            raise FileNotFoundError("No scene manifests found beneath {}".format(root))
        return paths
    records = [(path, _json(path)) for path in paths]
    query = str(scene)
    exact = [path for path, data in records
             if query in (path.parent.name, data.get("scene"), data.get("nuscenes_scene"))]
    matches = exact or [path for path, data in records
                        if query in path.parent.name or query in str(data.get("scene", ""))]
    if not matches:
        raise FileNotFoundError("No prepared scene matching {!r} beneath {}".format(query, root))
    if len(matches) != 1:
        raise ValueError("Scene {!r} is ambiguous: {}. Use the full scene name or --manifest.".format(
            query, ", ".join(path.parent.name for path in matches)))
    return matches


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation from a finite, nonzero nuScenes (w, x, y, z)."""
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Quaternion must contain four finite values in w, x, y, z order.")
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Quaternion must have nonzero finite length.")
    w, x, y, z = values / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
    ])


def _translation(record, label):
    value = np.asarray(record["translation"], dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("{} translation must contain three finite values.".format(label))
    return value


def _dataset_layout(root, version):
    """Accept a dataset root or its version-table subdirectory."""
    root = Path(root).expanduser().resolve()
    if (not version or version in (".", "..") or Path(version).name != version
            or PureWindowsPath(version).name != version or PureWindowsPath(version).drive):
        raise ValueError("nuScenes version must be a table-directory name, e.g. v1.0-mini.")
    if (root / version / "scene.json").is_file():
        return root, root / version
    if (root / "scene.json").is_file():
        # Standard nuScenes media sit beside (not inside) the version folder.
        dataset = root if (root / "samples").is_dir() else root.parent
        if root.name != version:
            raise ValueError("Table directory {} does not match requested version {}.".format(root, version))
        return dataset, root
    raise FileNotFoundError(
        "nuScenes tables were not found at {} or {}. Pass --dataroot containing samples/ "
        "and the {} table directory (or pass that table directory directly).".format(
            root / version / "scene.json", root / "scene.json", version))


def _table(path):
    records = _json(path)
    if not isinstance(records, list):
        raise ValueError("nuScenes table must contain a JSON list: {}".format(path))
    indexed = {}
    for record in records:
        token = record.get("token")
        if not token or token in indexed:
            raise ValueError("Missing or duplicate token {!r} in {}".format(token, path))
        indexed[token] = record
    return indexed


def _linked(tables, name, token, context):
    try:
        return tables[name][token]
    except KeyError:
        raise ValueError("{} references missing {} token {!r}.".format(context, name, token)) from None


def _dataset_file(root, filename, context):
    # Reject drive-relative Windows paths and backslash traversal on every OS.
    relative = Path(str(filename).replace("\\", "/"))
    if (not filename or relative.anchor or PureWindowsPath(str(filename)).drive
            or ".." in relative.parts):
        raise ValueError("{} has an unsafe dataset filename: {!r}".format(context, filename))
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError("{} sensor file escapes the dataset root: {}".format(context, path))
    if not path.is_file():
        raise FileNotFoundError("{} sensor file missing: {}. Check --dataroot and extract the "
                                "nuScenes samples archive.".format(context, path))
    return str(path)


def load_scene(manifest_path, dataroot=None, version=None):
    """Load a complete scene and reject frame-order or coordinate mismatches."""
    manifest_path = Path(manifest_path).expanduser().resolve()
    manifest = _json(manifest_path)
    coordinate_frame = manifest.get("coordinate_frame", {})
    if (not isinstance(coordinate_frame, dict)
            or coordinate_frame.get("name") != "opendrive_patch_local_metres"
            or coordinate_frame.get("unreal_flip_y") is not True):
        raise ValueError("Visualization requires manifest coordinate_frame.name="
                         "'opendrive_patch_local_metres' and unreal_flip_y=true; "
                         "other coordinate conventions cannot be safely aligned.")
    meta_value = manifest.get("trajectories", {}).get("meta")
    if not meta_value:
        raise ValueError("Manifest has no trajectories.meta: {}".format(manifest_path))
    meta_path = Path(meta_value).expanduser()
    if not meta_path.is_absolute():
        meta_path = manifest_path.parent / meta_path
    meta_path = meta_path.resolve()
    meta = _json(meta_path)
    scene_name = manifest.get("nuscenes_scene")
    map_name = manifest.get("nuscenes_map")
    if not scene_name or not map_name:
        raise ValueError("Manifest must identify nuscenes_scene and nuscenes_map.")
    if meta.get("scene") != scene_name or meta.get("map") != map_name:
        raise ValueError("Manifest scene/map identity does not match trajectories.meta.")
    full_scene = "{}_{}".format(map_name, scene_name)
    if manifest.get("scene") != full_scene:
        raise ValueError("Manifest scene {!r} does not match {}.".format(manifest.get("scene"), full_scene))
    source = manifest.get("source", {})
    root_value = dataroot if dataroot is not None else source.get("nuscenes_dataroot")
    if not root_value:
        raise ValueError("No nuScenes dataset location in manifest. Specify --dataroot.")
    root = Path(root_value).expanduser()
    if dataroot is None and not root.is_absolute():
        root = manifest_path.parent / root
    version = str(version or source.get("nuscenes_version") or "v1.0-mini")
    root, table_root = _dataset_layout(root, version)
    tables = {name: _table(table_root / (name + ".json")) for name in _TABLES}
    candidates = [record for record in tables["scene"].values() if record.get("name") == scene_name]
    if len(candidates) != 1:
        raise ValueError("Expected one {} in {} scene.json; found {}.".format(
            scene_name, table_root, len(candidates)))
    scene_record = candidates[0]
    log_path = table_root / "log.json"
    if log_path.is_file() and scene_record.get("log_token"):
        log = _linked({"log": _table(log_path)}, "log", scene_record["log_token"], scene_name)
        if log.get("location") != map_name:
            raise ValueError("nuScenes log location does not match manifest nuscenes_map.")

    samples = []
    token = scene_record.get("first_sample_token")
    seen = set()
    while token:
        if token in seen:
            raise ValueError("Cycle in {} sample chain at token {}.".format(scene_name, token))
        seen.add(token)
        sample = _linked(tables, "sample", token, scene_name)
        if sample.get("scene_token") != scene_record["token"]:
            raise ValueError("Sample {} belongs to a different scene.".format(token))
        if samples and int(sample["timestamp"]) <= int(samples[-1]["timestamp"]):
            raise ValueError("Nonmonotonic timestamps in {} sample chain.".format(scene_name))
        if sample.get("prev", "") != (samples[-1]["token"] if samples else ""):
            raise ValueError("Broken prev link in {} sample chain at {}.".format(scene_name, token))
        samples.append(sample)
        token = sample.get("next", "")
    if not samples:
        raise ValueError("Scene {} has an empty sample chain.".format(scene_name))
    if (int(scene_record.get("nbr_samples", len(samples))) != len(samples)
            or scene_record.get("last_sample_token", samples[-1]["token"]) != samples[-1]["token"]):
        raise ValueError("Scene sample-chain length or final token disagrees with scene.json.")
    trajectory = meta.get("ego_trajectory_local", [])
    if len(trajectory) != len(samples):
        raise ValueError("Reconstruction has {} ego frames but nuScenes {} has {} samples; "
                         "refusing to silently truncate or misalign.".format(
                             len(trajectory), scene_name, len(samples)))
    if "agent_frames" not in meta or not isinstance(meta["agent_frames"], list):
        raise ValueError("Reconstruction metadata must include agent_frames as a list of keyframes.")
    if len(meta["agent_frames"]) != len(samples):
        raise ValueError("Reconstruction agent-frame count does not match the sample chain.")
    for frame_index, annotations in enumerate(meta["agent_frames"]):
        if not isinstance(annotations, list):
            raise ValueError("Metadata agent_frames[{}] must be a list of annotations.".format(frame_index))
        actor_ids = set()
        for annotation in annotations:
            if (not isinstance(annotation, dict) or annotation.get("id") is None
                    or not str(annotation["id"]).strip()):
                raise ValueError("Metadata agent frame {} contains an annotation without an id.".format(frame_index))
            actor_id = str(annotation["id"])
            if actor_id in actor_ids:
                raise ValueError("Metadata agent frame {} contains duplicate actor id {!r}.".format(frame_index, actor_id))
            actor_ids.add(actor_id)
            try:
                coordinates = [float(annotation[key]) for key in ("x", "y", "yaw")]
            except (KeyError, TypeError, ValueError, OverflowError):
                coordinates = []
            if len(coordinates) != 3 or not all(math.isfinite(value) for value in coordinates):
                raise ValueError("Metadata agent frame {} actor {!r} must have finite x, y and yaw.".format(
                    frame_index, actor_id))
    for field in ("ego_keyframes", "agent_keyframes"):
        declared = manifest["trajectories"].get(field)
        if declared is not None and int(declared) != len(samples):
            raise ValueError("Manifest trajectories.{} does not match sample count.".format(field))

    channels = {sample["token"]: {} for sample in samples}
    for sd in tables["sample_data"].values():
        if sd.get("sample_token") not in channels or not sd.get("is_key_frame"):
            continue
        context = "sample_data {}".format(sd["token"])
        calibration = _linked(tables, "calibrated_sensor", sd.get("calibrated_sensor_token"), context)
        sensor = _linked(tables, "sensor", calibration.get("sensor_token"), context)
        channel = sensor.get("channel")
        if channel not in CAMERA_CHANNELS + ("LIDAR_TOP",):
            continue
        by_channel = channels[sd["sample_token"]]
        if channel in by_channel:
            raise ValueError("Duplicate keyframe {} for sample {}.".format(channel, sd["sample_token"]))
        pose = _linked(tables, "ego_pose", sd.get("ego_pose_token"), context)
        # Validate extrinsics even if only the camera mosaic will be rendered.
        quaternion_matrix(calibration["rotation"])
        quaternion_matrix(pose["rotation"])
        _translation(calibration, channel + " calibration")
        _translation(pose, channel + " ego pose")
        by_channel[channel] = (sd, calibration, pose)

    origin = meta.get("origin", {})
    origin_xy = np.asarray([origin.get("x"), origin.get("y")], dtype=np.float64)
    if not np.isfinite(origin_xy).all():
        raise ValueError("Metadata origin must contain finite x and y values.")
    frames = []
    first_timestamp = int(samples[0]["timestamp"])
    for index, sample in enumerate(samples):
        sensors = channels[sample["token"]]
        missing = [name for name in CAMERA_CHANNELS + ("LIDAR_TOP",) if name not in sensors]
        if missing:
            raise FileNotFoundError("{} sample {} is missing keyframe sensor data for {}. "
                                    "Check the selected nuScenes version/dataset.".format(
                                        scene_name, sample["token"], ", ".join(missing)))
        lidar_sd, calibration, ego_pose = sensors["LIDAR_TOP"]
        xy = _translation(ego_pose, "LiDAR ego pose")[:2] - origin_xy
        expected = trajectory[index]
        expected_xy = np.asarray([expected["x"], expected["y"]], dtype=np.float64)
        rotation = quaternion_matrix(ego_pose["rotation"])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        yaw_error = math.atan2(math.sin(yaw - float(expected["yaw"])),
                               math.cos(yaw - float(expected["yaw"])))
        if (not np.isfinite(expected_xy).all() or not math.isfinite(yaw_error)
                or np.linalg.norm(xy - expected_xy) > 0.05 or abs(yaw_error) > 0.01):
            raise ValueError("Frame {} ({}) ego pose disagrees with trajectories.meta "
                             "(position tolerance 0.05 m, yaw tolerance 0.01 rad). "
                             "Check scene, origin and keyframe order.".format(index, sample["token"]))
        cameras, camera_sd = {}, {}
        for channel in CAMERA_CHANNELS:
            sd, camera_calibration, camera_pose = sensors[channel]
            cameras[channel] = _dataset_file(root, sd.get("filename"), channel + " " + sample["token"])
            camera_sd[channel] = dict(sd, timestamp_us=int(sd["timestamp"]),
                                      calibrated_sensor=camera_calibration, ego_pose=camera_pose)
        frames.append({
            "index": index, "sample_token": sample["token"],
            "timestamp_us": int(sample["timestamp"]),
            "relative_time_s": (int(sample["timestamp"]) - first_timestamp) / 1e6,
            "cameras": cameras, "camera_sample_data": camera_sd,
            "lidar": {"path": _dataset_file(root, lidar_sd.get("filename"), "LIDAR_TOP " + sample["token"]),
                      "calibrated_sensor": calibration, "ego_pose": ego_pose,
                      "timestamp_us": int(lidar_sd["timestamp"]), "sample_data": lidar_sd},
        })
    return {"manifest_path": str(manifest_path), "manifest": manifest,
            "meta_path": str(meta_path), "meta": meta,
            "manifest_sha256": _sha256(manifest_path), "meta_sha256": _sha256(meta_path),
            "scene": full_scene, "nuscenes_scene": scene_name,
            "dataroot": str(root), "version": version, "frames": frames}


def real_lidar_points(frame):
    """Return original LiDAR xyz, north-up and relative to its ego pose (metres).

    Both calibrated sensor and ego rotations are applied in full 3D. Z is
    relative to the ego origin, not relative to the LiDAR mount or sea level.
    """
    lidar = frame["lidar"]
    if Path(lidar["path"]).stat().st_size % (5 * np.dtype(np.float32).itemsize):
        raise ValueError("nuScenes LiDAR file must contain N x 5 float32 values: {}".format(lidar["path"]))
    values = np.fromfile(lidar["path"], dtype=np.float32)
    points = values.reshape(-1, 5)[:, :3].astype(np.float64)
    calibration, pose = lidar["calibrated_sensor"], lidar["ego_pose"]
    points_ego = points @ quaternion_matrix(calibration["rotation"]).T + _translation(calibration, "LiDAR calibration")
    points_world = points_ego @ quaternion_matrix(pose["rotation"]).T + _translation(pose, "LiDAR ego pose")
    return points_world - _translation(pose, "LiDAR ego pose")


def carla_lidar_points(raw, sensor_world_matrix, ego_carla_xyz):
    """Transform CARLA N x 4 sensor returns into matching north-up RHS xyz.

    Applying the complete sensor world transform preserves mount offsets and
    arbitrary rotations. Only after centering on ego do we reverse CARLA Y.
    """
    raw = np.asarray(raw)
    matrix = np.asarray(sensor_world_matrix, dtype=np.float64)
    ego = np.asarray(ego_carla_xyz, dtype=np.float64)
    if raw.ndim != 2 or raw.shape[1] != 4:
        raise ValueError("CARLA LiDAR returns must have shape N x 4 (x, y, z, intensity).")
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Sensor world transform must be a finite 4 x 4 matrix.")
    if ego.shape != (3,) or not np.isfinite(ego).all():
        raise ValueError("CARLA ego position must contain three finite values.")
    if not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError("Sensor world transform must be affine with last row [0, 0, 0, 1].")
    points = raw[:, :3] @ matrix[:3, :3].T + matrix[:3, 3] - ego
    points[:, 1] *= -1
    return points
