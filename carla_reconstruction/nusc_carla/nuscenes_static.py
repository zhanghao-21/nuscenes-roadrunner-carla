"""Extract persistent traffic-related objects from raw nuScenes tables."""

import json
import math
import os


PERSISTENT_CATEGORIES = {
    "movable_object.barrier",
    "movable_object.trafficcone",
    "movable_object.debris",
    "static_object.bicycle_rack",
}


def find_version_dir(dataroot, version="v1.0-mini"):
    """Return the directory containing the nuScenes JSON tables, if present."""
    if not dataroot:
        return None
    root = os.path.abspath(dataroot)
    for candidate in (os.path.join(root, version), root):
        if os.path.isfile(os.path.join(candidate, "scene.json")):
            return candidate
    return None


def _table(version_dir, name):
    with open(os.path.join(version_dir, name + ".json"), "r", encoding="utf-8") as stream:
        return json.load(stream)


def _quaternion_yaw(rotation):
    # nuScenes stores quaternions as [w, x, y, z].
    w, x, y, z = [float(value) for value in rotation]
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def extract_static_objects(dataroot, scene_name, origin, patch,
                           version="v1.0-mini"):
    """Extract one pose per persistent nuScenes object instance.

    Traffic cones, road barriers, debris, and bicycle racks are included. A
    single annotation is kept per instance so an object seen in many samples
    does not create duplicate Unreal actors.
    """
    version_dir = find_version_dir(dataroot, version)
    if not version_dir:
        return []

    scenes = {item["name"]: item for item in _table(version_dir, "scene")}
    scene = scenes.get(scene_name)
    if not scene:
        return []
    samples = {item["token"]: item for item in _table(version_dir, "sample")}
    annotation_rows = _table(version_dir, "sample_annotation")
    annotations = {item["token"]: item for item in annotation_rows}
    annotations_by_sample = {}
    for item in annotation_rows:
        annotations_by_sample.setdefault(item.get("sample_token", ""), []).append(item["token"])
    instances = {item["token"]: item for item in _table(version_dir, "instance")}
    categories = {item["token"]: item["name"] for item in
                  _table(version_dir, "category")}

    selected = {}
    sample_token = scene.get("first_sample_token", "")
    while sample_token:
        sample = samples.get(sample_token)
        if not sample:
            break
        annotation_tokens = sample.get("anns", annotations_by_sample.get(sample_token, []))
        for annotation_token in annotation_tokens:
            annotation = annotations.get(annotation_token)
            if not annotation:
                continue
            instance = instances.get(annotation.get("instance_token"), {})
            category = categories.get(instance.get("category_token"), "")
            if category not in PERSISTENT_CATEGORIES:
                continue
            instance_token = annotation["instance_token"]
            if instance_token not in selected:
                selected[instance_token] = (category, annotation, sample_token)
        sample_token = sample.get("next", "")

    width = float(patch["x_max"]) - float(patch["x_min"])
    height = float(patch["y_max"]) - float(patch["y_min"])
    result = []
    for instance_token, (category, annotation, first_sample) in selected.items():
        translation = annotation["translation"]
        x = float(translation[0]) - float(origin["x"])
        y = float(translation[1]) - float(origin["y"])
        if not (-5.0 <= x <= width + 5.0 and -5.0 <= y <= height + 5.0):
            continue
        size = [float(value) for value in annotation["size"]]
        result.append({
            "id": "nusc_static_" + instance_token,
            "instance_token": instance_token,
            "category": category,
            "x": x,
            "y": y,
            "z": 0.0,
            "yaw_rad": _quaternion_yaw(annotation["rotation"]),
            "width": size[0],
            "length": size[1],
            "height": size[2],
            "first_sample_token": first_sample,
            "source": "nuscenes_sample_annotation",
            "confidence": "high",
        })
    return sorted(result, key=lambda item: item["id"])
