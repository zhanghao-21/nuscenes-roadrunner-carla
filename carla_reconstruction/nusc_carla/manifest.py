"""Build the versioned contract consumed by Unreal and the runtime replayer."""

import json
import os

from . import SCHEMA_VERSION
from .nuscenes_static import extract_static_objects, find_version_dir
from .osm import build_tree_placements, orient_signs, read_osm
from .rrhd import read_building_boxes
from .xodr import read_xodr


def discover_scenes(source_root):
    result = []
    if not os.path.isdir(source_root):
        return result
    for name in sorted(os.listdir(source_root)):
        if name.endswith("_meta.json"):
            result.append(name[:-10])
    return result


def resolve_scene(query, source_root):
    scenes = discover_scenes(source_root)
    exact = [scene for scene in scenes if scene == query]
    if exact:
        return exact[0]
    matches = [scene for scene in scenes if query.lower() in scene.lower()]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError("scene %r was not found under %s" % (query, source_root))
    raise ValueError("scene query %r is ambiguous: %s" % (query, ", ".join(matches)))


def _existing(*paths):
    for path in paths:
        if path and os.path.isfile(path):
            return os.path.abspath(path)
    return None


def build_manifest(scene, source_root, tree_spacing=13.0,
                   tree_road_clearance=6.0, max_trees=400,
                   nuscenes_dataroot=None, nuscenes_version="v1.0-mini"):
    source_root = os.path.abspath(source_root)
    meta_path = _existing(os.path.join(source_root, scene + "_meta.json"))
    if not meta_path:
        raise FileNotFoundError("missing metadata for " + scene)
    with open(meta_path, "r", encoding="utf-8") as stream:
        meta = json.load(stream)

    scene_dir = os.path.join(source_root, scene)
    xodr_path = _existing(os.path.join(scene_dir, scene + "_geo.xodr"),
                          os.path.join(source_root, scene + ".xodr"))
    rrhd_path = _existing(os.path.join(scene_dir, scene + "_city.rrhd"))
    osm_path = _existing(os.path.join(source_root, scene + ".osm"))
    if not xodr_path:
        raise FileNotFoundError("missing OpenDRIVE for " + scene)

    xodr = read_xodr(xodr_path)
    buildings = read_building_boxes(rrhd_path) if rrhd_path else []
    osm_data = (read_osm(osm_path, meta["map"], meta["origin"])
                if osm_path else {"trees": [], "green_polygons": [],
                                  "signs": [], "traffic_signals": []})
    trees = build_tree_placements(
        osm_data, meta["map"], xodr["samples"], buildings, meta["patch"],
        spacing=tree_spacing, road_clearance=tree_road_clearance,
        max_trees=max_trees)
    signs = orient_signs(osm_data["signs"], xodr["samples"], meta["patch"])
    if nuscenes_dataroot is None:
        candidate = os.path.join(os.path.dirname(source_root), nuscenes_version)
        nuscenes_dataroot = candidate if find_version_dir(candidate, nuscenes_version) else None
    static_objects = extract_static_objects(
        nuscenes_dataroot, meta["scene"], meta["origin"], meta["patch"],
        version=nuscenes_version) if nuscenes_dataroot else []
    width = meta["patch"]["x_max"] - meta["patch"]["x_min"]
    height = meta["patch"]["y_max"] - meta["patch"]["y_min"]

    short_scene = meta["scene"].replace("scene-", "")
    map_asset_name = "Nusc_%s_%s" % (meta["map"].replace("-", "_"), short_scene)
    package_name = map_asset_name
    map_folder = "/Game/%s/Maps/%s" % (package_name, map_asset_name)
    target_asset_name = map_asset_name + "_Decorated"
    return {
        "schema_version": SCHEMA_VERSION,
        "scene": scene,
        "nuscenes_scene": meta["scene"],
        "nuscenes_map": meta["map"],
        "coordinate_frame": {
            "name": "opendrive_patch_local_metres",
            "unreal_centimetres_per_metre": 100.0,
            "unreal_flip_y": True,
            "unreal_yaw_sign": -1.0,
            "note": "Matches the project's OpenDRIVE-to-CARLA conversion.",
        },
        "source": {
            "root": source_root,
            "meta": meta_path,
            "xodr": xodr_path,
            "rrhd": rrhd_path,
            "osm": osm_path,
            "nuscenes_dataroot": (os.path.abspath(nuscenes_dataroot)
                                    if nuscenes_dataroot else None),
            "nuscenes_version": nuscenes_version,
        },
        "map": {
            "asset_name": map_asset_name,
            "package_name": package_name,
            "target_asset_name": target_asset_name,
            "runtime_name": map_folder + "/" + target_asset_name,
            "unreal_source_level": map_folder + "/" + map_asset_name,
            "unreal_target_level": map_folder + "/" + target_asset_name,
            "package_map_path": map_folder,
            "ground_bounds": {"x_min": -120.0, "x_max": width + 120.0,
                              "y_min": -120.0, "y_max": height + 120.0,
                              "z": -0.15},
        },
        "environment": {
            "buildings": buildings,
            "trees": trees,
            "traffic_signs": signs,
            "traffic_lights": xodr["signals"],
            "osm_traffic_signal_nodes": osm_data["traffic_signals"],
            "nuscenes_static_objects": static_objects,
        },
        "trajectories": {
            "meta": meta_path,
            "frame_dt": float(meta.get("frame_dt", 0.5)),
            "ego_keyframes": len(meta.get("ego_trajectory_local", [])),
            "agent_keyframes": len(meta.get("agent_frames", [])),
        },
        "counts": {
            "roads": len(xodr["roads"]),
            "road_samples": len(xodr["samples"]),
            "buildings": len(buildings),
            "trees": len(trees),
            "traffic_signs": len(signs),
            "traffic_lights": len(xodr["signals"]),
            "nuscenes_static_objects": len(static_objects),
        },
        "provenance": {
            "buildings": "OSM footprints processed and road-cleared by the RoadRunner stage",
            "trees": "explicit OSM tree nodes plus deterministic samples inside OSM green areas",
            "traffic_signs": "supported OSM stop/yield/speed nodes; nuScenes has no general sign layer",
            "traffic_lights": "nuScenes/OpenDRIVE signal records; physical CARLA grouping requires validation",
            "nuscenes_static_objects": "deduplicated nuScenes barriers, cones, debris, and bicycle racks",
        },
    }


def write_manifest(manifest, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=False)
        stream.write("\n")
