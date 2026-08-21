"""Unreal Editor Python entry point for decorating a persistent CARLA level.

This file is executed by UE4Editor-Cmd.exe, not by the normal Python runtime.
Configuration is passed with NUSC_CARLA_* environment variables; see
launch_build.ps1.
"""

import hashlib
import json
import math
import os
import traceback

import unreal


PREFIX = "NSRC_"
TAG = unreal.Name("NuScenesReconstruction")


def log(message):
    unreal.log("[nuScenes CARLA] " + str(message))


def warn(message):
    unreal.log_warning("[nuScenes CARLA] " + str(message))


def required_env(name):
    value = os.environ.get(name)
    if not value:
        raise RuntimeError("missing environment variable " + name)
    return value


def read_json(path):
    with open(path, "r") as stream:
        return json.load(stream)


def choose(items, key):
    if not items:
        return None
    digest = hashlib.sha1(str(key).encode("utf-8")).digest()
    return items[int.from_bytes(digest[:4], "little") % len(items)]


def to_unreal(manifest, x, y, z, yaw_rad):
    frame = manifest["coordinate_frame"]
    scale = float(frame.get("unreal_centimetres_per_metre", 100.0))
    flip_y = -1.0 if frame.get("unreal_flip_y", True) else 1.0
    yaw_sign = float(frame.get("unreal_yaw_sign", -1.0))
    location = unreal.Vector(x * scale, y * scale * flip_y, z * scale)
    rotation = unreal.Rotator(roll=0.0, pitch=0.0,
                              yaw=math.degrees(yaw_rad) * yaw_sign)
    return location, rotation


def mark_actor(actor, label, folder, collision=True):
    actor.set_actor_label(PREFIX + label)
    actor.set_folder_path(unreal.Name("NuScenesGenerated/" + folder))
    tags = list(actor.get_editor_property("tags"))
    if TAG not in tags:
        tags.append(TAG)
        actor.set_editor_property("tags", tags)
    actor.set_actor_enable_collision(collision)


def delete_previous():
    removed = 0
    for actor in list(unreal.EditorLevelLibrary.get_all_level_actors()):
        if actor.get_actor_label().startswith(PREFIX):
            if unreal.EditorLevelLibrary.destroy_actor(actor):
                removed += 1
    log("removed %d previously generated actors" % removed)


def load_blueprint(path):
    value = unreal.EditorAssetLibrary.load_blueprint_class(path)
    if value is None:
        warn("could not load Blueprint " + path)
    return value


def load_asset(path):
    value = unreal.EditorAssetLibrary.load_asset(path)
    if value is None:
        warn("could not load asset " + path)
    return value


def spawn_blueprint(path, location, rotation):
    actor_class = load_blueprint(path)
    if actor_class is None:
        return None
    return unreal.EditorLevelLibrary.spawn_actor_from_class(
        actor_class, location, rotation, transient=False)


def spawn_asset(path, location, rotation):
    asset = load_asset(path)
    if asset is None:
        return None
    return unreal.EditorLevelLibrary.spawn_actor_from_object(
        asset, location, rotation, transient=False)


def apply_road_surface_material(manifest, catalog):
    """Replace RoadRunner's BadDefault surface on the copied level only."""
    material_path = catalog.get("materials", {}).get("road_asphalt")
    material = load_asset(material_path) if material_path else None
    if material is None:
        warn("road asphalt material is unavailable; retaining imported surface")
        return 0
    package_fragment = "/Game/%s/" % manifest["map"]["package_name"]
    changed = 0
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        component = actor.get_component_by_class(unreal.StaticMeshComponent)
        if component is None:
            continue
        mesh = component.get_editor_property("static_mesh")
        if mesh is None:
            continue
        mesh_path = mesh.get_path_name()
        if package_fragment not in mesh_path or "RoadsNode" not in mesh_path:
            continue
        slot_count = max(1, len(mesh.get_editor_property("static_materials")))
        for slot in range(slot_count):
            component.set_material(slot, material)
        changed += 1
        log("applied asphalt to %s (%s)" % (actor.get_actor_label(), mesh_path))
    if changed == 0:
        warn("could not find the imported RoadsNode actor in the copied level")
    return changed


def import_lane_marking_asset(manifest, obj_path):
    if not os.path.isfile(obj_path):
        raise RuntimeError("lane-marking OBJ not found: " + obj_path)
    asset_name = manifest["map"]["asset_name"] + "_LaneMarkings"
    destination = "/Game/%s/Static/Marking/%s" % (
        manifest["map"]["package_name"], manifest["map"]["asset_name"])

    options = unreal.FbxImportUI()
    options.set_editor_property("import_as_skeletal", False)
    options.set_editor_property("import_animations", False)
    options.set_editor_property("import_materials", True)
    options.set_editor_property("import_textures", False)
    static_data = options.get_editor_property("static_mesh_import_data")
    static_data.set_editor_property("combine_meshes", True)
    static_data.set_editor_property("convert_scene", False)
    static_data.set_editor_property("convert_scene_unit", False)
    static_data.set_editor_property("generate_lightmap_u_vs", False)

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", obj_path)
    task.set_editor_property("destination_path", destination)
    task.set_editor_property("destination_name", asset_name)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    task.set_editor_property("options", options)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    object_paths = list(task.get_editor_property("imported_object_paths"))
    expected = destination + "/" + asset_name
    mesh = load_asset(expected)
    if mesh is None:
        candidates = [path for path in object_paths
                      if isinstance(load_asset(path), unreal.StaticMesh)]
        mesh = load_asset(candidates[0]) if candidates else None
    if mesh is None:
        raise RuntimeError("Unreal did not import a lane-marking StaticMesh from " + obj_path)
    log("imported lane-marking mesh %s" % mesh.get_path_name())
    return mesh


def add_lane_markings(manifest, catalog, mesh):
    actor = unreal.EditorLevelLibrary.spawn_actor_from_object(
        mesh, unreal.Vector(), unreal.Rotator(), transient=False)
    if actor is None:
        raise RuntimeError("failed to spawn imported lane-marking mesh")
    component = actor.get_component_by_class(unreal.StaticMeshComponent)
    if component is None:
        raise RuntimeError("lane-marking actor has no StaticMeshComponent")

    materials = catalog.get("materials", {})
    white = load_asset(materials.get("lane_marking_white"))
    yellow = load_asset(materials.get("lane_marking_yellow"))
    static_materials = list(mesh.get_editor_property("static_materials"))
    for index, slot in enumerate(static_materials):
        slot_name = str(slot.get_editor_property("material_slot_name"))
        chosen = yellow if "yellow" in slot_name.lower() else white
        if chosen is not None:
            component.set_material(index, chosen)
            log("lane-marking slot %s -> %s" % (slot_name, chosen.get_path_name()))
    component.set_editor_property("receives_decals", False)
    mark_actor(actor, "LaneMarkings_OpenDRIVE", "LaneMarkings", collision=False)
    return 1


def building_group(building):
    area = 4.0 * building["half_x"] * building["half_y"]
    if building["height"] >= 28.0 or area >= 1500.0:
        return "tall"
    if building["height"] >= 13.0 or area >= 450.0:
        return "medium"
    return "small"


def fit_building(actor, manifest, building, rotation):
    """Non-uniformly fit a CARLA building Blueprint to an OSM-derived box."""
    _, extent = actor.get_actor_bounds(False)
    target_x = max(100.0, 200.0 * building["half_x"])
    target_y = max(100.0, 200.0 * building["half_y"])
    target_z = max(300.0, 100.0 * building["height"])
    if extent.x < 1.0 or extent.y < 1.0 or extent.z < 1.0:
        warn("zero bounds for %s; leaving its native scale" % building["id"])
    else:
        actor.set_actor_scale3d(unreal.Vector(
            target_x / (2.0 * extent.x),
            target_y / (2.0 * extent.y),
            target_z / (2.0 * extent.z)))
    actor.set_actor_rotation(rotation, True)
    base_location, _ = to_unreal(manifest, building["x"], building["y"], 0.0, 0.0)
    bounds_origin, bounds_extent = actor.get_actor_bounds(False)
    base_location.z += bounds_extent.z - bounds_origin.z
    actor.set_actor_location(base_location, False, True)


def add_buildings(manifest, catalog):
    added = 0
    groups = catalog["buildings"]
    for building in manifest["environment"].get("buildings", []):
        group = building_group(building)
        asset_path = choose(groups.get(group, []), building["id"])
        location, rotation = to_unreal(
            manifest, building["x"], building["y"], 0.0, building["yaw_rad"])
        actor = spawn_blueprint(asset_path, location, unreal.Rotator()) if asset_path else None
        if actor is None:
            continue
        fit_building(actor, manifest, building, rotation)
        mark_actor(actor, "Building_" + building["id"], "Buildings")
        added += 1
    log("added %d buildings" % added)
    return added


def add_trees(manifest, catalog):
    added = 0
    for tree in manifest["environment"].get("trees", []):
        family = tree.get("asset_family", "temperate")
        asset_path = choose(catalog["trees"].get(family, []), tree["id"])
        location, rotation = to_unreal(
            manifest, tree["x"], tree["y"], tree.get("z", 0.0), tree["yaw_rad"])
        actor = spawn_asset(asset_path, location, rotation) if asset_path else None
        if actor is None:
            continue
        scale = float(tree.get("scale", 1.0))
        actor.set_actor_scale3d(unreal.Vector(scale, scale, scale))
        mark_actor(actor, "Tree_" + tree["id"], "Vegetation")
        added += 1
    log("added %d trees" % added)
    return added


def add_signs(manifest, catalog):
    added = 0
    assets = catalog.get("traffic_signs", {})
    for sign in manifest["environment"].get("traffic_signs", []):
        asset_path = assets.get(sign["kind"])
        if not asset_path:
            warn("no asset mapping for traffic sign " + sign["kind"])
            continue
        location, rotation = to_unreal(
            manifest, sign["x"], sign["y"], sign.get("z", 0.0), sign["yaw_rad"])
        actor = spawn_blueprint(asset_path, location, rotation)
        if actor is None:
            continue
        mark_actor(actor, "TrafficSign_" + sign["id"], "TrafficSigns")
        added += 1
    log("added %d OSM-supported traffic signs" % added)
    return added


def fit_static_object(actor, manifest, item, rotation):
    """Fit a CARLA prop to a nuScenes 3D annotation box and ground it."""
    _, extent = actor.get_actor_bounds(False)
    if extent.x >= 1.0 and extent.y >= 1.0 and extent.z >= 1.0:
        actor.set_actor_scale3d(unreal.Vector(
            max(5.0, 100.0 * item["length"]) / (2.0 * extent.x),
            max(5.0, 100.0 * item["width"]) / (2.0 * extent.y),
            max(5.0, 100.0 * item["height"]) / (2.0 * extent.z)))
    actor.set_actor_rotation(rotation, True)
    location, _ = to_unreal(manifest, item["x"], item["y"], 0.0, 0.0)
    bounds_origin, bounds_extent = actor.get_actor_bounds(False)
    location.z += bounds_extent.z - bounds_origin.z
    actor.set_actor_location(location, False, True)


def add_nuscenes_static_objects(manifest, catalog):
    added = 0
    skipped = 0
    assets = catalog.get("nuscenes_static_objects", {})
    for item in manifest["environment"].get("nuscenes_static_objects", []):
        candidates = assets.get(item["category"], [])
        asset_path = choose(candidates, item["id"])
        if not asset_path:
            warn("no CARLA prop mapping for " + item["category"])
            skipped += 1
            continue
        location, rotation = to_unreal(
            manifest, item["x"], item["y"], 0.0, item["yaw_rad"])
        actor = spawn_asset(asset_path, location, rotation)
        if actor is None:
            skipped += 1
            continue
        fit_static_object(actor, manifest, item, rotation)
        mark_actor(actor, "Static_" + item["id"], "NuScenesStatic")
        added += 1
    log("added %d nuScenes static objects; skipped %d without a usable asset" %
        (added, skipped))
    return added


def add_traffic_lights(manifest, catalog):
    if os.environ.get("NUSC_CARLA_PLACE_TRAFFIC_LIGHTS", "0") != "1":
        log("traffic-light placement disabled; OpenDRIVE signals remain authoritative")
        return 0
    raise RuntimeError(
        "traffic-light Blueprint placement is disabled until CARLA junction "
        "groups/controllers are generated; ungrouped BP_TrafficLightNew actors "
        "crash WorldObserver during Python API connections")


def prepare_level(source_level, target_level):
    allow_in_place = os.environ.get("NUSC_CARLA_ALLOW_IN_PLACE", "0") == "1"
    if source_level == target_level:
        if not allow_in_place:
            raise RuntimeError("refusing in-place edit; use a distinct target level")
        if not unreal.EditorLevelLibrary.load_level(source_level):
            raise RuntimeError("failed to load " + source_level)
        return
    if unreal.EditorAssetLibrary.does_asset_exist(target_level):
        if not unreal.EditorLevelLibrary.load_level(target_level):
            raise RuntimeError("failed to load existing target " + target_level)
        return
    if not unreal.EditorLevelLibrary.new_level_from_template(target_level, source_level):
        raise RuntimeError("failed to create %s from %s" % (target_level, source_level))


def main():
    manifest_path = required_env("NUSC_CARLA_MANIFEST")
    catalog_path = required_env("NUSC_CARLA_ASSET_CATALOG")
    source_level = required_env("NUSC_CARLA_SOURCE_LEVEL")
    target_level = required_env("NUSC_CARLA_TARGET_LEVEL")
    manifest = read_json(manifest_path)
    catalog = read_json(catalog_path)
    log("building %s into %s" % (manifest["scene"], target_level))
    prepare_level(source_level, target_level)
    marking_mesh = import_lane_marking_asset(
        manifest, required_env("NUSC_CARLA_MARKINGS_OBJ"))
    with unreal.ScopedEditorTransaction("Rebuild nuScenes environment"):
        delete_previous()
        counts = {
            "road_surfaces": apply_road_surface_material(manifest, catalog),
            "lane_markings": add_lane_markings(manifest, catalog, marking_mesh),
            "buildings": add_buildings(manifest, catalog),
            "trees": add_trees(manifest, catalog),
            "traffic_signs": add_signs(manifest, catalog),
            "nuscenes_static_objects": add_nuscenes_static_objects(manifest, catalog),
            "traffic_lights": add_traffic_lights(manifest, catalog),
        }
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("Unreal failed to save the current level")
    result_path = required_env("NUSC_CARLA_RESULT")
    with open(result_path, "w") as stream:
        json.dump({"status": "success", "target_level": target_level,
                   "counts": counts}, stream, indent=2)
        stream.write("\n")
    log("saved %s with %s" % (target_level, counts))


try:
    main()
except Exception as exc:
    unreal.log_error("[nuScenes CARLA] build failed: " + str(exc))
    unreal.log_error(traceback.format_exc())
    raise
