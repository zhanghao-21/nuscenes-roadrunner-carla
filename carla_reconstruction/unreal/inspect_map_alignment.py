"""Read-only Unreal diagnostic for road and generated-marking transforms."""

import json
import os
import traceback

import unreal


def vec(value):
    return [float(value.x), float(value.y), float(value.z)]


def rot(value):
    return [float(value.pitch), float(value.yaw), float(value.roll)]


def main():
    level = os.environ["NUSC_CARLA_INSPECT_LEVEL"]
    output = os.environ["NUSC_CARLA_INSPECT_OUTPUT"]
    if not unreal.EditorLevelLibrary.load_level(level):
        raise RuntimeError("failed to load " + level)
    records = []
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        component = actor.get_component_by_class(unreal.StaticMeshComponent)
        if component is None:
            continue
        mesh = component.get_editor_property("static_mesh")
        if mesh is None:
            continue
        path = mesh.get_path_name()
        if not any(token in path for token in ("RoadsNode", "TerrainNode", "LaneMarkings")):
            continue
        origin, extent = actor.get_actor_bounds(False)
        records.append({
            "label": actor.get_actor_label(),
            "mesh": path,
            "location_cm": vec(actor.get_actor_location()),
            "rotation_pitch_yaw_roll": rot(actor.get_actor_rotation()),
            "scale": vec(actor.get_actor_scale3d()),
            "world_bounds_origin_cm": vec(origin),
            "world_bounds_extent_cm": vec(extent),
            "material_overrides": [
                component.get_material(index).get_path_name()
                if component.get_material(index) is not None else None
                for index in range(component.get_num_materials())
            ],
        })
    with open(output, "w") as stream:
        json.dump({"level": level, "actors": records}, stream, indent=2)
        stream.write("\n")
    unreal.log("[nuScenes CARLA] wrote alignment inspection to " + output)


try:
    main()
except Exception as exc:
    unreal.log_error("[nuScenes CARLA] alignment inspection failed: " + str(exc))
    unreal.log_error(traceback.format_exc())
    raise
