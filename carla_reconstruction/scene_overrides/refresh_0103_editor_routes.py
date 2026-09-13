"""Scene-0103-only completion of stale Unreal editor junction splines.

Host Python: --prepare-reference FILE validates the installed CARLA graph.
Unreal Python: NUSC_0103_ROUTES_REFERENCE/REPORT select the reference/report;
NUSC_0103_ROUTES_APPLY=1 enables a save. Otherwise this is a read-only audit.
The caller must close the editor and back up the decorated .umap before apply.
Adds collision-disabled editor-only junction routes and short bridges to the
cached ordinary routes. All existing routes, spawn points, and map actors stay
unchanged. No XODR, mesh, lane marking, or shared pipeline is modified.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ASSET = "Nusc_boston_seaport_0103"
LEVEL = "/Game/{0}/Maps/{0}/{0}_Decorated".format(ASSET)
XODR_RELATIVE = "Content/{0}/Maps/{0}/OpenDrive/{0}_Decorated.xodr".format(ASSET)
ROUTE_PREFIX = "NSRC_0103_EditorJunction_R"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_reference(output, carla_root):
    import carla
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from tools.validate_waypoint_topology import validate
    xodr = Path(carla_root).resolve() / "Unreal/CarlaUE4" / XODR_RELATIVE
    report = validate(str(xodr))
    if not report["valid"] or report["incoming_transitions"] != 41 or report["outgoing_transitions"] != 41:
        raise RuntimeError("Expected all 41 scene-0103 junction routes to validate before refreshing editor routes")
    contents = xodr.read_text(encoding="utf-8")
    cmap = carla.Map(ASSET + "_Decorated", contents)
    connectors, road_starts = [], []
    for road in ET.fromstring(contents).findall("road"):
        road_id, length = int(road.get("id")), float(road.get("length"))
        def position(station):
            wp = cmap.get_waypoint_xodr(road_id, -1, station)
            if wp is None:
                raise RuntimeError("Missing connector waypoint")
            p = wp.transform.location
            return [p.x * 100, p.y * 100]
        road_starts.append({"road_id": road_id, "start_cm": position(0.001)})
        if road.get("junction", "-1") == "-1":
            continue
        stations = [0.001] + [float(i) for i in range(1, int(math.ceil(length))) if i < length - 0.001]
        stations.append(length - 0.001)
        connectors.append({"road_id": road_id, "start_cm": position(0.001),
                           "end_cm": position(length - 0.001),
                           "incoming_road": int(road.find("link/predecessor").get("elementId")),
                           "outgoing_road": int(road.find("link/successor").get("elementId")),
                           "points_cm": [position(s) for s in stations]})
    data = {"level": LEVEL, "xodr": str(xodr), "xodr_sha256": sha256(xodr),
            "topology": report, "road_starts": road_starts,
            "connectors": sorted(connectors, key=lambda r: r["road_id"])}
    output = Path(output).resolve()
    if output == xodr.resolve():
        raise ValueError("Reference output cannot overwrite the XODR")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print("Validated reference: " + str(output))


def missing_connectors(route_records, connectors, tolerance_cm=2.0):
    """Each declared connector must have a rendered route spanning both ends."""
    routes = [points for record in route_records for points in record["routes_cm"] if points]
    def spans(points, item):
        # A short entry/exit bridge may precede/follow the connector itself.
        starts = [i for i, p in enumerate(points) if math.hypot(
            p[0] - item["start_cm"][0], p[1] - item["start_cm"][1]) <= tolerance_cm]
        ends = [i for i, p in enumerate(points) if math.hypot(
            p[0] - item["end_cm"][0], p[1] - item["end_cm"][1]) <= tolerance_cm]
        return bool(starts and ends and min(starts) < max(ends))
    return [item["road_id"] for item in connectors if not any(spans(points, item) for points in routes)]


def match_cached_roads(records, road_starts):
    matched = {}
    for record in records:
        if len(record["routes_cm"]) != 1 or len(record["routes_cm"][0]) < 2:
            raise ValueError("Expected one ordinary spline per existing scene-0103 planner")
        points = record["routes_cm"][0]
        candidates = [r["road_id"] for r in road_starts if math.hypot(
            r["start_cm"][0] - points[0][0], r["start_cm"][1] - points[0][1]) < 2.0]
        if len(candidates) != 1 or candidates[0] in matched:
            raise ValueError("Ambiguous cached editor road; refusing to bridge")
        matched[candidates[0]] = points
    return matched


def display_points(connector, cached, height_cm):
    points = [[p[0], p[1], height_cm] for p in connector["points_cm"]]
    for road_id, index, at_start in ((connector["incoming_road"], -1, True),
                                     (connector["outgoing_road"], 0, False)):
        if road_id not in cached:
            continue
        point = cached[road_id][index]
        endpoint = points[0] if at_start else points[-1]
        gap = math.hypot(point[0] - endpoint[0], point[1] - endpoint[1])
        # Old splines were sampled every 2m and stopped before the junction.
        if gap > (205.0 if at_start else 2.0):
            raise ValueError("Unexpected editor route gap; refusing to invent a connection")
        if gap > 0.001:
            points.insert(0, list(point)) if at_start else points.append(list(point))
    return points


def route_snapshot(unreal, actors):
    result = []
    for actor in actors:
        routes = []
        for spline in actor.get_editor_property("routes"):
            if spline is None:
                continue
            points = []
            for i in range(spline.get_number_of_spline_points()):
                p = spline.get_location_at_spline_point(i, unreal.SplineCoordinateSpace.WORLD)
                points.append([p.x, p.y, p.z])
            routes.append(points)
        result.append({"path": actor.get_path_name(), "label": actor.get_actor_label(),
                       "intersection": actor.get_editor_property("is_intersection"), "routes_cm": routes,
                       "editor_only": actor.get_editor_property("is_editor_only_actor"),
                       "collision_enabled": actor.get_actor_enable_collision(),
                       "overlap_events": actor.get_editor_property("trigger_volume").get_editor_property("generate_overlap_events")})
    return sorted(result, key=lambda r: r["path"])


def unrelated_fingerprint(unreal, excluded):
    """Includes marking mesh/material identities, other actors, and spawn points."""
    records = []
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if actor.get_path_name() in excluded:
            continue
        location, rotation, scale = actor.get_actor_location(), actor.get_actor_rotation(), actor.get_actor_scale3d()
        meshes = []
        for component in actor.get_components_by_class(unreal.StaticMeshComponent):
            mesh = component.get_editor_property("static_mesh")
            meshes.append([component.get_path_name(), mesh.get_path_name() if mesh else None,
                           [component.get_material(i).get_path_name() if component.get_material(i) else None
                            for i in range(component.get_num_materials())]])
        records.append([actor.get_path_name(), actor.get_actor_label(), actor.get_class().get_path_name(),
                        [location.x, location.y, location.z], [rotation.pitch, rotation.yaw, rotation.roll],
                        [scale.x, scale.y, scale.z], sorted(meshes)])
    digest = hashlib.sha256(json.dumps(sorted(records), sort_keys=True).encode("utf-8")).hexdigest()
    return {"actor_count": len(records), "sha256": digest}


def unique_actors(items):
    # Identity-based snapshots must not double-count a route actor.
    return list({actor.get_path_name(): actor for actor in items if actor is not None}.values())


def run_unreal():
    import unreal
    reference = json.loads(Path(os.environ["NUSC_0103_ROUTES_REFERENCE"]).read_text(encoding="utf-8"))
    report_path = Path(os.environ["NUSC_0103_ROUTES_REPORT"])
    apply = os.environ.get("NUSC_0103_ROUTES_APPLY", "0") == "1"
    if reference["level"] != LEVEL or sha256(reference["xodr"]) != reference["xodr_sha256"]:
        raise RuntimeError("Wrong scene or XODR changed since offline validation")
    if len(reference["connectors"]) != 41 or not reference["topology"]["valid"]:
        raise RuntimeError("Invalid scene-0103 topology reference")
    # The native route generator searches project Content; refuse ambiguous files.
    content = Path(unreal.Paths.project_content_dir()).resolve()
    candidates = list(content.rglob(ASSET + "_Decorated.xodr"))
    if len(candidates) != 1 or candidates[0].resolve() != Path(reference["xodr"]).resolve():
        raise RuntimeError("Ambiguous installed scene-0103 XODR location")
    if not unreal.EditorLevelLibrary.load_level(LEVEL):
        raise RuntimeError("Could not load " + LEVEL)
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    open_drive = [a for a in actors if a.get_class().get_name() == "OpenDriveActor"]
    if len(open_drive) != 1:
        raise RuntimeError("Expected exactly one existing OpenDriveActor")
    old_actors = unique_actors(a for a in actors if isinstance(a, unreal.RoutePlanner))
    before = route_snapshot(unreal, old_actors)
    owned_actors = [a for a in old_actors if a.get_actor_label().startswith(ROUTE_PREFIX)]
    original_actors = [a for a in old_actors if a not in owned_actors]
    original_routes = route_snapshot(unreal, original_actors)
    existing_spawners = [a for a in actors if isinstance(a, unreal.VehicleSpawnPoint)]
    unrelated_before = unrelated_fingerprint(unreal, {a.get_path_name() for a in owned_actors})
    report = {"level": LEVEL, "applied": False, "xodr_sha256": reference["xodr_sha256"],
              "before": before, "missing_connector_routes_before": missing_connectors(before, reference["connectors"]),
              "unrelated_before": unrelated_before, "existing_spawn_points": len(existing_spawners)}
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if not apply:
        unreal.log("[0103 routes] audit only: " + str(report["missing_connector_routes_before"]))
        return
    if len(original_routes) != 31:
        raise RuntimeError("Expected scene 0103's 31 original ordinary route actors")
    cached = match_cached_roads(original_routes, reference["road_starts"])
    height = original_routes[0]["routes_cm"][0][0][2]
    plans = {c["road_id"]: display_points(c, cached, height) for c in reference["connectors"]}
    owned_by_label = {a.get_actor_label(): a for a in owned_actors}
    if len(owned_by_label) != len(owned_actors) or any(
            label not in {ROUTE_PREFIX + str(rid) for rid in plans} for label in owned_by_label):
        raise RuntimeError("Unexpected owned editor route; refusing to modify it")
    with unreal.ScopedEditorTransaction("Complete scene 0103 editor junction routes only"):
        for road_id, points in sorted(plans.items()):
            label = ROUTE_PREFIX + str(road_id)
            actor = owned_by_label.get(label)
            if actor is None:
                actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.RoutePlanner, unreal.Vector(*points[0]))
                actor.set_actor_label(label)
                actor.set_folder_path("NuScenesGenerated/EditorJunctionRoutes0103")
                actor.set_editor_property("routes", [None])
            actor.set_editor_property("is_editor_only_actor", True)
            actor.set_actor_hidden_in_game(True)
            actor.set_actor_enable_collision(False)
            actor.set_editor_property("is_intersection", True)
            actor.get_editor_property("trigger_volume").set_editor_property("generate_overlap_events", False)
            splines = list(actor.get_editor_property("routes"))
            if len(splines) != 1 or splines[0] is None:
                raise RuntimeError("Expected one registered editor junction spline")
            spline = splines[0]
            spline.set_spline_points([unreal.Vector(*p) for p in points], unreal.SplineCoordinateSpace.WORLD, True)
            for i in range(len(points)):
                spline.set_spline_point_type(i, unreal.SplinePointType.LINEAR, False)
            spline.update_spline()
            spline.set_unselected_spline_segment_color(unreal.LinearColor(1.0, 0.15, 0.15, 1.0))
            spline.set_draw_debug(True)
    new_actors = unique_actors(a for a in unreal.EditorLevelLibrary.get_all_level_actors()
                               if isinstance(a, unreal.RoutePlanner))
    after = route_snapshot(unreal, new_actors)
    owned_after = [a for a in new_actors if a.get_actor_label().startswith(ROUTE_PREFIX)]
    unrelated_after = unrelated_fingerprint(unreal, {a.get_path_name() for a in owned_after})
    missing = missing_connectors(after, reference["connectors"])
    if len(owned_after) != 41 or original_routes != route_snapshot(unreal, original_actors):
        raise RuntimeError("Original route splines changed or unexpected junction route count")
    if any(not r["editor_only"] or r["collision_enabled"] or r["overlap_events"]
           for r in after if r["label"].startswith(ROUTE_PREFIX)):
        raise RuntimeError("New editor route could affect runtime traffic; refusing to save")
    if missing or unrelated_before != unrelated_after:
        raise RuntimeError("Refusing to save: missing connectors %s; unrelated actors preserved=%s" %
                           (missing, unrelated_before == unrelated_after))
    if sha256(reference["xodr"]) != reference["xodr_sha256"]:
        raise RuntimeError("XODR unexpectedly changed")
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("Failed to save scene 0103")
    report.update({"applied": True, "after": after, "missing_connector_routes_after": missing,
                   "unrelated_after": unrelated_after, "original_route_splines_unchanged": True,
                   "editor_only_connector_routes": len(owned_after)})
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    unreal.log("[0103 routes] refreshed; all 41 junction routes present; unrelated actors unchanged")


if __name__ == "__main__":
    if os.environ.get("NUSC_0103_ROUTES_REFERENCE"):
        run_unreal()
    else:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--prepare-reference", required=True)
        parser.add_argument("--carla-root", default="C:/carla")
        args = parser.parse_args()
        prepare_reference(args.prepare_reference, args.carla_root)
