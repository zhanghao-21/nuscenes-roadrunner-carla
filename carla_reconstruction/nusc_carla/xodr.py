"""Read and prepare OpenDRIVE 1.4 files for CARLA."""

import math
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET


def remap_colliding_junction_ids(root):
    """Move junction IDs that collide with road IDs to unused values.

    OpenDRIVE gives roads and junctions separate identifier domains, but
    CARLA 0.9.15's map builder identifies a junction successor by checking
    that its numeric ID is not also a road ID. A collision therefore turns an
    intersection entrance into a dead end in ``Waypoint.next()``.
    """
    road_ids = {int(road.get("id")) for road in root.findall("road")}
    junctions = root.findall("junction")
    junction_ids = {int(junction.get("id")) for junction in junctions}
    collisions = sorted(road_ids & junction_ids)
    if not collisions:
        return {}

    used = road_ids | junction_ids
    candidate = max(used, default=0) + 1
    mapping = {}
    for old_id in collisions:
        while candidate in used:
            candidate += 1
        mapping[old_id] = candidate
        used.add(candidate)
        candidate += 1

    for junction in junctions:
        old_id = int(junction.get("id"))
        if old_id in mapping:
            junction.set("id", str(mapping[old_id]))

    for road in root.findall("road"):
        junction_id = int(road.get("junction", "-1"))
        if junction_id in mapping:
            road.set("junction", str(mapping[junction_id]))
        link = road.find("link")
        if link is None:
            continue
        for tag in ("predecessor", "successor"):
            endpoint = link.find(tag)
            if endpoint is None or endpoint.get("elementType") != "junction":
                continue
            old_id = int(endpoint.get("elementId"))
            if old_id in mapping:
                endpoint.set("elementId", str(mapping[old_id]))

    for reference in root.findall(".//junctionReference"):
        old_id = int(reference.get("junction", "-1"))
        if old_id in mapping:
            reference.set("junction", str(mapping[old_id]))
    return mapping


def write_carla_compatible_xodr(source_path, target_path):
    """Copy an XODR while repairing CARLA-incompatible junction ID clashes."""
    source_path = os.path.abspath(os.fspath(source_path))
    target_path = os.path.abspath(os.fspath(target_path))
    tree = ET.parse(source_path)
    mapping = remap_colliding_junction_ids(tree.getroot())
    same_path = os.path.normcase(source_path) == os.path.normcase(target_path)
    if not mapping:
        if not same_path:
            os.makedirs(os.path.dirname(target_path), exist_ok=True)
            shutil.copy2(source_path, target_path)
        return mapping

    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    ET.indent(tree, space="    ")
    descriptor, temporary = tempfile.mkstemp(
        prefix=os.path.basename(target_path) + ".", suffix=".tmp",
        dir=os.path.dirname(target_path))
    os.close(descriptor)
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        os.replace(temporary, target_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return mapping


def _geometry_pose(geometry, distance):
    x = float(geometry.get("x", 0.0))
    y = float(geometry.get("y", 0.0))
    heading = float(geometry.get("hdg", 0.0))
    length = float(geometry.get("length", 0.0))
    distance = max(0.0, min(length, distance))
    arc = geometry.find("arc")
    if arc is not None:
        curvature = float(arc.get("curvature", 0.0))
        if abs(curvature) > 1.0e-12:
            next_heading = heading + curvature * distance
            x += (math.sin(next_heading) - math.sin(heading)) / curvature
            y -= (math.cos(next_heading) - math.cos(heading)) / curvature
            return x, y, next_heading
    return x + math.cos(heading) * distance, y + math.sin(heading) * distance, heading


def _road_pose(geometries, station):
    if not geometries:
        return 0.0, 0.0, 0.0
    selected = geometries[0]
    for geometry in geometries:
        if float(geometry.get("s", 0.0)) <= station:
            selected = geometry
        else:
            break
    return _geometry_pose(selected, station - float(selected.get("s", 0.0)))


def read_xodr(path, sample_spacing=2.0):
    root = ET.parse(path).getroot()
    roads = []
    signals = []
    flat_samples = []
    for road in root.findall("road"):
        geometries = sorted(road.findall("./planView/geometry"),
                            key=lambda item: float(item.get("s", 0.0)))
        length = float(road.get("length", 0.0))
        width_node = road.find('.//lane[@id="-1"]/width')
        width = float(width_node.get("a", 3.5)) if width_node is not None else 3.5
        samples = []
        station = 0.0
        step = max(0.25, float(sample_spacing))
        while station < length:
            x, y, heading = _road_pose(geometries, station)
            sample = (x, y, heading, width)
            samples.append(sample)
            flat_samples.append(sample)
            station += step
        x, y, heading = _road_pose(geometries, length)
        sample = (x, y, heading, width)
        samples.append(sample)
        flat_samples.append(sample)
        roads.append({
            "id": road.get("id", ""),
            "junction": road.get("junction", "-1"),
            "length": length,
            "width": width,
            "samples": samples,
        })
        for signal in road.findall("./signals/signal"):
            signal_s = float(signal.get("s", 0.0))
            offset = float(signal.get("t", 0.0))
            cx, cy, road_heading = _road_pose(geometries, signal_s)
            x = cx - math.sin(road_heading) * offset
            y = cy + math.cos(road_heading) * offset
            h_offset = float(signal.get("hOffset", 0.0))
            signals.append({
                "id": signal.get("id", ""),
                "name": signal.get("name", ""),
                "road_id": road.get("id", ""),
                "junction": road.get("junction", "-1"),
                "x": x,
                "y": y,
                "z": float(signal.get("zOffset", 0.0)),
                "yaw_rad": road_heading + math.pi + h_offset,
                "type": signal.get("type", ""),
                "subtype": signal.get("subtype", ""),
                "dynamic": signal.get("dynamic", "no") == "yes",
                "source": "opendrive_signal",
                "confidence": "high",
            })
    return {"roads": roads, "samples": flat_samples, "signals": signals}
