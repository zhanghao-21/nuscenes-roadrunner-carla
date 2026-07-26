"""Extract conservative environment cues from a saved OpenStreetMap file."""

import math
import re
import xml.etree.ElementTree as ET

from .geometry import nearest_sample, point_in_oriented_box, sample_polygon_grid


EARTH_RADIUS_M = 6378137.0
NUSCENES_MAP_ORIGINS = {
    "boston-seaport": (42.336849169438615, -71.05785369873047),
    "singapore-onenorth": (1.2882100868743724, 103.78475189208984),
    "singapore-hollandvillage": (1.2993652317780957, 103.78217697143555),
    "singapore-queenstown": (1.2782562240223188, 103.76741409301758),
}

GREEN_TAGS = {
    ("leisure", "park"),
    ("leisure", "garden"),
    ("landuse", "grass"),
    ("landuse", "forest"),
    ("landuse", "recreation_ground"),
    ("natural", "wood"),
    ("natural", "grassland"),
    ("natural", "scrub"),
}


def _tags(element):
    return {tag.get("k", ""): tag.get("v", "") for tag in element.findall("tag")}


def _project(lat, lon, map_name, origin):
    lat0, lon0 = NUSCENES_MAP_ORIGINS[map_name]
    y = math.radians(lat - lat0) * EARTH_RADIUS_M
    x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return x - origin["x"], y - origin["y"]


def _is_green(tags):
    return any(tags.get(key) == value for key, value in GREEN_TAGS)


def _sign_kind(tags):
    highway = tags.get("highway", "").lower()
    if highway == "stop":
        return "stop"
    if highway == "give_way":
        return "yield"
    value = tags.get("traffic_sign", "").lower()
    if "stop" in value:
        return "stop"
    if "yield" in value or "give_way" in value:
        return "yield"
    speed = re.search(r"(?:maxspeed|speed)[^0-9]*([0-9]{2,3})", value)
    if speed:
        return "speed_limit_" + speed.group(1)
    return None


def read_osm(path, map_name, origin):
    """Return projected OSM nodes, green polygons, and supported signs."""
    if map_name not in NUSCENES_MAP_ORIGINS:
        return {"trees": [], "green_polygons": [], "signs": [], "traffic_signals": []}
    root = ET.parse(path).getroot()
    nodes = {}
    node_tags = {}
    for node in root.findall("node"):
        node_id = node.get("id")
        nodes[node_id] = _project(float(node.get("lat")), float(node.get("lon")),
                                  map_name, origin)
        node_tags[node_id] = _tags(node)

    trees = []
    signs = []
    traffic_signals = []
    for node_id, point in nodes.items():
        tags = node_tags[node_id]
        if tags.get("natural") == "tree":
            trees.append({"id": "osm_tree_" + node_id, "x": point[0], "y": point[1],
                          "source": "osm_node", "confidence": "high"})
        kind = _sign_kind(tags)
        if kind:
            signs.append({"id": "osm_sign_" + node_id, "kind": kind,
                          "x": point[0], "y": point[1],
                          "source": "osm_node", "confidence": "medium"})
        if tags.get("highway") == "traffic_signals":
            traffic_signals.append({"id": "osm_signal_" + node_id,
                                    "x": point[0], "y": point[1],
                                    "source": "osm_node", "confidence": "medium"})

    green_polygons = []
    for way in root.findall("way"):
        tags = _tags(way)
        if not _is_green(tags):
            continue
        polygon = [nodes[nd.get("ref")] for nd in way.findall("nd")
                   if nd.get("ref") in nodes]
        if len(polygon) >= 3:
            green_polygons.append({"id": "osm_green_" + way.get("id", ""),
                                   "points": polygon, "tags": tags})
    return {"trees": trees, "green_polygons": green_polygons,
            "signs": signs, "traffic_signals": traffic_signals}


def build_tree_placements(osm_data, map_name, road_samples, buildings, patch,
                          spacing=13.0, road_clearance=6.0,
                          building_clearance=2.0, max_trees=400):
    """Create deterministic trees from explicit nodes and mapped green areas."""
    x_max = patch["x_max"] - patch["x_min"]
    y_max = patch["y_max"] - patch["y_min"]
    candidates = list(osm_data["trees"])
    for area in osm_data["green_polygons"]:
        for index, point in enumerate(sample_polygon_grid(area["points"], spacing, area["id"])):
            candidates.append({
                "id": "%s_tree_%04d" % (area["id"], index),
                "x": point[0], "y": point[1],
                "source": "osm_green_area", "confidence": "medium",
            })

    result = []
    for item in candidates:
        point = (item["x"], item["y"])
        if not (-5.0 <= point[0] <= x_max + 5.0 and -5.0 <= point[1] <= y_max + 5.0):
            continue
        sample, distance = nearest_sample(point, road_samples)
        required = road_clearance + (sample[3] * 0.5 if sample else 0.0)
        if distance < required:
            continue
        if any(point_in_oriented_box(point, box, building_clearance) for box in buildings):
            continue
        phase = (sum(bytearray(item["id"].encode("utf-8"))) % 360)
        scale_phase = (sum(bytearray(reversed(item["id"].encode("utf-8")))) % 31) / 100.0
        result.append({
            "id": item["id"], "x": item["x"], "y": item["y"], "z": 0.0,
            "yaw_rad": math.radians(phase), "scale": 0.90 + scale_phase,
            "asset_family": "tropical" if map_name.startswith("singapore") else "temperate",
            "source": item["source"], "confidence": item["confidence"],
        })
        if max_trees and len(result) >= max_trees:
            break
    return result


def orient_signs(signs, road_samples, patch=None, max_road_distance=15.0):
    result = []
    x_max = y_max = None
    if patch:
        x_max = patch["x_max"] - patch["x_min"]
        y_max = patch["y_max"] - patch["y_min"]
    for sign in signs:
        if patch and not (-5.0 <= sign["x"] <= x_max + 5.0 and
                          -5.0 <= sign["y"] <= y_max + 5.0):
            continue
        sample, distance = nearest_sample((sign["x"], sign["y"]), road_samples)
        if distance > max_road_distance:
            continue
        copy = dict(sign)
        copy["z"] = 0.0
        copy["yaw_rad"] = (sample[2] + math.pi) if sample else 0.0
        copy["nearest_road_distance_m"] = distance
        result.append(copy)
    return result
