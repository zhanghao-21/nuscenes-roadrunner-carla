"""Create data-seeded SUMO routes from a netconvert-generated network."""

from collections import deque
from dataclasses import dataclass
import json
import math
import os
import re
import xml.etree.ElementTree as ET

from .tracks import actor_kind


@dataclass
class LaneGeometry:
    edge_id: str
    lane_id: str
    lane_index: int
    shape: list
    length: float
    allow: set
    disallow: set


@dataclass
class NetworkGeometry:
    offset: tuple
    lanes: list
    adjacency: dict


def _parse_shape(value):
    points = []
    for item in (value or "").split():
        x, y = item.split(",")[:2]
        points.append((float(x), float(y)))
    return points


def load_network(path):
    root = ET.parse(path).getroot()
    location = root.find("location")
    offset = (0.0, 0.0)
    if location is not None and location.get("netOffset"):
        offset = tuple(float(value) for value in location.get("netOffset").split(",")[:2])

    lanes = []
    normal_edges = set()
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        if not edge_id or edge_id.startswith(":") or edge.get("function") == "internal":
            continue
        normal_edges.add(edge_id)
        for lane in edge.findall("lane"):
            shape = _parse_shape(lane.get("shape"))
            if len(shape) < 2:
                continue
            lanes.append(LaneGeometry(
                edge_id=edge_id,
                lane_id=lane.get("id", "%s_0" % edge_id),
                lane_index=int(lane.get("index", 0)),
                shape=shape,
                length=float(lane.get("length", 0.0)),
                allow=set((lane.get("allow") or "").split()),
                disallow=set((lane.get("disallow") or "").split())))

    adjacency = {edge_id: set() for edge_id in normal_edges}
    for connection in root.findall("connection"):
        left, right = connection.get("from"), connection.get("to")
        if left in normal_edges and right in normal_edges:
            adjacency[left].add(right)
    return NetworkGeometry(offset, lanes, adjacency)


def _vehicle_class(kind):
    return {
        "truck": "truck",
        "bus": "bus",
        "motorcycle": "motorcycle",
        "bicycle": "bicycle",
    }.get(kind, "passenger")


def _allows(lane, vehicle_class):
    if lane.allow and vehicle_class not in lane.allow:
        return False
    return vehicle_class not in lane.disallow


def _angle_difference(left, right):
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def _project_segment(point, left, right):
    dx, dy = right[0] - left[0], right[1] - left[1]
    denom = dx * dx + dy * dy
    ratio = 0.0 if denom <= 1.0e-12 else (
        ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / denom)
    ratio = max(0.0, min(1.0, ratio))
    projected = (left[0] + ratio * dx, left[1] + ratio * dy)
    distance = math.hypot(point[0] - projected[0], point[1] - projected[1])
    return distance, ratio, math.atan2(dy, dx)


def project_to_lane(point, yaw, lane):
    best = None
    travelled = 0.0
    for left, right in zip(lane.shape, lane.shape[1:]):
        segment_length = math.hypot(right[0] - left[0], right[1] - left[1])
        distance, ratio, heading = _project_segment(point, left, right)
        result = {
            "distance": distance,
            "heading_error": _angle_difference(yaw, heading),
            "position": travelled + ratio * segment_length,
        }
        if best is None or result["distance"] < best["distance"]:
            best = result
        travelled += segment_length
    return best


def nearest_lane(network, point, yaw, kind="car", heading_weight=3.0):
    sumo_point = (point[0] + network.offset[0], point[1] + network.offset[1])
    vehicle_class = _vehicle_class(kind)
    best = None
    for lane in network.lanes:
        if not _allows(lane, vehicle_class):
            continue
        projection = project_to_lane(sumo_point, yaw, lane)
        if projection is None:
            continue
        score = projection["distance"] + heading_weight * projection["heading_error"]
        candidate = (score, lane, projection)
        if best is None or score < best[0]:
            best = candidate
    return best


def shortest_edge_path(adjacency, start, target):
    if start == target:
        return [start]
    queue = deque([(start, [start])])
    visited = {start}
    while queue:
        edge, path = queue.popleft()
        for successor in sorted(adjacency.get(edge, ())):
            if successor == target:
                return path + [successor]
            if successor not in visited:
                visited.add(successor)
                queue.append((successor, path + [successor]))
    return None


def connect_edge_sequence(adjacency, edges):
    collapsed = []
    for edge in edges:
        if not collapsed or edge != collapsed[-1]:
            collapsed.append(edge)
    if not collapsed:
        return None
    route = [collapsed[0]]
    for target in collapsed[1:]:
        bridge = shortest_edge_path(adjacency, route[-1], target)
        if bridge is None:
            return None
        route.extend(bridge[1:])
    return route


def plan_track_route(network, track, maximum_snap_distance=8.0):
    matches = []
    kind = actor_kind(track.category)
    for point in track.points:
        match = nearest_lane(network, (point.x, point.y), point.yaw, kind=kind)
        if match is None or match[2]["distance"] > maximum_snap_distance:
            continue
        matches.append(match)
    if not matches:
        return None, "no point was close enough to a SUMO lane"
    route = connect_edge_sequence(
        network.adjacency, [match[1].edge_id for match in matches])
    if route is None:
        return None, "matched edges are not connected in the SUMO network"
    first_lane, first_projection = matches[0][1], matches[0][2]
    return {
        "edges": route,
        "depart_lane": first_lane.lane_index,
        "depart_pos": max(0.0, min(first_projection["position"], first_lane.length)),
        "maximum_snap_distance": max(match[2]["distance"] for match in matches),
    }, None


def _safe_id(value):
    return re.sub(r"[^A-Za-z0-9_.:-]", "_", str(value))


def _add_vtypes(root):
    common = {
        "carFollowModel": "IDM",
        # CARLA 0.9.15's SumoSimulation unconditionally enables
        # ``--lateral-resolution 0.25``; sublane simulation requires SL2015.
        "laneChangeModel": "SL2015",
        "emergencyDecel": "9.0",
        "lcStrategic": "1.0",
        "lcCooperative": "0.6",
        "lcSpeedGain": "0.8",
        "lcKeepRight": "0.5",
    }
    definitions = {
        "nusc_car": dict(vClass="passenger", guiShape="passenger", length="4.7",
                         width="1.85", accel="2.6", decel="4.5", minGap="2.5", tau="1.2"),
        "nusc_truck": dict(vClass="truck", guiShape="truck", length="7.5",
                           width="2.5", accel="1.3", decel="3.5", minGap="3.0", tau="1.5"),
        "nusc_bus": dict(vClass="bus", guiShape="bus", length="11.0",
                         width="2.5", accel="1.2", decel="3.5", minGap="3.0", tau="1.5"),
        "nusc_motorcycle": dict(vClass="motorcycle", guiShape="motorcycle", length="2.2",
                                width="0.8", accel="3.0", decel="5.0", minGap="1.2", tau="1.0"),
        "nusc_bicycle": dict(vClass="bicycle", guiShape="bicycle", length="1.8",
                             width="0.7", accel="1.2", decel="3.0", minGap="1.0", tau="1.0"),
    }
    for type_id, values in definitions.items():
        ET.SubElement(root, "vType", {"id": type_id, **common, **values})


def _vtype_for(track):
    kind = actor_kind(track.category)
    return "nusc_%s" % (kind if kind in {"truck", "bus", "motorcycle", "bicycle"}
                         else "car")


def write_sumo_scenario(network_path, tracks, output_dir, scene_name,
                        maximum_snap_distance=8.0, excluded_actor_ids=(),
                        step_length=0.05, end_time=None):
    os.makedirs(output_dir, exist_ok=True)
    network = load_network(network_path)
    root = ET.Element("routes")
    _add_vtypes(root)
    report = {"scene": scene_name, "included": [], "skipped": []}
    excluded = set(excluded_actor_ids)

    for actor_id, track in sorted(tracks.items(), key=lambda item: item[1].start_time):
        if actor_id in excluded:
            report["skipped"].append({"id": actor_id, "reason": "CARLA authority"})
            continue
        if not track.is_vehicle:
            report["skipped"].append({"id": actor_id, "reason": "not a vehicle"})
            continue
        plan, error = plan_track_route(network, track, maximum_snap_distance)
        if error:
            report["skipped"].append({"id": actor_id, "reason": error})
            continue

        route_id = "route_%s" % _safe_id(actor_id)
        ET.SubElement(root, "route", {
            "id": route_id,
            "edges": " ".join(plan["edges"]),
        })
        attributes = {
            "id": "nusc_%s" % _safe_id(actor_id),
            "type": _vtype_for(track),
            "route": route_id,
            "depart": "%.3f" % track.start_time,
            "departLane": str(plan["depart_lane"]),
            "departPos": "%.3f" % plan["depart_pos"],
            # Starting at the recorded lane position can be close to a signal or
            # junction. A zero insertion speed is accepted there reliably; the
            # selected car-following model takes authority immediately after.
            "departSpeed": "0",
        }
        ET.SubElement(root, "vehicle", attributes)
        report["included"].append({
            "id": actor_id,
            "sumo_id": attributes["id"],
            "category": track.category,
            "depart": track.start_time,
            "recorded_initial_speed_mps": track.initial_speed,
            "mean_speed_mps": track.mean_speed,
            **plan,
        })

    if hasattr(ET, "indent"):
        ET.indent(root, space="  ")
    routes_path = os.path.join(output_dir, "routes.rou.xml")
    ET.ElementTree(root).write(routes_path, encoding="utf-8", xml_declaration=True)

    if end_time is None:
        end_time = max((track.end_time for track in tracks.values()), default=20.0) + 5.0
    config_root = ET.Element("configuration")
    input_node = ET.SubElement(config_root, "input")
    ET.SubElement(input_node, "net-file", {"value": os.path.basename(network_path)})
    ET.SubElement(input_node, "route-files", {"value": os.path.basename(routes_path)})
    time_node = ET.SubElement(config_root, "time")
    ET.SubElement(time_node, "begin", {"value": "0"})
    ET.SubElement(time_node, "end", {"value": "%.3f" % end_time})
    ET.SubElement(time_node, "step-length", {"value": "%.3f" % step_length})
    processing = ET.SubElement(config_root, "processing")
    ET.SubElement(processing, "time-to-teleport", {"value": "-1"})
    if hasattr(ET, "indent"):
        ET.indent(config_root, space="  ")
    config_path = os.path.join(output_dir, "scene.sumocfg")
    ET.ElementTree(config_root).write(config_path, encoding="utf-8", xml_declaration=True)

    report_path = os.path.join(output_dir, "route_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=False)
        stream.write("\n")
    return config_path, report_path, report
