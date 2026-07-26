"""Read road samples and signal poses from OpenDRIVE 1.4 files."""

import math
import xml.etree.ElementTree as ET


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
