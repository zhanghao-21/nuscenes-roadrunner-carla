"""Generate renderable lane-marking ribbons from ASAM OpenDRIVE.

CARLA uses ``roadMark`` records for routing and lane-invasion semantics, but
those records do not create pixels on an imported persistent mesh.  This
module makes a thin visual overlay from the same records. Coordinates are
written in RoadRunner/OpenDRIVE axes but Unreal centimetres. Unreal's OBJ
importer performs the handedness conversion (including Y inversion), so the
imported mesh is placed at the world origin without an actor transform.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from collections import defaultdict
import json
import math
import os
import xml.etree.ElementTree as ET


def _f(element, name, default=0.0):
    return float(element.get(name, default))


def _poly(element, distance):
    return (_f(element, "a") + _f(element, "b") * distance
            + _f(element, "c") * distance ** 2
            + _f(element, "d") * distance ** 3)


def _active(elements, distance, start_attribute):
    candidates = [item for item in elements
                  if _f(item, start_attribute) <= distance + 1e-9]
    if not candidates:
        return None
    return max(candidates, key=lambda item: _f(item, start_attribute))


def _reference_pose(road, distance):
    geometries = road.findall("./planView/geometry")
    geometry = _active(geometries, distance, "s")
    if geometry is None:
        raise ValueError("road %s has no plan-view geometry at s=%g" %
                         (road.get("id"), distance))
    ds = max(0.0, min(distance - _f(geometry, "s"),
                      _f(geometry, "length")))
    x, y, heading = (_f(geometry, "x"), _f(geometry, "y"),
                     _f(geometry, "hdg"))
    if geometry.find("line") is not None:
        x += ds * math.cos(heading)
        y += ds * math.sin(heading)
    elif geometry.find("arc") is not None:
        curvature = _f(geometry.find("arc"), "curvature")
        if abs(curvature) < 1e-12:
            x += ds * math.cos(heading)
            y += ds * math.sin(heading)
        else:
            end_heading = heading + curvature * ds
            x += (math.sin(end_heading) - math.sin(heading)) / curvature
            y -= (math.cos(end_heading) - math.cos(heading)) / curvature
            heading = end_heading
    else:
        child = next(iter(geometry), None)
        kind = child.tag if child is not None else "empty"
        raise ValueError("unsupported OpenDRIVE geometry %s on road %s" %
                         (kind, road.get("id")))

    elevations = road.findall("./elevationProfile/elevation")
    elevation = _active(elevations, distance, "s")
    z = _poly(elevation, distance - _f(elevation, "s")) if elevation is not None else 0.0
    return x, y, z, heading


def _lane_offset(road, distance):
    offsets = road.findall("./lanes/laneOffset")
    offset = _active(offsets, distance, "s")
    return _poly(offset, distance - _f(offset, "s")) if offset is not None else 0.0


def _lane_width(lane, section_distance):
    widths = lane.findall("width")
    width = _active(widths, section_distance, "sOffset")
    if width is None:
        return 0.0
    return _poly(width, section_distance - _f(width, "sOffset"))


def _boundary_offset(road, section, lane_id, distance):
    result = _lane_offset(road, distance)
    section_distance = distance - _f(section, "s")
    if lane_id > 0:
        lanes = {int(lane.get("id")): lane
                 for lane in section.findall("./left/lane")}
        result += sum(_lane_width(lanes[index], section_distance)
                      for index in range(1, lane_id + 1) if index in lanes)
    elif lane_id < 0:
        lanes = {int(lane.get("id")): lane
                 for lane in section.findall("./right/lane")}
        result -= sum(_lane_width(lanes[-index], section_distance)
                      for index in range(1, abs(lane_id) + 1) if -index in lanes)
    return result


def _paint_ranges(start, end, style, dash_length, dash_gap, phase_origin=None):
    if style == "solid":
        return [(start, end)] if end - start > 1e-4 else []
    if style != "broken":
        return []
    if phase_origin is None:
        phase_origin = start
    result = []
    cycle = dash_length + dash_gap
    cursor = phase_origin + math.floor((start - phase_origin) / cycle) * cycle
    if cursor + dash_length <= start + 1e-9:
        cursor += cycle
    while cursor < end - 1e-6:
        dash_start = max(cursor, start)
        dash_end = min(cursor + dash_length, end)
        if dash_end - dash_start > 1e-4:
            result.append((dash_start, dash_end))
        cursor += cycle
    return result


def _stripe_styles(mark_type, double_separation,
                   preserve_double_broken=False):
    parts = mark_type.lower().split()
    if not parts or parts == ["none"]:
        return []
    if len(parts) == 1:
        return [(parts[0], 0.0)]
    # nuScenes describes many lane dividers as DOUBLE_DASHED_WHITE. Without
    # surveyed stripe spacing, expanding that semantic label with a guessed
    # offset looks like a duplicated/overlaid dash in the reconstructed map.
    # The CARLA visual default is therefore one centered dashed stripe. The
    # OpenDRIVE file is not changed, and callers can request the literal pair.
    if parts == ["broken", "broken"] and not preserve_double_broken:
        return [("broken", 0.0)]
    # OpenDRIVE orders the two styles from inside to outside.  The visual
    # overlay only needs their relative placement around the boundary.
    return [(parts[0], -double_separation / 2.0),
            (parts[1], double_separation / 2.0)]


@dataclass
class MarkingMesh:
    vertices: list[tuple[float, float, float]] = field(default_factory=list)
    faces: dict[str, list[tuple[int, int, int, int]]] = field(
        default_factory=lambda: {"white": [], "yellow": []})
    ribbons: int = 0
    definitions: int = 0
    source_definitions: int = 0
    accepted_base_segments: int = 0
    duplicate_base_segments: int = 0
    collapsed_double_broken_definitions: int = 0
    source_kind: str = "opendrive"
    metadata: dict = field(default_factory=dict)

    def add_ribbon(self, points, width_m, color):
        if len(points) < 2:
            return
        # Points remain in OpenDRIVE X/Y axes, but are still in metres. The
        # Unreal OBJ importer performs the same Y inversion as the FBX import.
        normals = []
        segment_normals = []
        for first, second in zip(points, points[1:]):
            dx, dy = second[0] - first[0], second[1] - first[1]
            length = math.hypot(dx, dy)
            if length < 1e-9:
                segment_normals.append((0.0, 1.0))
            else:
                segment_normals.append((-dy / length, dx / length))
        for index in range(len(points)):
            if index == 0:
                normal = segment_normals[0]
            elif index == len(points) - 1:
                normal = segment_normals[-1]
            else:
                nx = segment_normals[index - 1][0] + segment_normals[index][0]
                ny = segment_normals[index - 1][1] + segment_normals[index][1]
                length = math.hypot(nx, ny)
                normal = ((nx / length, ny / length) if length > 1e-9
                          else segment_normals[index])
            normals.append(normal)

        half = width_m / 2.0
        indices = []
        for point, normal in zip(points, normals):
            left = ((point[0] - normal[0] * half) * 100.0,
                    (point[1] - normal[1] * half) * 100.0,
                    point[2] * 100.0)
            right = ((point[0] + normal[0] * half) * 100.0,
                     (point[1] + normal[1] * half) * 100.0,
                     point[2] * 100.0)
            indices.append((len(self.vertices) + 1, len(self.vertices) + 2))
            self.vertices.extend((left, right))
        color = color if color in self.faces else "white"
        for first, second in zip(indices, indices[1:]):
            self.faces[color].append((first[0], second[0], second[1], first[1]))
        self.ribbons += 1


def _sample_marking(road, section, lane_id, start, end, lateral_adjust,
                    step, height_m):
    count = max(1, int(math.ceil((end - start) / step)))
    values = [start + (end - start) * index / count
              for index in range(count + 1)]
    result = []
    for distance in values:
        x, y, z, heading = _reference_pose(road, distance)
        lateral = (_boundary_offset(road, section, lane_id, distance)
                   + lateral_adjust)
        x -= math.sin(heading) * lateral
        y += math.cos(heading) * lateral
        # Do not pre-flip Y. Unreal's OBJ importer always performs the same
        # handedness conversion used for the RoadRunner FBX; pre-flipping here
        # mirrors the markings relative to RoadsNode.
        result.append((x, y, z + height_m))
    return result


def _segment_record(first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return None
    return {
        "mid": ((first[0] + second[0]) / 2.0,
                (first[1] + second[1]) / 2.0),
        "unit": (dx / length, dy / length),
        "length": length,
    }


class _BoundaryDeduplicator:
    """Suppress locally overlapping physical boundaries before dash expansion.

    nuScenes conversion emits one OpenDRIVE road per lane. A shared divider can
    therefore be encoded on two lane-road boundaries. Genuine double markings
    are expanded only after this pass, so a 30 cm duplicate tolerance cannot
    collapse their two paint stripes.
    """

    def __init__(self, tolerance=0.30, cell_size=0.50,
                 parallel_angle_degrees=15.0):
        self.tolerance = tolerance
        self.cell_size = cell_size
        self.parallel_cosine = math.cos(math.radians(parallel_angle_degrees))
        self.cells = defaultdict(list)
        self.search_radius = max(1, int(math.ceil(
            (tolerance + cell_size) / cell_size)))

    def _key(self, point):
        return (int(math.floor(point[0] / self.cell_size)),
                int(math.floor(point[1] / self.cell_size)))

    def _near_existing(self, candidate):
        key_x, key_y = self._key(candidate["mid"])
        for dx in range(-self.search_radius, self.search_radius + 1):
            for dy in range(-self.search_radius, self.search_radius + 1):
                for accepted in self.cells.get((key_x + dx, key_y + dy), []):
                    dot = abs(candidate["unit"][0] * accepted["unit"][0]
                              + candidate["unit"][1] * accepted["unit"][1])
                    if dot < self.parallel_cosine:
                        continue
                    mx = candidate["mid"][0] - accepted["mid"][0]
                    my = candidate["mid"][1] - accepted["mid"][1]
                    lateral = abs(mx * accepted["unit"][1]
                                  - my * accepted["unit"][0])
                    if lateral > self.tolerance:
                        continue
                    longitudinal = abs(mx * accepted["unit"][0]
                                       + my * accepted["unit"][1])
                    maximum = ((candidate["length"] + accepted["length"]) / 2.0
                               + self.tolerance)
                    if longitudinal <= maximum:
                        return True
        return False

    def unique_ranges(self, points, start, end):
        segment_count = len(points) - 1
        if segment_count <= 0:
            return [], 0, 0
        records = [_segment_record(first, second)
                   for first, second in zip(points, points[1:])]
        flags = [record is not None and not self._near_existing(record)
                 for record in records]
        # Index this definition only after classifying all its segments, so a
        # tight curve cannot suppress itself.
        for record, accepted in zip(records, flags):
            if record is not None and accepted:
                self.cells[self._key(record["mid"])].append(record)

        ranges = []
        index = 0
        while index < segment_count:
            if not flags[index]:
                index += 1
                continue
            first = index
            while index < segment_count and flags[index]:
                index += 1
            ranges.append((
                start + (end - start) * first / segment_count,
                start + (end - start) * index / segment_count))
        accepted_count = sum(flags)
        return ranges, accepted_count, segment_count - accepted_count


def _mark_priority(definition):
    mark_type = definition["mark"].get("type", "none").lower()
    color = definition["mark"].get("color", "white").lower()
    # Real double-divider records remain authoritative. For coincident single
    # defaults, prefer a lane divider (broken) over an artificial solid edge.
    double_rank = 0 if len(mark_type.split()) > 1 else 1
    style_rank = 0 if "broken" in mark_type else 1
    color_rank = 0 if color == "yellow" else 1
    return (double_rank, style_rank, color_rank, definition["order"])


def _collect_mark_definitions(root):
    definitions = []
    order = 0
    for road in root.findall("road"):
        road_length = _f(road, "length")
        sections = sorted(road.findall("./lanes/laneSection"),
                          key=lambda item: _f(item, "s"))
        for section_index, section in enumerate(sections):
            section_start = _f(section, "s")
            section_end = (_f(sections[section_index + 1], "s")
                           if section_index + 1 < len(sections) else road_length)
            lanes = (section.findall("./left/lane")
                     + section.findall("./center/lane")
                     + section.findall("./right/lane"))
            for lane in lanes:
                lane_id = int(lane.get("id"))
                marks = sorted(lane.findall("roadMark"),
                               key=lambda item: _f(item, "sOffset"))
                for mark_index, mark in enumerate(marks):
                    start = section_start + _f(mark, "sOffset")
                    end = (section_start + _f(marks[mark_index + 1], "sOffset")
                           if mark_index + 1 < len(marks) else section_end)
                    if _stripe_styles(mark.get("type", "none"), 0.24):
                        definitions.append({
                            "road": road, "section": section,
                            "lane_id": lane_id, "mark": mark,
                            "start": start, "end": end, "order": order,
                        })
                    order += 1
    return definitions


def build_marking_mesh(xodr_path, step=1.0, dash_length=3.0,
                       dash_gap=6.0, double_separation=0.24,
                       height_m=0.02, deduplicate=True,
                       duplicate_tolerance=0.30, dedup_step=0.50,
                       preserve_double_broken=False):
    root = ET.parse(xodr_path).getroot()
    mesh = MarkingMesh()
    definitions = _collect_mark_definitions(root)
    mesh.source_definitions = len(definitions)
    deduplicator = _BoundaryDeduplicator(tolerance=duplicate_tolerance)
    for definition in sorted(definitions, key=_mark_priority):
        road, section = definition["road"], definition["section"]
        lane_id, mark = definition["lane_id"], definition["mark"]
        start, end = definition["start"], definition["end"]
        base_points = _sample_marking(
            road, section, lane_id, start, end, 0.0, dedup_step, height_m)
        if deduplicate:
            unique_ranges, accepted, duplicates = deduplicator.unique_ranges(
                base_points, start, end)
        else:
            unique_ranges = [(start, end)]
            accepted, duplicates = len(base_points) - 1, 0
        mesh.accepted_base_segments += accepted
        mesh.duplicate_base_segments += duplicates
        if not unique_ranges:
            continue
        mesh.definitions += 1
        mark_type = mark.get("type", "none").lower()
        styles = _stripe_styles(
            mark_type, double_separation,
            preserve_double_broken=preserve_double_broken)
        if (mark_type.split() == ["broken", "broken"]
                and not preserve_double_broken):
            mesh.collapsed_double_broken_definitions += 1
        color = mark.get("color", "white").lower()
        declared_width = max(0.05, _f(mark, "width", 0.13))
        stripe_width = min(declared_width, 0.12) if len(styles) > 1 else declared_width
        for unique_start, unique_end in unique_ranges:
            for style, lateral_adjust in styles:
                for paint_start, paint_end in _paint_ranges(
                        unique_start, unique_end, style, dash_length, dash_gap,
                        phase_origin=start):
                    points = _sample_marking(
                        road, section, lane_id, paint_start, paint_end,
                        lateral_adjust, step, height_m)
                    mesh.add_ribbon(points, stripe_width, color)
    return mesh


def write_obj(mesh, obj_path):
    obj_path = os.path.abspath(obj_path)
    os.makedirs(os.path.dirname(obj_path), exist_ok=True)
    stem = os.path.splitext(os.path.basename(obj_path))[0]
    mtl_path = os.path.splitext(obj_path)[0] + ".mtl"
    with open(mtl_path, "w", encoding="ascii", newline="\n") as stream:
        stream.write("newmtl LaneMarkingWhite\nKd 0.82 0.82 0.78\nKs 0.05 0.05 0.05\nNs 8\n\n")
        stream.write("newmtl LaneMarkingYellow\nKd 0.95 0.70 0.05\nKs 0.05 0.05 0.05\nNs 8\n")
    with open(obj_path, "w", encoding="ascii", newline="\n") as stream:
        stream.write("# Generated from %s marking records\n" % mesh.source_kind)
        stream.write("mtllib %s.mtl\n" % stem)
        stream.write("o LaneMarkings\n")
        for x, y, z in mesh.vertices:
            stream.write("v %.6f %.6f %.6f\n" % (x, y, z))
        stream.write("vn 0 0 1\n")
        for color, material in (("white", "LaneMarkingWhite"),
                                ("yellow", "LaneMarkingYellow")):
            if not mesh.faces[color]:
                continue
            stream.write("usemtl %s\n" % material)
            for face in mesh.faces[color]:
                stream.write("f %d//1 %d//1 %d//1 %d//1\n" % face)
    stats = {
        "source_kind": mesh.source_kind,
        "vertices": len(mesh.vertices),
        "faces": sum(len(items) for items in mesh.faces.values()),
        "faces_by_color": {key: len(value) for key, value in mesh.faces.items()},
        "ribbons": mesh.ribbons,
        "source_road_mark_definitions": mesh.source_definitions,
        "rendered_road_mark_definitions": mesh.definitions,
        "accepted_base_segments": mesh.accepted_base_segments,
        "duplicate_base_segments_removed": mesh.duplicate_base_segments,
        "double_broken_definitions_collapsed":
            mesh.collapsed_double_broken_definitions,
        "units": "unreal_centimetres",
        "coordinate_frame": "opendrive_x_y_z; Unreal OBJ import converts Y",
    }
    if mesh.source_kind != "opendrive":
        stats["source_divider_style_runs"] = stats.pop("source_road_mark_definitions")
        stats["rendered_divider_style_runs"] = stats.pop("rendered_road_mark_definitions")
    if mesh.vertices:
        converted = [(x, -y, z) for x, y, z in mesh.vertices]
        stats["expected_unreal_bounds_cm"] = {
            "min": [min(p[i] for p in converted) for i in range(3)],
            "max": [max(p[i] for p in converted) for i in range(3)],
        }
    stats.update(mesh.metadata)
    with open(os.path.splitext(obj_path)[0] + ".json", "w", encoding="utf-8") as stream:
        json.dump(stats, stream, indent=2)
        stream.write("\n")
    return stats


def generate_marking_obj(xodr_path, obj_path, **kwargs):
    mesh = build_marking_mesh(xodr_path, **kwargs)
    if not mesh.vertices:
        raise ValueError("OpenDRIVE contains no visible roadMark geometry")
    return write_obj(mesh, obj_path)
