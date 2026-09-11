"""Visual-only divider meshes from nuScenes nodes, not estimated lane widths.

No simulator, devkit, NumPy or Shapely dependency. Missing/NIL styles are not
painted. Style annotations apply to the outgoing edge of their named node;
an optional annotation on the final node therefore adds no geometry.
"""

import hashlib
import json
import math
import os
from collections import Counter, defaultdict
import xml.etree.ElementTree as ET

from .markings import MarkingMesh, _paint_ranges, _sample_marking, write_obj


def _read_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _sha256(path):
    with open(path, "rb") as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def _positive(value, name, allow_zero=False):
    value = float(value)
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError("%s must be finite and %s" %
                         (name, "nonnegative" if allow_zero else "positive"))
    return value


def _clip_edge(a, b, bounds):
    """Liang-Barsky clipping, including edges with both endpoints outside."""
    lo, hi = 0.0, 1.0
    for axis in (0, 1):
        delta = b[axis] - a[axis]
        minimum, maximum = bounds[axis], bounds[axis + 2]
        if abs(delta) < 1e-12:
            if not minimum <= a[axis] <= maximum:
                return None
        else:
            enter, leave = sorted(((minimum - a[axis]) / delta,
                                   (maximum - a[axis]) / delta))
            lo, hi = max(lo, enter), min(hi, leave)
            if hi <= lo + 1e-12:
                return None
    return lo, hi


def _interpolate(a, b, t):
    return tuple(x + t * (y - x) for x, y in zip(a, b))


def _style(label):
    parts = str(label).upper().split("_")
    if (len(parts) != 3 or parts[0] not in ("SINGLE", "DOUBLE") or
            parts[1] not in ("SOLID", "DASHED", "ZIGZAG") or
            parts[2] not in ("WHITE", "YELLOW")):
        return None
    return (2 if parts[0] == "DOUBLE" else 1,
            {"SOLID": "solid", "DASHED": "broken", "ZIGZAG": "zigzag"}[parts[1]],
            parts[2].lower())


def _slice_run(points, stations, start, end, step):
    """Retain every surveyed corner; subdivide long edges only."""
    result, distances = [], []
    for a, b, sa, sb in zip(points, points[1:], stations, stations[1:]):
        lo, hi = max(start, sa), min(end, sb)
        if hi - lo < 1e-9:
            continue
        count = max(1, int(math.ceil((hi - lo) / step)))
        for index in range(count + 1):
            s = lo + (hi - lo) * index / count
            if distances and abs(s - distances[-1]) < 1e-9:
                continue
            result.append(_interpolate(a, b, (s - sa) / (sb - sa)))
            distances.append(s)
    return result, distances


def _offset_path(points, offsets):
    normals = []
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        normals.append((-(b[1] - a[1]) / length, (b[0] - a[0]) / length))
    result = []
    for i, (p, offset) in enumerate(zip(points, offsets)):
        if i == 0:
            nx, ny = normals[0]
        elif i == len(points) - 1:
            nx, ny = normals[-1]
        else:
            first, second = normals[i - 1], normals[i]
            denominator = max(0.25, 1.0 + first[0] * second[0] + first[1] * second[1])
            nx, ny = ((first[0] + second[0]) / denominator,
                      (first[1] + second[1]) / denominator)
        result.append((p[0] + nx * offset, p[1] + ny * offset, p[2]))
    return result


def build_nuscenes_marking_mesh(map_data, origin, bounds, step=0.5,
                               dash_length=3.0, dash_gap=6.0, stripe_width=0.13,
                               double_separation=0.24, height_m=0.02,
                               zigzag_amplitude=0.20, zigzag_wavelength=2.0):
    """Build exact divider polylines clipped to a patch-local rectangle.

    Width/dash spacing/double separation/zigzag dimensions are explicit visual
    assumptions: nuScenes provides semantic labels, not surveyed paint sizes.
    Only exactly coincident source edges are deduplicated, never nearby lines.
    """
    for name, value in (("step", step), ("dash_length", dash_length),
                        ("stripe_width", stripe_width),
                        ("double_separation", double_separation),
                        ("zigzag_wavelength", zigzag_wavelength)):
        _positive(value, name)
    for name, value in (("dash_gap", dash_gap), ("height_m", height_m),
                        ("zigzag_amplitude", zigzag_amplitude)):
        _positive(value, name, allow_zero=True)
    if double_separation <= stripe_width:
        raise ValueError("double_separation must exceed stripe_width")
    if (len(origin) != 2 or len(bounds) != 4 or
            not all(math.isfinite(float(x)) for x in tuple(origin) + tuple(bounds)) or
            bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
        raise ValueError("invalid origin or clipping bounds")
    nodes = {r["token"]: (float(r["x"]) - origin[0],
                           float(r["y"]) - origin[1], height_m)
             for r in map_data["node"]}
    if not all(math.isfinite(v) for p in nodes.values() for v in p):
        raise ValueError("non-finite nuScenes node coordinates")
    lines = {r["token"]: r["node_tokens"] for r in map_data["line"]}
    mesh = MarkingMesh(source_kind="nuscenes")
    skipped, styles, seen = Counter(), Counter(), {}
    records_in_patch, runs, conflicts = [], [], []
    for layer in ("lane_divider", "road_divider"):
        for record in sorted(map_data.get(layer, []), key=lambda r: r["token"]):
            tokens = lines[record["line_token"]]
            annotations = {}
            for annotation in record.get(layer + "_segments", []):
                token, label = annotation["node_token"], annotation.get("segment_type")
                if token not in tokens:
                    raise ValueError("divider %s annotation is outside its line" % record["token"])
                if token in annotations and annotations[token] != label:
                    raise ValueError("conflicting styles at node " + token)
                annotations[token] = label
            station, phase, previous, current, in_patch = 0.0, 0.0, None, None, False
            for ta, tb in zip(tokens, tokens[1:]):
                a, b = nodes[ta], nodes[tb]
                length = math.hypot(b[0] - a[0], b[1] - a[1])
                if length < 1e-9:
                    continue
                label = annotations.get(ta)
                if label != previous:
                    phase, previous = station, label
                clipped = _clip_edge(a, b, bounds)
                s0 = station
                station += length
                if clipped is None:
                    current = None
                    continue
                in_patch = True
                parsed = _style(label)
                if parsed is None:
                    skipped["missing" if label is None else str(label)] += 1
                    current = None
                    continue
                lo, hi = clipped
                ca, cb = _interpolate(a, b, lo), _interpolate(a, b, hi)
                start, end = s0 + lo * length, s0 + hi * length
                key = tuple(sorted((tuple(round(x, 6) for x in ca[:2]),
                                    tuple(round(x, 6) for x in cb[:2]))))
                if key in seen:
                    mesh.duplicate_base_segments += 1
                    if seen[key] != label:
                        conflicts.append({"record": record["token"], "kept": seen[key],
                                          "skipped": label})
                    current = None
                    continue
                seen[key] = label
                mesh.accepted_base_segments += 1
                styles[label] += 1
                if (current is None or current["label"] != label or
                        abs(current["s"][-1] - start) > 1e-8 or
                        math.hypot(current["points"][-1][0] - ca[0],
                                   current["points"][-1][1] - ca[1]) > 1e-8):
                    current = {"record": record["token"], "layer": layer, "label": label,
                               "phase": phase, "points": [ca], "s": [start]}
                    runs.append(current)
                current["points"].append(cb)
                current["s"].append(end)
            if in_patch:
                records_in_patch.append({"layer": layer, "token": record["token"],
                                         "has_style_annotations": bool(annotations)})
    mesh.source_definitions = len(runs)
    for run in runs:
        count, style, color = _style(run["label"])
        start, end = run["s"][0], run["s"][-1]
        paint = _paint_ranges(start, end, "solid" if style == "zigzag" else style,
                              dash_length, dash_gap, phase_origin=run["phase"])
        if paint:
            mesh.definitions += 1
        for lo, hi in paint:
            points, distances = _slice_run(run["points"], run["s"], lo, hi,
                                          min(step, zigzag_wavelength / 16.0)
                                          if style == "zigzag" else step)
            for stripe in range(count):
                offset = (stripe - (count - 1) / 2.0) * double_separation
                offsets = [offset] * len(points)
                if style == "zigzag":
                    offsets = [offset + zigzag_amplitude * (1.0 - 4.0 * abs(
                        ((s - run["phase"]) / zigzag_wavelength) % 1.0 - 0.5))
                               for s in distances]
                mesh.add_ribbon(_offset_path(points, offsets), stripe_width, color)
    mesh.metadata = {
        "source_styles_by_clipped_edge": dict(sorted(styles.items())),
        "skipped_edges_by_style": dict(sorted(skipped.items())),
        "records_in_patch": records_in_patch,
        "exact_duplicate_style_conflicts": conflicts,
        "invented_boundary_markings": 0,
        "origin_global_m": list(origin), "clip_bounds_local_m": list(bounds),
        "visual_assumptions": {
            "stripe_width_m": stripe_width, "dash_length_m": dash_length,
            "dash_gap_m": dash_gap, "double_separation_m": double_separation,
            "height_m": height_m, "zigzag_amplitude_m": zigzag_amplitude,
            "zigzag_wavelength_m": zigzag_wavelength,
            "dash_phase": "starts at each source style run; retained through patch clipping",
            "dimensions_are_surveyed": False,
            "unknown_style_policy": "skip and report; no default paint",
        },
    }
    return mesh


def audit_road_alignment(mesh, xodr_path, tolerance=0.25):
    """Compare paint vertices with sampled driving-lane envelopes, not FBX.

    The nuScenes map is 2D. Refuse elevated/banked roads instead of silently
    laying all paint at Z=0.02. Never snap surveyed XY positions to a lane.
    """
    root = ET.parse(xodr_path).getroot()
    for p in root.findall(".//elevation") + root.findall(".//superelevation"):
        if any(abs(float(p.get(k, 0))) > 1e-9 for k in ("a", "b", "c", "d")):
            raise ValueError("nuScenes marking overlay currently requires a flat OpenDRIVE map")
    polygons, grid = [], defaultdict(list)
    for road in root.findall("road"):
        sections = road.findall("lanes/laneSection")
        for si, section in enumerate(sections):
            start = float(section.get("s"))
            end = (float(sections[si + 1].get("s")) if si + 1 < len(sections)
                   else float(road.get("length")))
            for lane in section.findall("left/lane") + section.findall("right/lane"):
                if lane.get("type") != "driving":
                    continue
                lid = int(lane.get("id"))
                inner = lid - (1 if lid > 0 else -1)
                left = _sample_marking(road, section, inner, start, end, 0, 0.5, 0)
                right = _sample_marking(road, section, lid, start, end, 0, 0.5, 0)
                for a, b, c, d in zip(left, left[1:], right[1:], right):
                    poly = (a, b, c, d)
                    index = len(polygons)
                    polygons.append(poly)
                    for gx in range(int(math.floor((min(p[0] for p in poly) - tolerance) / 10)),
                                    int(math.floor((max(p[0] for p in poly) + tolerance) / 10)) + 1):
                        for gy in range(int(math.floor((min(p[1] for p in poly) - tolerance) / 10)),
                                        int(math.floor((max(p[1] for p in poly) + tolerance) / 10)) + 1):
                            grid[(gx, gy)].append(index)

    def covered(p, poly):
        inside = False
        for a, b in zip(poly, poly[1:] + poly[:1]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length2 = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2)) if length2 else 0
            if math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy) <= tolerance:
                return True
            if ((a[1] > p[1]) != (b[1] > p[1]) and
                    p[0] < dx * (p[1] - a[1]) / dy + a[0]):
                inside = not inside
        return inside

    samples = sorted({(round(x / 100, 5), round(y / 100, 5)) for x, y, z in mesh.vertices})
    outside = [p for p in samples if not any(covered(p, polygons[i]) for i in grid.get(
        (int(math.floor(p[0] / 10)), int(math.floor(p[1] / 10))), []))]
    return {"method": "sampled OpenDRIVE driving-lane envelope; not imported FBX surface",
            "tolerance_m": tolerance, "sample_count": len(samples),
            "outside_sample_count": len(outside),
            "outside_fraction": len(outside) / max(1, len(samples)),
            "outside_examples_local_m": outside[:20],
            "xy_snapping_applied": False}


def generate_nuscenes_marking_obj(manifest_path, output, map_json=None, **kwargs):
    manifest = _read_json(manifest_path)
    frame = manifest.get("coordinate_frame", {})
    if (frame.get("unreal_centimetres_per_metre", 100.0) != 100.0 or
            frame.get("unreal_flip_y", True) is not True):
        raise ValueError("marking OBJ import requires the standard CARLA centimetre/Y-flip frame")
    source = manifest["source"]
    meta = _read_json(source["meta"])
    if (meta["map"] != manifest["nuscenes_map"] or
            meta["scene"] != manifest["nuscenes_scene"]):
        raise ValueError("manifest and source metadata identify different scenes/maps")
    if not map_json:
        dataroot = source.get("nuscenes_dataroot")
        if not dataroot:
            raise ValueError("nuScenes map source missing: supply --map-json or --source opendrive")
        map_json = os.path.join(dataroot, "maps", "expansion", manifest["nuscenes_map"] + ".json")
    if not os.path.isfile(map_json):
        raise FileNotFoundError("nuScenes map JSON missing: %s; supply --map-json or explicitly use --source opendrive" % map_json)
    origin = (meta["origin"]["x"], meta["origin"]["y"])
    patch = meta["patch"]
    bounds = (patch["x_min"] - origin[0], patch["y_min"] - origin[1],
              patch["x_max"] - origin[0], patch["y_max"] - origin[1])
    mesh = build_nuscenes_marking_mesh(_read_json(map_json), origin, bounds, **kwargs)
    mesh.metadata.update({"scene": manifest["scene"], "map": manifest["nuscenes_map"],
                          "source_map_json": os.path.abspath(map_json),
                          "source_map_sha256": _sha256(map_json),
                          "source_xodr_sha256": _sha256(source["xodr"]),
                          "road_alignment": audit_road_alignment(mesh, source["xodr"])})
    return write_obj(mesh, output)
