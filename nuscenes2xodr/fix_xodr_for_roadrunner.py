"""Post-process nuscenes2xodr output so it imports cleanly into RoadRunner.

Fixes applied per file:
  1. Header bounds recomputed from actual geometry (fixes RoadRunner's
     "Bounds of file invalid" import error - the originals declared
     south/west=0 while geometry extends negative).
  2. geoReference set to a real transverse-mercator origin at the scene
     origin (published nuScenes map origins + per-scene offset from the
     _meta.json sidecar), so aerial imagery / GIS overlays line up.
  3. Endpoint snapping: connecting roads whose start/end is within
     SNAP_TOL of their linked road's endpoint are snapped exactly onto it.
  4. Plan-view simplification: each road's 0.5 m polyline is reduced with
     Douglas-Peucker (endpoints preserved) so RoadRunner gets far fewer
     control points to manage.
  5. Lane markings: real lanes (junction="-1") whose divider data was NIL
     get standard defaults - broken white centerline, solid white edge.
     Connecting roads (junction interiors) stay unmarked.
  6. Lane-level <link> elements added to connecting-road driving lanes,
     mirroring the road-level predecessor/successor (RoadRunner and other
     tools use these for maneuver topology).
  7. Junction IDs that collide with road IDs are remapped. CARLA 0.9.15 uses
     numeric ID presence to distinguish roads from junctions; a collision
     otherwise makes every incoming lane a waypoint dead end.

Usage:
    python fix_xodr_for_roadrunner.py [--in DIR] [--out DIR]
"""

import argparse
import glob
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET

# published nuScenes map origins (lat/lon of map coordinate (0,0)),
# same table as convert_nuscenes_to_xodr.py
NUSC_REF = {
    "boston-seaport":            (42.336849169438615, -71.05785369873047),
    "singapore-onenorth":        (1.2882100868743724, 103.78475189208984),
    "singapore-hollandvillage":  (1.2993652317780957, 103.78217697143555),
    "singapore-queenstown":      (1.2782562240223188, 103.76741409301758),
}
EARTH_R = 6378137.0

SNAP_TOL = 2.0      # m: max gap to snap a connector endpoint across
DP_TOL = 0.03       # m: Douglas-Peucker lateral tolerance
BOUNDS_PAD = 10.0   # m: margin added around geometry in the header bounds


def remap_colliding_junction_ids(root):
    """Return and apply the road/junction ID collision mapping."""
    road_ids = {int(road.get("id")) for road in root.findall("road")}
    junctions = root.findall("junction")
    junction_ids = {int(junction.get("id")) for junction in junctions}
    collisions = sorted(road_ids & junction_ids)
    used = road_ids | junction_ids
    candidate = max(used, default=0) + 1
    mapping = {}
    for old_id in collisions:
        while candidate in used:
            candidate += 1
        mapping[old_id] = candidate
        used.add(candidate)
        candidate += 1
    if not mapping:
        return mapping

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
    return mapping


def local_to_latlon(map_name, x, y):
    la, lo = NUSC_REF[map_name]
    lat = la + (y / EARTH_R) * (180.0 / math.pi)
    lon = lo + (x / (EARTH_R * math.cos(math.radians(la)))) * (180.0 / math.pi)
    return lat, lon


# --------------------------------------------------------------------------- #
# plan-view geometry helpers
# --------------------------------------------------------------------------- #
def extract_polyline(road):
    """Road <planView> -> [(x, y), ...] including the final endpoint."""
    pts = []
    for g in road.find("planView").findall("geometry"):
        x, y = float(g.get("x")), float(g.get("y"))
        pts.append((x, y))
    gN = road.find("planView").findall("geometry")[-1]
    x, y = float(gN.get("x")), float(gN.get("y"))
    hdg, ln = float(gN.get("hdg")), float(gN.get("length"))
    pts.append((x + ln * math.cos(hdg), y + ln * math.sin(hdg)))
    return pts


def dp_simplify(pts, tol):
    """Douglas-Peucker; always keeps first and last points."""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i0, i1 = stack.pop()
        ax, ay = pts[i0]
        bx, by = pts[i1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        dmax, imax = -1.0, -1
        for i in range(i0 + 1, i1):
            px, py = pts[i]
            if seg2 < 1e-12:
                d = math.hypot(px - ax, py - ay)
            else:
                t = ((px - ax) * dx + (py - ay) * dy) / seg2
                t = max(0.0, min(1.0, t))
                d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d > dmax:
                dmax, imax = d, i
        if dmax > tol:
            keep[imax] = True
            stack.append((i0, imax))
            stack.append((imax, i1))
    return [p for p, k in zip(pts, keep) if k]


def rebuild_planview(road, pts):
    """Replace the road's <planView> geometries from a polyline; return length."""
    plan = road.find("planView")
    for g in list(plan):
        plan.remove(g)
    s = 0.0
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        g = ET.SubElement(plan, "geometry",
                          s=f"{s:.6f}", x=f"{x0:.6f}", y=f"{y0:.6f}",
                          hdg=f"{math.atan2(y1 - y0, x1 - x0):.6f}",
                          length=f"{seg:.6f}")
        ET.SubElement(g, "line")
        s += seg
    road.set("length", f"{s:.6f}")
    return s


# --------------------------------------------------------------------------- #
# per-file fix
# --------------------------------------------------------------------------- #
def fix_file(xodr_path, out_path, meta):
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    junction_mapping = remap_colliding_junction_ids(root)
    roads = {r.get("id"): r for r in root.findall("road")}
    stats = {"snapped": 0, "marks_center": 0, "marks_edge": 0,
             "lane_links": 0, "segs_before": 0, "segs_after": 0,
             "junction_ids": len(junction_mapping)}

    # -- extract all polylines up front (snapping needs neighbours' endpoints)
    polys = {rid: extract_polyline(r) for rid, r in roads.items()}

    # -- 3. snap connector endpoints onto their linked roads' endpoints
    for rid, road in roads.items():
        if road.get("junction") == "-1":
            continue
        link = road.find("link")
        if link is None:
            continue
        pts = polys[rid]
        pred, succ = link.find("predecessor"), link.find("successor")
        if pred is not None and pred.get("elementType") == "road":
            tgt = polys.get(pred.get("elementId"))
            if tgt:
                ref = tgt[-1] if pred.get("contactPoint") == "end" else tgt[0]
                gap = math.hypot(pts[0][0] - ref[0], pts[0][1] - ref[1])
                if 1e-6 < gap <= SNAP_TOL:
                    pts[0] = ref
                    stats["snapped"] += 1
        if succ is not None and succ.get("elementType") == "road":
            tgt = polys.get(succ.get("elementId"))
            if tgt:
                ref = tgt[0] if succ.get("contactPoint") == "start" else tgt[-1]
                gap = math.hypot(pts[-1][0] - ref[0], pts[-1][1] - ref[1])
                if 1e-6 < gap <= SNAP_TOL:
                    pts[-1] = ref
                    stats["snapped"] += 1

    # -- 4. simplify + rebuild plan views
    for rid, road in roads.items():
        stats["segs_before"] += len(polys[rid]) - 1
        simplified = dp_simplify(polys[rid], DP_TOL)
        stats["segs_after"] += len(simplified) - 1
        rebuild_planview(road, simplified)
        polys[rid] = simplified

    # -- 5. marking defaults on real lanes; 6. lane links on connectors
    for rid, road in roads.items():
        is_conn = road.get("junction") != "-1"
        section = road.find("lanes/laneSection")
        center_mark = section.find("center/lane/roadMark")
        drive_lane = section.find("right/lane")
        drive_mark = drive_lane.find("roadMark")

        if not is_conn:
            if center_mark.get("type") == "none":
                center_mark.set("type", "broken")
                center_mark.set("color", "white")
                stats["marks_center"] += 1
            if drive_mark.get("type") == "none":
                drive_mark.set("type", "solid")
                drive_mark.set("color", "white")
                stats["marks_edge"] += 1
        else:
            link = road.find("link")
            if link is not None:
                lane_link = ET.Element("link")
                pred, succ = link.find("predecessor"), link.find("successor")
                if pred is not None and pred.get("elementType") == "road":
                    ET.SubElement(lane_link, "predecessor", id="-1")
                if succ is not None and succ.get("elementType") == "road":
                    ET.SubElement(lane_link, "successor", id="-1")
                if len(lane_link):
                    drive_lane.insert(0, lane_link)  # <link> first in <lane>
                    stats["lane_links"] += 1

    # -- 1. real header bounds
    xs, ys = [], []
    for pts in polys.values():
        xs += [p[0] for p in pts]
        ys += [p[1] for p in pts]
    header = root.find("header")
    header.set("north", f"{max(ys) + BOUNDS_PAD:.6f}")
    header.set("south", f"{min(ys) - BOUNDS_PAD:.6f}")
    header.set("east", f"{max(xs) + BOUNDS_PAD:.6f}")
    header.set("west", f"{min(xs) - BOUNDS_PAD:.6f}")

    # -- 2. real geoReference from map origin + scene origin offset
    map_name = header.get("name").rsplit("_", 1)[0]
    geo = header.find("geoReference")
    if map_name in NUSC_REF and meta and "origin" in meta:
        lat0, lon0 = local_to_latlon(map_name,
                                     meta["origin"]["x"], meta["origin"]["y"])
        geo.text = (f"+proj=tmerc +lat_0={lat0:.10f} +lon_0={lon0:.10f} "
                    f"+k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")

    # -- serialize with pretty indent and CDATA-wrapped geoReference
    ET.indent(tree, space="    ")
    xml = ET.tostring(root, encoding="unicode")
    proj = geo.text
    xml = xml.replace(f"<geoReference>{proj}</geoReference>",
                      f"<geoReference><![CDATA[{proj}]]></geoReference>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write(xml)
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--in", dest="indir",
                    default=os.path.join(here, "output"))
    ap.add_argument("--out", dest="outdir",
                    default=os.path.join(here, "..", "output_roadrunner"))
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    for xodr in sorted(glob.glob(os.path.join(args.indir, "*.xodr"))):
        base = os.path.basename(xodr)
        meta_path = xodr.replace(".xodr", "_meta.json")
        meta = None
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            shutil.copy2(meta_path, os.path.join(
                args.outdir, os.path.basename(meta_path)))
        out = os.path.join(args.outdir, base)
        st = fix_file(xodr, out, meta)
        print(f"{base}")
        print(f"    snapped endpoints : {st['snapped']}")
        print(f"    marking defaults  : {st['marks_center']} center, "
              f"{st['marks_edge']} edge")
        print(f"    lane-level links  : {st['lane_links']} connector lanes")
        print(f"    junction IDs fixed: {st['junction_ids']}")
        print(f"    plan-view segments: {st['segs_before']} -> "
              f"{st['segs_after']}")
    print(f"\nDone -> {os.path.abspath(args.outdir)}")


if __name__ == "__main__":
    main()
