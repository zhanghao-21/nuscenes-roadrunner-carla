"""
Convert a region of a nuScenes vector map into an OpenDRIVE (.xodr) file that
CARLA 0.9.15 can ingest via `generate_opendrive_world`.

The region is defined by the ego-vehicle trajectory of a single nuScenes scene
(default: the first scene of v1.0-mini, "scene-0061", on map "singapore-onenorth"),
padded by a configurable margin.

Pipeline
--------
1. Resolve the scene -> log -> map; walk the scene's samples to get the ego
   trajectory and its (padded) bounding box = the conversion "patch".
2. Select all `lane` and `lane_connector` records intersecting the patch.
3. Discretize each lane/connector centerline (arcline geometry) into a polyline,
   shifted so the patch corner sits near the local origin (better numerics).
4. Estimate a per-lane width from its polygon (area / centerline length).
5. Build a drivable topology:
      lane            -> OpenDRIVE <road> (not in a junction)
      lane_connector  -> OpenDRIVE <road> belonging to a <junction>
   nuScenes connectors are strictly 1-incoming / 1-outgoing and always connect
   lane -> lane, so connectors that share an incoming OR outgoing lane are
   unioned into the same junction. This guarantees every lane has at most one
   junction on each end.
6. Emit OpenDRIVE 1.4 XML and a JSON sidecar (coordinate offset + ego start /
   trajectory) used by the CARLA loader to place the ego.

This script is CARLA-independent and runs on any Python with nuscenes-devkit,
numpy and shapely installed (e.g. your existing Python 3.12).
"""

import argparse
import json
import math
import os
import time
import urllib.request
import urllib.parse
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET

from nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.map_expansion import arcline_path_utils


# --------------------------------------------------------------------------- #
# OSM georeferencing (published nuScenes map origins) -> real traffic signals
# --------------------------------------------------------------------------- #
NUSC_REF = {
    "boston-seaport":            (42.336849169438615, -71.05785369873047),
    "singapore-onenorth":        (1.2882100868743724, 103.78475189208984),
    "singapore-hollandvillage":  (1.2993652317780957, 103.78217697143555),
    "singapore-queenstown":      (1.2782562240223188, 103.76741409301758),
}
EARTH_R = 6378137.0
OVERPASS_MIRRORS = ["https://overpass-api.de/api/interpreter",
                    "https://overpass.kumi.systems/api/interpreter"]


def local_to_latlon(loc, x, y):
    la, lo = NUSC_REF[loc]
    lat = la + (y / EARTH_R) * (180.0 / math.pi)
    lon = lo + (x / (EARTH_R * math.cos(math.radians(la)))) * (180.0 / math.pi)
    return lat, lon


def latlon_to_local(loc, lat, lon):
    la, lo = NUSC_REF[loc]
    y = (lat - la) * (math.pi / 180.0) * EARTH_R
    x = (lon - lo) * (math.pi / 180.0) * EARTH_R * math.cos(math.radians(la))
    return x, y


def fetch_osm_traffic_signals(loc, patch, pad_deg=0.0006):
    """Return real OSM traffic_signals as nuScenes (x, y); None if fetch fails."""
    if loc not in NUSC_REF:
        return None
    lls = [local_to_latlon(loc, patch[0], patch[1]),
           local_to_latlon(loc, patch[2], patch[3])]
    lats = [a for a, b in lls]
    lons = [b for a, b in lls]
    s, w = min(lats) - pad_deg, min(lons) - pad_deg
    n, e = max(lats) + pad_deg, max(lons) + pad_deg
    q = ('[out:json][timeout:60];'
         '(node["highway"="traffic_signals"](%f,%f,%f,%f););out geom;'
         % (s, w, n, e))
    data = urllib.parse.urlencode({"data": q}).encode()
    for attempt in range(3):
        for url in OVERPASS_MIRRORS:
            try:
                req = urllib.request.Request(
                    url, data=data, headers={"User-Agent": "nuscenes-osm/1.0"})
                els = json.loads(urllib.request.urlopen(
                    req, timeout=90).read().decode())["elements"]
                return [latlon_to_local(loc, el["lat"], el["lon"])
                        for el in els if el.get("type") == "node"]
            except Exception as ex:
                print("      OSM signal fetch retry (%s): %s"
                      % (url.split("/")[2], repr(ex)[:50]))
        time.sleep(4)
    return None


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #
class UnionFind:
    """Minimal union-find over hashable items (used to cluster connectors)."""

    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        root = x
        while self.parent[root] != root:
            root = self.parent[root]
        # path compression
        while self.parent[x] != root:
            self.parent[x], x = root, self.parent[x]
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def heading(p0, p1):
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def project_to_polyline(pts, p):
    """Project point p onto a polyline; return (s along line, signed t, left+)."""
    px, py = p
    best = (float("inf"), 0.0, 0.0)
    cum = 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        seglen = math.sqrt(seg2)
        if seg2 < 1e-9:
            continue
        u = clamp(((px - ax) * dx + (py - ay) * dy) / seg2, 0.0, 1.0)
        cx, cy = ax + u * dx, ay + u * dy
        d = (px - cx) ** 2 + (py - cy) ** 2
        if d < best[0]:
            s = cum + u * seglen
            t = (dx * (py - ay) - dy * (px - ax)) / seglen   # >0 = left of dir
            best = (d, s, t)
        cum += seglen
    return best[1], best[2]


# --------------------------------------------------------------------------- #
# Step 1 - resolve scene + ego trajectory / patch
# --------------------------------------------------------------------------- #
def quat_yaw(rotation):
    """Yaw (rad) from a nuScenes quaternion (w, x, y, z)."""
    w, qx, qy, qz = rotation
    return math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def get_scene_ego_trajectory(nusc, scene):
    """Return list of (x, y, yaw) ego poses (global/map frame) along the scene."""
    poses = []
    sample_token = scene["first_sample_token"]
    while sample_token:
        sample = nusc.get("sample", sample_token)
        sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        ego = nusc.get("ego_pose", sd["ego_pose_token"])
        poses.append((ego["translation"][0], ego["translation"][1],
                      quat_yaw(ego["rotation"])))
        sample_token = sample["next"]
    return poses


def compute_building_footprints(nmap, patch, origin, setback=3.0,
                                cell=14.0, min_area=120.0):
    """Synthesize building footprints in the 'negative space' (blocks) of the patch.

    nuScenes has no building layer, so we take the patch rectangle minus all
    drivable/road/walkway/parking/crossing polygons, shrink the leftover blocks,
    and tile axis-aligned building boxes inside them. Returns a list of dicts
    with local-frame centre x/y, footprint sx/sy, yaw=0 and a height.
    """
    from shapely.geometry import box, Point
    from shapely.ops import unary_union

    ox, oy = origin
    occupied = []
    layers = ["drivable_area", "road_segment", "walkway", "carpark_area",
              "ped_crossing", "lane"]
    recs = nmap.get_records_in_patch(patch, layers, mode="intersect")
    for layer in layers:
        for tok in recs.get(layer, []):
            rec = nmap.get(layer, tok)
            ptok = rec.get("polygon_token")
            if not ptok:
                continue
            try:
                occupied.append(nmap.extract_polygon(ptok))
            except Exception:
                continue
    if not occupied:
        return []
    used = unary_union(occupied).buffer(2.0)
    region = box(patch[0], patch[1], patch[2], patch[3])
    blocks = region.difference(used)

    polys = list(getattr(blocks, "geoms", [blocks]))
    buildings = []
    for blk in polys:
        inner = blk.buffer(-setback)
        if inner.is_empty or inner.area < min_area:
            continue
        for sub in getattr(inner, "geoms", [inner]):
            minx, miny, maxx, maxy = sub.bounds
            gx = minx + cell / 2.0
            while gx < maxx:
                gy = miny + cell / 2.0
                while gy < maxy:
                    if sub.contains(Point(gx, gy).buffer(cell * 0.35)):
                        h = 8.0 + ((int(gx) * 7 + int(gy) * 13) % 5) * 4.0
                        buildings.append({
                            "x": gx - ox, "y": gy - oy,
                            "sx": cell * 0.8, "sy": cell * 0.8,
                            "yaw": 0.0, "height": h,
                        })
                    gy += cell
                gx += cell
    return buildings


def get_scene_agent_frames(nusc, scene, origin, include_static=False):
    """Per-sample list of other agents (local frame), aligned with the ego trajectory.

    Each frame is a list of dicts: id, category, x, y, yaw, wlh.
    By default only dynamic agents (vehicle.* / human.pedestrian.*) are kept.
    """
    ox, oy = origin
    frames = []
    sample_token = scene["first_sample_token"]
    while sample_token:
        sample = nusc.get("sample", sample_token)
        agents = []
        for ann_token in sample["anns"]:
            a = nusc.get("sample_annotation", ann_token)
            cat = a["category_name"]
            dynamic = cat.startswith("vehicle.") or cat.startswith("human.pedestrian.")
            if not dynamic and not include_static:
                continue
            agents.append({
                "id": a["instance_token"],
                "category": cat,
                "x": a["translation"][0] - ox,
                "y": a["translation"][1] - oy,
                "yaw": quat_yaw(a["rotation"]),
                "wlh": a["size"],   # width, length, height
            })
        frames.append(agents)
        sample_token = sample["next"]
    return frames


def trajectory_patch(poses, padding):
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    return (min(xs) - padding, min(ys) - padding,
            max(xs) + padding, max(ys) + padding)


# --------------------------------------------------------------------------- #
# Step 3/4 - geometry + width
# --------------------------------------------------------------------------- #
def lane_width(nmap, token, length, default=3.5):
    """Estimate lane width as polygon_area / centerline_length, clamped."""
    rec = nmap_record(nmap, token)
    if rec is None or length <= 1e-6:
        return default
    try:
        poly = nmap.extract_polygon(rec["polygon_token"])
        w = poly.area / length
        return clamp(w, 2.0, 6.0)
    except Exception:
        return default


def nmap_record(nmap, token):
    for layer in ("lane", "lane_connector"):
        try:
            return nmap.get(layer, token)
        except KeyError:
            continue
    return None


def _dominant_segtype(segments):
    """Most common non-NIL segment_type among a lane's divider segments."""
    counts = {}
    for s in segments or []:
        t = s.get("segment_type")
        if t and t != "NIL":
            counts[t] = counts.get(t, 0) + 1
    return max(counts, key=counts.get) if counts else None


def lane_markings(nmap, token):
    """Return (left_mark, right_mark) for a lane from its divider segment types.

    Connectors (inside junctions) and lanes without divider data get no center
    marking and a faint right edge, matching how intersections look on the map.
    """
    rec = nmap_record(nmap, token)
    if rec is None or "left_lane_divider_segments" not in rec:
        # lane_connector: leave junction interiors unmarked
        return (("none", "white"), ("none", "white"))
    left = roadmark_from_segtype(
        _dominant_segtype(rec.get("left_lane_divider_segments")))
    right = roadmark_from_segtype(
        _dominant_segtype(rec.get("right_lane_divider_segments")))
    return (left, right)


def discretize(nmap, token, resolution):
    """Return a list of (x, y, yaw) centerline samples for a lane/connector."""
    try:
        path = nmap.get_arcline_path(token)
        pts = arcline_path_utils.discretize_lane(path, resolution_meters=resolution)
        return [(p[0], p[1], p[2]) for p in pts]
    except Exception:
        return []


# --------------------------------------------------------------------------- #
# OpenDRIVE emission
# --------------------------------------------------------------------------- #
def build_planview(road_el, pts, origin):
    """Emit <planView> as a chain of straight <line> geometries; return length."""
    plan = ET.SubElement(road_el, "planView")
    s = 0.0
    ox, oy = origin
    # filter out zero-length steps
    clean = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-6:
            clean.append(p)
    for i in range(len(clean) - 1):
        p0, p1 = clean[i], clean[i + 1]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        hdg = heading(p0, p1)
        g = ET.SubElement(plan, "geometry",
                          s=f"{s:.6f}",
                          x=f"{p0[0] - ox:.6f}",
                          y=f"{p0[1] - oy:.6f}",
                          hdg=f"{hdg:.6f}",
                          length=f"{seg_len:.6f}")
        ET.SubElement(g, "line")
        s += seg_len
    return s


def roadmark_from_segtype(segtype, fallback=("broken", "white")):
    """Map a nuScenes lane_divider segment_type to (OpenDRIVE roadMark type, color)."""
    if not segtype or segtype == "NIL":
        return ("none", "white")
    s = segtype.upper()
    color = "yellow" if "YELLOW" in s else "white"
    if "DASHED" in s:
        base = "broken"
    elif "ZIGZAG" in s:
        base = "solid"          # zig-zag marks (near crossings) ~ approximate as solid
    elif "SOLID" in s:
        base = "solid"
    else:
        return fallback
    typ = f"{base} {base}" if "DOUBLE" in s else base
    return (typ, color)


def _road_mark(parent, mark):
    typ, color = mark
    ET.SubElement(parent, "roadMark", sOffset="0.0", type=typ,
                  weight="standard", color=color, width="0.13",
                  laneChange="both")


def add_lanes_section(road_el, width, left_mark=None, right_mark=None):
    """One driving lane (id -1) centered on the reference line via laneOffset.

    left_mark / right_mark: (roadMark type, color) for the lane's left (center
    line) and right (outer) boundaries; default broken / solid white.
    """
    left_mark = left_mark or ("broken", "white")
    right_mark = right_mark or ("solid", "white")
    lanes = ET.SubElement(road_el, "lanes")
    # shift the lane reference so the single driving lane straddles the centerline
    ET.SubElement(lanes, "laneOffset", s="0.0", a=f"{width / 2.0:.6f}",
                  b="0.0", c="0.0", d="0.0")
    ls = ET.SubElement(lanes, "laneSection", s="0.0")

    center = ET.SubElement(ls, "center")
    c_lane = ET.SubElement(center, "lane", id="0", type="none", level="false")
    _road_mark(c_lane, left_mark)        # marking on the lane's left (centerline) side

    right = ET.SubElement(ls, "right")
    r_lane = ET.SubElement(right, "lane", id="-1", type="driving", level="false")
    ET.SubElement(r_lane, "width", sOffset="0.0", a=f"{width:.6f}",
                  b="0.0", c="0.0", d="0.0")
    _road_mark(r_lane, right_mark)       # marking on the lane's right (outer) side


def add_link(road_el, predecessor=None, successor=None):
    """predecessor/successor: tuple (elementType, elementId, contactPoint?)."""
    if not predecessor and not successor:
        return
    link = ET.SubElement(road_el, "link")
    for tag, spec in (("predecessor", predecessor), ("successor", successor)):
        if spec is None:
            continue
        attrs = {"elementType": spec[0], "elementId": str(spec[1])}
        if spec[0] == "road" and len(spec) > 2 and spec[2]:
            attrs["contactPoint"] = spec[2]
        ET.SubElement(link, tag, **attrs)
    # links must come before planView in the schema; reorder below at write time


# --------------------------------------------------------------------------- #
# Main conversion
# --------------------------------------------------------------------------- #
def _convert_one(nusc, nmap, scene, map_name, out_path, args):
    print(f"      scene = {scene['name']}  ({scene['description']})")
    print(f"      map   = {map_name}")

    print("[2/7] Computing ego trajectory + region patch")
    poses = get_scene_ego_trajectory(nusc, scene)
    if getattr(args, "region", None):
        patch = tuple(args.region)
        print(f"      using explicit --region patch")
    else:
        patch = trajectory_patch(poses, args.padding)
    origin = (patch[0], patch[1])
    print(f"      patch (x_min,y_min,x_max,y_max) = "
          f"({patch[0]:.1f}, {patch[1]:.1f}, {patch[2]:.1f}, {patch[3]:.1f})")
    print(f"      size = {patch[2]-patch[0]:.1f} x {patch[3]-patch[1]:.1f} m")

    print("[3/7] Selecting lanes / connectors in patch")
    recs = nmap.get_records_in_patch(patch, ["lane", "lane_connector"],
                                     mode="intersect")
    lane_tokens = set(recs["lane"])
    conn_tokens = set(recs["lane_connector"])
    print(f"      lanes = {len(lane_tokens)}  connectors = {len(conn_tokens)}")

    # geometry + width + lane markings for everything selected
    print("[4/7] Discretizing geometry + estimating widths + markings")
    geom = {}      # token -> list[(x,y,yaw)]
    widths = {}    # token -> float
    marks = {}     # token -> (left_mark, right_mark)
    for tok in lane_tokens | conn_tokens:
        pts = discretize(nmap, tok, args.resolution)
        if len(pts) < 2:
            continue
        geom[tok] = pts
        length = arcline_path_utils.length_of_lane(nmap.get_arcline_path(tok))
        widths[tok] = args.lane_width if args.lane_width else lane_width(
            nmap, tok, length)
        marks[tok] = lane_markings(nmap, tok)
    # drop tokens without usable geometry
    lane_tokens &= geom.keys()
    conn_tokens &= geom.keys()

    print("[5/7] Building topology (junctions via union-find on connectors)")
    conn_in = {}   # connector -> incoming lane token (if selected)
    conn_out = {}  # connector -> outgoing lane token (if selected)
    usable_conns = set()
    for c in conn_tokens:
        info = nmap.connectivity.get(c, {})
        inc = [t for t in info.get("incoming", []) if t in lane_tokens]
        out = [t for t in info.get("outgoing", []) if t in lane_tokens]
        if inc and out:                       # both ends inside the region
            conn_in[c] = inc[0]
            conn_out[c] = out[0]
            usable_conns.add(c)

    # union connectors that share an incoming or outgoing lane
    uf = UnionFind()
    by_in = {}
    by_out = {}
    for c in usable_conns:
        uf.find(c)
        by_in.setdefault(conn_in[c], []).append(c)
        by_out.setdefault(conn_out[c], []).append(c)
    for group in list(by_in.values()) + list(by_out.values()):
        for c in group[1:]:
            uf.union(group[0], c)

    # assign junction ids (only clusters that contain >=1 usable connector)
    junction_of_conn = {}
    junction_ids = {}
    # CARLA 0.9.15 distinguishes a road successor from a junction successor
    # by testing whether the numeric ID exists in the road table. Keep the two
    # ID domains disjoint even though OpenDRIVE itself does not require it.
    next_jid = 1000000
    for c in usable_conns:
        root = uf.find(c)
        if root not in junction_ids:
            junction_ids[root] = next_jid
            next_jid += 1
        junction_of_conn[c] = junction_ids[root]

    # lane -> junction at each end
    succ_junction = {}   # lane -> jid (lane feeds connectors -> junction at lane END)
    pred_junction = {}   # lane -> jid (connectors feed lane -> junction at lane START)
    for c in usable_conns:
        jid = junction_of_conn[c]
        succ_junction[conn_in[c]] = jid    # incoming lane's successor is this junction
        pred_junction[conn_out[c]] = jid   # outgoing lane's predecessor is this junction

    # assign integer road ids
    all_roads = list(lane_tokens) + list(conn_tokens)
    road_id = {tok: i + 1 for i, tok in enumerate(all_roads)}

    print(f"      junctions = {len(junction_ids)}  "
          f"usable connectors = {len(usable_conns)}")

    print("[6/7] Emitting OpenDRIVE XML")
    root = ET.Element("OpenDRIVE")
    header = ET.SubElement(root, "header", revMajor="1", revMinor="4",
                           name=f"{map_name}_{scene['name']}",
                           version="1.00",
                           north=f"{patch[3]-origin[1]:.6f}",
                           south="0.0",
                           east=f"{patch[2]-origin[0]:.6f}",
                           west="0.0")
    geo = ET.SubElement(header, "geoReference")
    geo.text = ("<![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 "
                "+datum=WGS84 +units=m +no_defs]]>")

    road_el = {}     # token -> <road> element
    road_lpts = {}   # token -> local-frame [(x,y), ...] for projection

    def make_road(tok, is_connector):
        rid = road_id[tok]
        pts = geom[tok]
        road_lpts[tok] = [(p[0] - origin[0], p[1] - origin[1]) for p in pts]
        length = 0.0
        for i in range(len(pts) - 1):
            length += math.hypot(pts[i + 1][0] - pts[i][0],
                                 pts[i + 1][1] - pts[i][1])
        jattr = str(junction_of_conn[tok]) if (is_connector and tok in
                                               junction_of_conn) else "-1"
        road = ET.SubElement(root, "road", name=("conn" if is_connector else "lane"),
                             length=f"{length:.6f}", id=str(rid), junction=jattr)

        # link element (must precede planView in schema -> build then move)
        pred = succ = None
        if is_connector and tok in usable_conns:
            pred = ("road", road_id[conn_in[tok]], "end")
            succ = ("road", road_id[conn_out[tok]], "start")
        elif not is_connector:
            if tok in pred_junction:
                pred = ("junction", pred_junction[tok])
            if tok in succ_junction:
                succ = ("junction", succ_junction[tok])
        add_link(road, pred, succ)

        # type element
        ET.SubElement(road, "type", s="0.0", type="town")

        real_len = build_planview(road, pts, origin)
        # keep declared length consistent with emitted geometry
        road.set("length", f"{real_len:.6f}")

        lm, rm = marks.get(tok, (None, None))
        add_lanes_section(road, widths[tok], left_mark=lm, right_mark=rm)

        # reorder children: link, type, planView, lanes  (OpenDRIVE schema order)
        order = {"link": 0, "type": 1, "planView": 2, "lanes": 3}
        children = sorted(list(road), key=lambda e: order.get(e.tag, 99))
        for ch in list(road):
            road.remove(ch)
        for ch in children:
            road.append(ch)
        road_el[tok] = road

    for tok in lane_tokens:
        make_road(tok, is_connector=False)
    for tok in conn_tokens:
        make_road(tok, is_connector=True)

    # crosswalks: attach each ped_crossing polygon to its nearest road as an
    # OpenDRIVE <object type="crosswalk"> with an outline in (s, t) coords, and
    # keep the local-frame polygon so the loader can also draw zebra stripes.
    n_cw = 0
    crosswalk_polys = []
    if args.crosswalks:
        cw_tokens = nmap.get_records_in_patch(
            patch, ["ped_crossing"], mode="intersect")["ped_crossing"]
        for idx, tok in enumerate(cw_tokens):
            rec = nmap.get("ped_crossing", tok)
            try:
                poly = nmap.extract_polygon(rec["polygon_token"])
            except Exception:
                continue
            corners = [(x - origin[0], y - origin[1])
                       for x, y in list(poly.exterior.coords)[:-1]]
            if len(corners) < 3:
                continue
            crosswalk_polys.append([[round(x, 3), round(y, 3)] for x, y in corners])
            ccx = sum(c[0] for c in corners) / len(corners)
            ccy = sum(c[1] for c in corners) / len(corners)
            # nearest road by centroid distance to its polyline points
            best_tok, best_d = None, float("inf")
            for rt, lpts in road_lpts.items():
                for px, py in lpts:
                    d = (px - ccx) ** 2 + (py - ccy) ** 2
                    if d < best_d:
                        best_d, best_tok = d, rt
            if best_tok is None:
                continue
            lpts = road_lpts[best_tok]
            rlen = polyline_length(lpts)
            s0, t0 = project_to_polyline(lpts, (ccx, ccy))
            objs = road_el[best_tok].find("objects")
            if objs is None:
                objs = ET.SubElement(road_el[best_tok], "objects")
            obj = ET.SubElement(
                objs, "object", id=str(9000 + idx), name="crosswalk",
                type="crosswalk", s=f"{clamp(s0, 0.0, rlen):.3f}",
                t=f"{t0:.3f}", zOffset="0.0", hdg="0.0", roll="0.0",
                pitch="0.0", orientation="none",
                width="0.0", length="0.0", height="0.0")
            outline = ET.SubElement(obj, "outline")
            for px, py in corners:
                sc, tc = project_to_polyline(lpts, (px, py))
                ET.SubElement(outline, "cornerRoad", s=f"{clamp(sc, 0.0, rlen):.3f}",
                              t=f"{tc:.3f}", dz="0.0", height="0.0")
            n_cw += 1
    print(f"      crosswalks = {n_cw}")

    inv_junction = {}
    for c, jid in junction_of_conn.items():
        inv_junction.setdefault(jid, []).append(c)

    # traffic lights: OpenDRIVE traffic-light signals grouped into per-phase
    # controllers (CARLA builds cycling light groups). With --osm-signals we
    # place exactly one signal per real OSM traffic_signals node (mapped to the
    # nearest approach lane); otherwise every junction's incoming roads are lit.
    # (controllers must precede <junction> elements in the document.)
    junction_ctrls = {}     # jid -> [(controller_id, sequence), ...]
    n_tl = 0
    if args.traffic_lights:
        osm_sig = fetch_osm_traffic_signals(map_name, patch) if args.osm_signals \
            else None
        # OSM-driven only when the fetch succeeded (an empty list == no signals
        # -> zero lights; None == fetch failed -> fall back to lighting all)
        use_osm = args.osm_signals and osm_sig is not None
        if args.osm_signals:
            print("      OSM traffic_signals = %s"
                  % ("fetch failed (lighting all)" if osm_sig is None
                     else len(osm_sig)))

        sig_id, ctrl_id = 20000, 30000
        jbuckets = {}           # jid -> {phase_bucket -> [signal_id]}

        def emit_signal(lane_tok):
            """Add a traffic-light signal at the end of an approach lane road."""
            nonlocal sig_id, n_tl
            lpts = road_lpts[lane_tok]
            (x0, y0), (x1, y1) = lpts[-2], lpts[-1]
            hdg = math.degrees(math.atan2(y1 - y0, x1 - x0))
            bucket = int(round(hdg / 90.0)) % 2         # approach axis -> phase
            rlen = polyline_length(lpts)
            w = widths.get(lane_tok, 3.5)
            road = road_el[lane_tok]
            sigs = road.find("signals")
            if sigs is None:
                sigs = ET.SubElement(road, "signals")
            sig = ET.SubElement(
                sigs, "signal", name=f"TL_{sig_id}", id=str(sig_id),
                s=f"{max(rlen - 0.6, 0.0):.4f}", t=f"{-(w / 2 + 1.0):.4f}",
                zOffset="5.0", hOffset="0.0", roll="0.0", pitch="0.0",
                orientation="+", dynamic="yes", country="OpenDRIVE",
                type="1000001", subtype="-1", value="-1",
                height="3.0", width="0.5")
            ET.SubElement(sig, "validity", **{"fromLane": "-1", "toLane": "-1"})
            n_tl += 1
            sig_id += 1
            return bucket, sig_id - 1

        if use_osm:
            # one signal per OSM node -> nearest approach lane (whose end is the
            # junction entry); group by that lane's junction for phasing
            approach = [t for t in lane_tokens
                        if t in succ_junction and len(road_lpts.get(t, [])) >= 2]
            for sx, sy in osm_sig:
                best, bd = None, float("inf")
                for t in approach:
                    lx, ly = road_lpts[t][-1]
                    d = (lx + origin[0] - sx) ** 2 + (ly + origin[1] - sy) ** 2
                    if d < bd:
                        bd, best = d, t
                if best is None:
                    continue
                b, sid = emit_signal(best)
                jbuckets.setdefault(succ_junction[best], {}).setdefault(
                    b, []).append(sid)
        else:
            # fallback: light every junction's incoming roads
            for jid, conns in sorted(inv_junction.items()):
                for lane_tok in sorted({conn_in[c] for c in conns}):
                    if len(road_lpts.get(lane_tok, [])) < 2:
                        continue
                    b, sid = emit_signal(lane_tok)
                    jbuckets.setdefault(jid, {}).setdefault(b, []).append(sid)

        for jid, buckets in jbuckets.items():
            ctrls, seq = [], 0
            for bkey in sorted(buckets):
                cel = ET.SubElement(root, "controller", name=f"ctrl{ctrl_id}",
                                    id=str(ctrl_id), sequence=str(seq))
                for sid in buckets[bkey]:
                    ET.SubElement(cel, "control", signalId=str(sid), type="")
                ctrls.append((ctrl_id, seq))
                ctrl_id += 1
                seq += 1
            junction_ctrls[jid] = ctrls
        print("      traffic lights = %d signals across %d junction(s)"
              % (n_tl, len(jbuckets)))

    # junctions
    for jid, conns in sorted(inv_junction.items()):
        jel = ET.SubElement(root, "junction", id=str(jid), name=f"junction{jid}")
        for k, c in enumerate(conns):
            connection = ET.SubElement(
                jel, "connection",
                id=str(k),
                incomingRoad=str(road_id[conn_in[c]]),
                connectingRoad=str(road_id[c]),
                contactPoint="start")
            ET.SubElement(connection, "laneLink", **{"from": "-1", "to": "-1"})
        for cid, seq in junction_ctrls.get(jid, []):
            ET.SubElement(jel, "controller", id=str(cid), type="0",
                          sequence=str(seq))

    # pretty print
    rough = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(rough).toprettyxml(indent="    ")
    # minidom escapes the CDATA; restore it
    pretty = pretty.replace(
        "&lt;![CDATA[", "<![CDATA[").replace("]]&gt;", "]]>")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(pretty)

    # sidecar (ego start/trajectory + origin offset for the CARLA loader)
    sidecar = {
        "scene": scene["name"],
        "map": map_name,
        "origin": {"x": origin[0], "y": origin[1]},
        "patch": {"x_min": patch[0], "y_min": patch[1],
                  "x_max": patch[2], "y_max": patch[3]},
        "ego_start_local": {"x": poses[0][0] - origin[0],
                            "y": poses[0][1] - origin[1],
                            "yaw": poses[0][2]},
        "ego_trajectory_local": [
            {"x": p[0] - origin[0], "y": p[1] - origin[1], "yaw": p[2]}
            for p in poses
        ],
        "frame_dt": 0.5,  # nuScenes keyframes are ~2 Hz
        "agent_frames": get_scene_agent_frames(
            nusc, scene, origin, include_static=args.include_static),
        "buildings": (compute_building_footprints(nmap, patch, origin)
                      if args.buildings else []),
        "crosswalks": crosswalk_polys,
        "counts": {"lanes": len(lane_tokens),
                   "connectors": len(conn_tokens),
                   "junctions": len(junction_ids)},
    }
    side_path = os.path.splitext(out_path)[0] + "_meta.json"
    with open(side_path, "w", encoding="utf-8") as f:
        json.dump(sidecar, f, indent=2)

    print("[7/7] Done")
    print(f"      xodr     -> {os.path.abspath(out_path)}")
    print(f"      sidecar  -> {os.path.abspath(side_path)}")
    print(f"      roads={len(all_roads)}  junctions={len(junction_ids)}")
    return {"scene": scene["name"], "map": map_name, "out": out_path,
            "roads": len(all_roads), "junctions": len(junction_ids)}


OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def convert(args):
    print(f"[1/7] Loading nuScenes {args.version} from {args.dataroot}")
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    if args.all_scenes:
        scenes = list(nusc.scene)
    elif args.scene_name:
        scenes = [next(s for s in nusc.scene if s["name"] == args.scene_name)]
    else:
        scenes = [nusc.scene[args.scene_index]]

    nmap_cache = {}
    results = []
    for i, scene in enumerate(scenes):
        map_name = nusc.get("log", scene["log_token"])["location"]
        if map_name not in nmap_cache:
            nmap_cache[map_name] = NuScenesMap(dataroot=args.map_dataroot,
                                               map_name=map_name)
        if args.out and not args.all_scenes:
            out_path = args.out
        else:
            out_path = os.path.join(OUTPUT_DIR, f"{map_name}_{scene['name']}.xodr")
        print(f"\n=== [{i + 1}/{len(scenes)}] {scene['name']} ({map_name}) ===")
        try:
            results.append(_convert_one(nusc, nmap_cache[map_name], scene,
                                        map_name, out_path, args))
        except Exception as e:
            print(f"      !! failed: {e}")
            results.append({"scene": scene["name"], "map": map_name,
                            "error": str(e)})

    if len(results) > 1:
        print("\n=== Summary ===")
        for r in results:
            if "error" in r:
                print(f"  {r['scene']:<12} {r['map']:<22} FAILED: {r['error']}")
            else:
                print(f"  {r['scene']:<12} {r['map']:<22} "
                      f"roads={r['roads']:<4} junctions={r['junctions']}")
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataroot",
                    default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "v1.0-mini"),
                    help="split root holding both <version>/*.json tables and "
                         "maps/expansion/<map>.json")
    ap.add_argument("--map-dataroot", default=None,
                    help="folder holding maps/expansion/<map>.json "
                         "(defaults to --dataroot)")
    ap.add_argument("--version", default="v1.0-mini")
    ap.add_argument("--scene-index", type=int, default=0,
                    help="index into nusc.scene (0 = first mini scene = scene-0061)")
    ap.add_argument("--scene-name", default=None,
                    help="override scene by name, e.g. scene-0061")
    ap.add_argument("--all-scenes", action="store_true",
                    help="convert every scene in the version (auto-named outputs)")
    ap.add_argument("--padding", type=float, default=50.0,
                    help="metres padded around the ego-trajectory bbox")
    ap.add_argument("--resolution", type=float, default=0.5,
                    help="centerline discretization step (m); smaller = smoother")
    ap.add_argument("--crosswalks", action="store_true",
                    help="emit ped_crossing crosswalks (off by default)")
    ap.add_argument("--buildings", action="store_true",
                    help="synthesize building footprints in the sidecar "
                         "(off by default)")
    ap.add_argument("--traffic-lights", action="store_true",
                    help="emit OpenDRIVE traffic-light signals at junctions "
                         "(CARLA builds cycling light groups)")
    ap.add_argument("--osm-signals", action="store_true",
                    help="only light junctions near a real OSM traffic_signals "
                         "node (needs internet); else every junction is lit")
    ap.add_argument("--osm-radius", type=float, default=40.0,
                    help="metres: junction-centroid to OSM-signal match distance")
    ap.add_argument("--lane-width", type=float, default=None,
                    help="fixed lane width (m); default estimates per lane")
    ap.add_argument("--include-static", action="store_true",
                    help="also export static objects (cones, barriers, debris) "
                         "as replay agents")
    ap.add_argument("--region", type=float, nargs=4, default=None,
                    metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    help="override the conversion patch with explicit map-frame "
                         "bounds (m); the chosen scene still provides the ego/"
                         "agent replay data in the sidecar")
    ap.add_argument("--out", default=None,
                    help="output .xodr path; default auto-names per scene under "
                         "output/<map>_<scene>.xodr")
    args = ap.parse_args()
    if args.map_dataroot is None:
        args.map_dataroot = args.dataroot
    convert(args)


if __name__ == "__main__":
    main()
