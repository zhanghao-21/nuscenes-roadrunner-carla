#!/usr/bin/env python
"""Replay nuScenes scenes on the RoadRunner-generated geo maps.

Self-contained (stdlib + numpy + matplotlib + Pillow only, no imports from
nuscenes2xodr). For each scene it

  1. parses ``output_roadrunner/<base>/<base>_geo.xodr`` — the geo-referenced
     OpenDRIVE exported by the RoadRunner pipeline — into lane ribbons and
     boundary markings (the geometry is in the same patch-local frame as the
     converter sidecars, only the geoReference header differs),
  2. loads ``output_roadrunner/<base>_meta.json`` for the ego trajectory and
     the per-keyframe agent poses (2 Hz),
  3. renders a top-down replay: agents as oriented boxes with past (dim) /
     future (bright) trajectory trails, ego in gold, vehicles blue,
     pedestrians green, cyclists/motorcycles magenta,
  4. writes results to ``replay_geo/results/<base>/``:
        <base>_map.png      static map + full trajectories overview
        <base>_replay.gif   the animated replay
        frames/*.png        one PNG per keyframe (with --save-frames)

Usage:
    python replay_geo.py --list
    python replay_geo.py --scene 0103
    python replay_geo.py --all
    python replay_geo.py --scene 0061 --follow --window 70 --save-frames
"""

import argparse
import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.lines import Line2D
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
RR_DIR = ROOT / "output_roadrunner"
# results root; override with REPLAY_GEO_RESULTS (used by main.py -> OUTPUT/)
RESULTS = Path(os.environ.get("REPLAY_GEO_RESULTS",
                              Path(__file__).resolve().parent / "results"))

# published nuScenes map origins (lat/lon of map coordinate (0,0)),
# same values as nuscenes_ground_buildings.m / nuscenes2xodr
MAP_ORIGIN = {
    "boston-seaport":           (42.336849169438615, -71.05785369873047),
    "singapore-onenorth":       (1.2882100868743724, 103.78475189208984),
    "singapore-hollandvillage": (1.2993652317780957, 103.78217697143555),
    "singapore-queenstown":     (1.2782562240223188, 103.76741409301758),
}
EARTH_R = 6378137.0

# ---------------------------------------------------------------- appearance
BG = "#1e1f24"
ROAD_FILL = "#3c3f46"
JUNCTION_FILL = "#34363c"
BUILDING_FILL = "#292c33"
BUILDING_EDGE = "#565b66"
RR_BLDG_EDGE = "#4dccdd"     # cyan, matching the MATLAB preview colour
MARK_COLORS = {"white": "#d8d8d8", "yellow": "#e6c84a", "standard": "#d8d8d8"}
EGO_C = {"bright": "#ffd700", "dim": "#8a7a1e"}
CAT_STYLE = {
    "vehicle": {"bright": "#4da3ff", "dim": "#2a5c94"},
    "pedestrian": {"bright": "#57d977", "dim": "#2e7342"},
    "bike": {"bright": "#e86fe8", "dim": "#7c3a7c"},
    "other": {"bright": "#b0b0b0", "dim": "#6a6a6a"},
}
EGO_LW = 4.084  # nuScenes ego (Renault Zoe) length / width
EGO_W = 1.730


def classify(category: str) -> str:
    if category.startswith("human.pedestrian"):
        return "pedestrian"
    if category in ("vehicle.bicycle", "vehicle.motorcycle"):
        return "bike"
    if category.startswith("vehicle."):
        return "vehicle"
    return "other"


# ---------------------------------------------------------------- xodr parse
def parse_geo_xodr(path: Path):
    """Parse the geo OpenDRIVE into drawable primitives.

    Returns a list of dicts per road: centerline polyline (N,2), constant
    lane width, junction flag, and the boundary roadMark styles.
    """
    root = ET.parse(path).getroot()
    roads = []
    for road in root.findall("road"):
        pts = []
        for g in road.findall("./planView/geometry"):
            x = float(g.get("x"))
            y = float(g.get("y"))
            hdg = float(g.get("hdg"))
            length = float(g.get("length"))
            pts.append((x, y))
            end = (x + length * math.cos(hdg), y + length * math.sin(hdg))
        if not pts:
            continue
        pts.append(end)
        center = np.asarray(pts)

        wnode = road.find('.//lane[@id="-1"]/width')
        width = float(wnode.get("a")) if wnode is not None else 3.5
        offn = road.find(".//laneOffset")
        offset = float(offn.get("a")) if offn is not None else width / 2.0

        def mark_style(lane_xpath):
            m = road.find(lane_xpath + "/roadMark")
            if m is None:
                return None
            return {"type": m.get("type", "solid"), "color": m.get("color", "white")}

        roads.append(
            {
                "id": road.get("id"),
                "center": center,
                "width": width,
                "offset": offset,
                "junction": road.get("junction", "-1") != "-1",
                "mark_left": mark_style('.//center/lane[@id="0"]'),
                "mark_right": mark_style('.//right/lane[@id="-1"]'),
            }
        )
    return roads


def load_osm_buildings(base: str, meta: dict, margin: float = 60.0):
    """Building footprints from the scene's OSM extract, in patch-local metres.

    Same selection + projection as nuscenes_ground_buildings.m: ways with a
    ``building`` tag, equirectangular projection anchored at the published
    nuScenes map origin, shifted to the patch corner (manualShift = 0).
    """
    osm_path = RR_DIR / f"{base}.osm"
    origin = MAP_ORIGIN.get(meta.get("map", ""))
    if not osm_path.is_file() or origin is None:
        return []
    lat0, lon0 = origin
    patch = meta["patch"]
    lat_c = lat0 + (patch["y_min"] / EARTH_R) * (180.0 / math.pi)
    lon_c = lon0 + (patch["x_min"] / (EARTH_R * math.cos(math.radians(lat0)))) * (180.0 / math.pi)

    root = ET.parse(osm_path).getroot()
    nodes = {
        n.get("id"): (float(n.get("lat")), float(n.get("lon")))
        for n in root.findall("node")
    }
    x_max = patch["x_max"] - patch["x_min"] + margin
    y_max = patch["y_max"] - patch["y_min"] + margin
    footprints = []
    for way in root.findall("way"):
        if not any(t.get("k") == "building" for t in way.findall("tag")):
            continue
        pts = [nodes[nd.get("ref")] for nd in way.findall("nd") if nd.get("ref") in nodes]
        if len(pts) < 3:
            continue
        lat = np.array([p[0] for p in pts])
        lon = np.array([p[1] for p in pts])
        x = np.radians(lon - lon_c) * EARTH_R * math.cos(math.radians(lat0))
        y = np.radians(lat - lat_c) * EARTH_R
        if x.max() < -margin or x.min() > x_max or y.max() < -margin or y.min() > y_max:
            continue
        footprints.append(np.column_stack([x, y]))
    return footprints


def draw_buildings(ax, footprints):
    for fp in footprints:
        ax.add_patch(
            MplPolygon(fp, closed=True, facecolor=BUILDING_FILL,
                       edgecolor=BUILDING_EDGE, lw=0.5, zorder=0.5)
        )


# ------------------------------------------------------- RoadRunner buildings
def _pb_varint(b, i):
    v = s = 0
    while True:
        x = b[i]
        i += 1
        v |= (x & 0x7F) << s
        if not x & 0x80:
            return v, i
        s += 7


def _pb_triple(msg):
    """Doubles at protobuf fields 1..3 of a Vector3-style message."""
    out, i = {}, 0
    while i < len(msg):
        key, i = _pb_varint(msg, i)
        f, wt = key >> 3, key & 7
        if wt == 1:
            out[f] = struct.unpack("<d", msg[i:i + 8])[0]
            i += 8
        elif wt == 2:
            ln, i = _pb_varint(msg, i)
            i += ln
        else:
            _, i = _pb_varint(msg, i)
    return out


def load_rrhd_buildings(base: str):
    """Building boxes actually placed in the RoadRunner scene.

    Scans the ``<base>_city.rrhd`` HD-map protobuf for the ``BldgN_M``
    StaticObjects written by nuscenes_ground_buildings.m and decodes their
    GeoOrientedBoundingBox: rows (cx, cy, hx, hy, yaw) in patch-local metres
    (hx/hy are half-extents, yaw in radians — the file's projection is
    anchored at the patch corner, i.e. the same frame as the geo.xodr).
    """
    path = RR_DIR / base / f"{base}_city.rrhd"
    if not path.is_file():
        return []
    b = path.read_bytes()
    boxes = []
    for m in re.finditer(rb"\x0a(.)Bldg", b, re.DOTALL):
        ln = m.group(1)[0]
        name = b[m.start() + 2 : m.start() + 2 + ln]
        if not re.fullmatch(rb"Bldg[0-9]+_[0-9]+", name):
            continue
        i = m.start() + 2 + ln
        if i >= len(b) or b[i] != 0x12:       # geometry submessage expected
            continue
        glen, j = _pb_varint(b, i + 1)
        g = b[j : j + glen]
        fields, k = {}, 0
        while k < len(g):
            key, k = _pb_varint(g, k)
            f, wt = key >> 3, key & 7
            if wt == 2:
                l2, k = _pb_varint(g, k)
                fields[f] = g[k : k + l2]
                k += l2
            elif wt == 1:
                k += 8
            else:
                _, k = _pb_varint(g, k)
        center = _pb_triple(fields.get(1, b""))
        dim = _pb_triple(fields.get(2, b""))
        yaw = 0.0
        if 3 in fields:                       # orientation -> inner vector, z = yaw
            inner = fields[3]
            key, k = _pb_varint(inner, 0)
            if key & 7 == 2:
                l2, k = _pb_varint(inner, k)
                yaw = _pb_triple(inner[k : k + l2]).get(3, 0.0)
        boxes.append((center.get(1, 0.0), center.get(2, 0.0),
                      dim.get(1, 0.0), dim.get(2, 0.0), yaw))
    return boxes


def draw_rr_buildings(ax, boxes):
    for cx, cy, hx, hy, yaw in boxes:
        ax.add_patch(
            MplPolygon(box_corners(cx, cy, yaw, 2 * hx, 2 * hy), closed=True,
                       facecolor="none", edgecolor=RR_BLDG_EDGE, lw=0.9,
                       zorder=2.5)
        )


def offset_polyline(center: np.ndarray, t: float) -> np.ndarray:
    """Offset a polyline laterally: +t is left of the travel direction."""
    d = np.gradient(center, axis=0)
    n = np.stack([-d[:, 1], d[:, 0]], axis=1)
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    return center + t * n / norm


def draw_map(ax, roads):
    """Draw lane ribbons + boundary markings; return the drawn extent."""
    all_pts = []
    for r in roads:
        c, off, w = r["center"], r["offset"], r["width"]
        left = offset_polyline(c, off)
        right = offset_polyline(c, off - w)
        poly = np.vstack([left, right[::-1]])
        ax.add_patch(
            MplPolygon(
                poly,
                closed=True,
                facecolor=JUNCTION_FILL if r["junction"] else ROAD_FILL,
                edgecolor="none",
                zorder=1,
            )
        )
        all_pts.append(poly)
        if r["junction"]:  # junction interiors are unmarked, like the source map
            continue
        for boundary, mark in (("L", r["mark_left"]), ("R", r["mark_right"])):
            if mark is None:
                continue
            line = left if boundary == "L" else right
            dashed = "broken" in mark["type"]
            ax.plot(
                line[:, 0],
                line[:, 1],
                color=MARK_COLORS.get(mark["color"], "#d8d8d8"),
                lw=1.6 if "solid solid" in mark["type"] else 0.8,
                ls=(0, (4, 4)) if dashed else "-",
                zorder=2,
                solid_capstyle="butt",
            )
    pts = np.vstack(all_pts)
    return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()


# ---------------------------------------------------------------- meta / tracks
def load_tracks(meta: dict):
    """Turn agent_frames into per-agent keyframe tracks.

    Returns {id: {kind, wlh, k (keyframe indices), x, y, yaw(unwrapped)}}.
    """
    tracks = {}
    for k, frame in enumerate(meta["agent_frames"]):
        for a in frame:
            t = tracks.setdefault(
                a["id"],
                {"kind": classify(a["category"]), "wlh": a["wlh"], "k": [], "x": [], "y": [], "yaw": []},
            )
            t["k"].append(k)
            t["x"].append(a["x"])
            t["y"].append(a["y"])
            t["yaw"].append(a["yaw"])
    for t in tracks.values():
        t["k"] = np.asarray(t["k"], float)
        t["x"] = np.asarray(t["x"])
        t["y"] = np.asarray(t["y"])
        t["yaw"] = np.unwrap(np.asarray(t["yaw"]))
    return tracks


def box_corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + (x, y)


def sample(track, k):
    """Pose of a track at fractional keyframe k, or None outside its life."""
    ks = track["k"]
    if k < ks[0] - 0.25 or k > ks[-1] + 0.25:
        return None
    x = np.interp(k, ks, track["x"])
    y = np.interp(k, ks, track["y"])
    yaw = np.interp(k, ks, track["yaw"])
    return x, y, yaw


# ---------------------------------------------------------------- rendering
class Replay:
    def __init__(self, base, args):
        self.base = base
        self.args = args
        self.xodr = RR_DIR / base / f"{base}_geo.xodr"
        meta_path = RR_DIR / f"{base}_meta.json"
        if not self.xodr.is_file():
            raise FileNotFoundError(self.xodr)
        if not meta_path.is_file():
            raise FileNotFoundError(meta_path)
        self.meta = json.loads(meta_path.read_text())
        self.roads = parse_geo_xodr(self.xodr)
        self.buildings = [] if args.no_buildings else load_osm_buildings(base, self.meta)
        self.rr_buildings = [] if args.no_rr_buildings else load_rrhd_buildings(base)
        self.tracks = load_tracks(self.meta)
        self.dt = float(self.meta.get("frame_dt", 0.5))

        ego = self.meta["ego_trajectory_local"]
        self.ego = {
            "kind": "ego",
            "wlh": [EGO_W, EGO_LW, 1.5],
            "k": np.arange(len(ego), dtype=float),
            "x": np.array([p["x"] for p in ego]),
            "y": np.array([p["y"] for p in ego]),
            "yaw": np.unwrap(np.array([p["yaw"] for p in ego])),
        }
        self.n_key = len(ego)

    # -- static overview -----------------------------------------------------
    def save_overview(self, out_dir: Path):
        fig, ax = self._new_fig()
        draw_buildings(ax, self.buildings)
        draw_rr_buildings(ax, self.rr_buildings)
        ext = draw_map(ax, self.roads)
        for t in self.tracks.values():
            c = CAT_STYLE[t["kind"]]["bright"]
            ax.plot(t["x"], t["y"], color=c, lw=0.9, alpha=0.55, zorder=3)
        ax.plot(self.ego["x"], self.ego["y"], color=EGO_C["bright"], lw=2.2, zorder=4)
        ax.plot(self.ego["x"][0], self.ego["y"][0], "o", color=EGO_C["bright"], ms=6, zorder=4)
        self._frame_axes(ax, ext)
        layers = "geo map + OSM buildings"
        if self.rr_buildings:
            layers += f" + {len(self.rr_buildings)} RoadRunner boxes"
        ax.set_title(
            f"{self.base}  —  {layers} + recorded trajectories "
            f"({len(self.tracks)} agents, {self.n_key} keyframes)",
            color="#e0e0e0", fontsize=10,
        )
        path = out_dir / f"{self.base}_map.png"
        fig.savefig(path, dpi=self.args.dpi, facecolor=BG, bbox_inches="tight")
        plt.close(fig)
        return path

    # -- animation -----------------------------------------------------------
    def save_replay(self, out_dir: Path):
        args = self.args
        fig, ax = self._new_fig()
        draw_buildings(ax, self.buildings)
        draw_rr_buildings(ax, self.rr_buildings)
        ext = draw_map(ax, self.roads)
        self._frame_axes(ax, ext)
        title = ax.set_title("", color="#e0e0e0", fontsize=10)
        self._legend(ax, with_rr=bool(self.rr_buildings))

        hist_kf = args.hist_seconds / self.dt
        fut_kf = args.future_seconds / self.dt

        actors = []  # (track, box_patch, past_line, future_line)
        for t in [self.ego] + list(self.tracks.values()):
            style = EGO_C if t["kind"] == "ego" else CAT_STYLE[t["kind"]]
            z = 8 if t["kind"] == "ego" else 6
            box = MplPolygon(
                np.zeros((4, 2)), closed=True, facecolor=style["bright"],
                edgecolor="black", lw=0.4, visible=False, zorder=z,
            )
            ax.add_patch(box)
            past = Line2D([], [], color=style["dim"], lw=1.3, zorder=z - 2)
            future = Line2D([], [], color=style["bright"], lw=1.3, zorder=z - 2)
            ax.add_line(past)
            ax.add_line(future)
            actors.append((t, box, past, future))

        frames_dir = out_dir / ("frames_follow" if args.follow else "frames")
        if args.save_frames:
            frames_dir.mkdir(exist_ok=True)

        n_frames = (self.n_key - 1) * args.substeps + 1
        images = []
        for f in range(n_frames):
            k = f / args.substeps
            title.set_text(f"{self.base}   t = {k * self.dt:5.1f} s   (geo.xodr replay)")
            for t, box, past, future in actors:
                pose = sample(t, k)
                if pose is None:
                    box.set_visible(False)
                    past.set_data([], [])
                    future.set_data([], [])
                    continue
                x, y, yaw = pose
                w, l = t["wlh"][0], t["wlh"][1]
                box.set_xy(box_corners(x, y, yaw, l, w))
                box.set_visible(True)
                ks = t["k"]
                pm = (ks >= k - hist_kf) & (ks <= k)
                fm = (ks >= k) & (ks <= k + fut_kf)
                past.set_data(np.append(t["x"][pm], x), np.append(t["y"][pm], y))
                future.set_data(np.insert(t["x"][fm], 0, x), np.insert(t["y"][fm], 0, y))
            if args.follow:
                ex, ey, _ = sample(self.ego, k)
                half = args.window / 2.0
                ax.set_xlim(ex - half, ex + half)
                ax.set_ylim(ey - half, ey + half)

            fig.canvas.draw()
            img = Image.frombuffer(
                "RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()
            ).convert("RGB")
            images.append(img.quantize(colors=256, dither=Image.Dither.NONE))
            if args.save_frames and f % args.substeps == 0:
                img.save(frames_dir / f"{f // args.substeps:04d}.png")

        plt.close(fig)
        suffix = "_follow" if args.follow else ""
        gif = out_dir / f"{self.base}_replay{suffix}.gif"
        images[0].save(
            gif, save_all=True, append_images=images[1:],
            duration=int(1000 / args.fps), loop=0, optimize=False,
        )
        return gif, n_frames

    # -- clean ego-centred stills for the aligned panel ----------------------
    def save_panel_frames(self, out_dir: Path):
        """One square, ego-centred BEV per 2 Hz keyframe, plot_topdown-style:
        no ticks/title/legend, trajectory dots, heading ticks on agents, an
        arrow on the ego — but with this project's dark palette + building
        layers. Saved to frames_panel/NNN.png."""
        args = self.args
        half = args.window / 2.0
        fdir = out_dir / "frames_panel"
        fdir.mkdir(exist_ok=True)

        fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=args.dpi)
        fig.patch.set_facecolor(BG)
        ax.set_facecolor(BG)
        draw_buildings(ax, self.buildings)
        draw_rr_buildings(ax, self.rr_buildings)
        draw_map(ax, self.roads)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color("#777777")

        hist_kf = args.hist_seconds / self.dt
        fut_kf = args.future_seconds / self.dt
        dyn = []                        # per-frame artists, removed each frame

        def trail(t, i, style):
            ks = t["k"]
            for future in (False, True):
                m = ((ks >= i - hist_kf) & (ks <= i)) if not future else \
                    ((ks >= i) & (ks <= i + fut_kf))
                if not m.any():
                    continue
                col = style["bright" if future else "dim"]
                lw = 2.2 if future else 1.6
                z = 4 + (1 if future else 0)
                dyn.append(ax.plot(t["x"][m], t["y"][m], color=col, lw=lw,
                                   zorder=z, solid_capstyle="round")[0])
                dyn.append(ax.scatter(t["x"][m], t["y"][m],
                                      s=9 if future else 6, color=col,
                                      zorder=z + 0.1, edgecolors="none"))

        paths = []
        for i in range(self.n_key):
            for a in dyn:
                a.remove()
            dyn = []

            trail(self.ego, i, EGO_C)
            for t in self.tracks.values():
                if i in t["k"]:
                    trail(t, i, CAT_STYLE[t["kind"]])

            for a in self.meta["agent_frames"][i]:
                w, l = a["wlh"][0], a["wlh"][1]
                col = CAT_STYLE[classify(a["category"])]["bright"]
                dyn.append(ax.add_patch(MplPolygon(
                    box_corners(a["x"], a["y"], a["yaw"], l, w), closed=True,
                    facecolor=col, edgecolor="black", lw=0.4, alpha=0.95,
                    zorder=8)))
                hx = a["x"] + math.cos(a["yaw"]) * l * 0.5
                hy = a["y"] + math.sin(a["yaw"]) * l * 0.5
                dyn.append(ax.plot([a["x"], hx], [a["y"], hy], color="black",
                                   lw=0.5, zorder=8.1)[0])

            ex = self.ego["x"][i]
            ey = self.ego["y"][i]
            eyaw = self.ego["yaw"][i]
            dyn.append(ax.add_patch(MplPolygon(
                box_corners(ex, ey, eyaw, EGO_LW, EGO_W), closed=True,
                facecolor=EGO_C["bright"], edgecolor="black", lw=0.9,
                zorder=9)))
            dyn.append(ax.annotate(
                "", xy=(ex + math.cos(eyaw) * EGO_LW,
                        ey + math.sin(eyaw) * EGO_LW),
                xytext=(ex, ey), zorder=9.1,
                arrowprops=dict(arrowstyle="->", color="black", lw=1.4)))

            ax.set_xlim(ex - half, ex + half)
            ax.set_ylim(ey - half, ey + half)
            fp = fdir / f"{i:03d}.png"
            fig.savefig(fp, dpi=args.dpi, facecolor=BG,
                        bbox_inches="tight", pad_inches=0.05)
            paths.append(fp)
        plt.close(fig)
        return paths

    # -- helpers -------------------------------------------------------------
    def _new_fig(self):
        fig, ax = plt.subplots(figsize=(9, 7.2), dpi=self.args.dpi)
        fig.patch.set_facecolor(BG)
        ax.set_facecolor(BG)
        return fig, ax

    @staticmethod
    def _frame_axes(ax, ext):
        x0, x1, y0, y1 = ext
        pad = 8
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y0 - pad, y1 + pad)
        ax.set_aspect("equal")
        ax.tick_params(colors="#888888", labelsize=7)
        for s in ax.spines.values():
            s.set_color("#555555")

    @staticmethod
    def _legend(ax, with_rr=False):
        entries = [("ego", EGO_C), ("vehicle", CAT_STYLE["vehicle"]),
                   ("pedestrian", CAT_STYLE["pedestrian"]), ("cyclist", CAT_STYLE["bike"])]
        handles = [Line2D([], [], color=s["bright"], lw=3, label=n) for n, s in entries]
        if with_rr:
            handles.append(Line2D([], [], color=RR_BLDG_EDGE, lw=1.5,
                                  label="RR building"))
        leg = ax.legend(handles=handles, loc="upper right", fontsize=7,
                        facecolor="#2a2b30", edgecolor="#555555", labelcolor="#e0e0e0")
        leg.set_zorder(20)


# ---------------------------------------------------------------- CLI
def discover_scenes():
    bases = []
    for d in sorted(RR_DIR.iterdir()):
        if d.is_dir() and (d / f"{d.name}_geo.xodr").is_file():
            bases.append(d.name)
    return bases


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--scene", help="scene id substring, e.g. 0103 or scene-0061")
    p.add_argument("--all", action="store_true", help="replay every scene with a geo.xodr")
    p.add_argument("--list", action="store_true", help="list available scenes and exit")
    p.add_argument("--substeps", type=int, default=4, help="interp frames per 2 Hz keyframe")
    p.add_argument("--fps", type=int, default=8, help="GIF playback fps")
    p.add_argument("--hist-seconds", type=float, default=2.0, help="past trail length (s)")
    p.add_argument("--future-seconds", type=float, default=6.0, help="future trail length (s)")
    p.add_argument("--follow", action="store_true", help="camera follows the ego")
    p.add_argument("--window", type=float, default=80.0, help="follow-view size (m)")
    p.add_argument("--save-frames", action="store_true", help="also save per-keyframe PNGs")
    p.add_argument("--no-buildings", action="store_true",
                   help="skip the OSM building footprints layer")
    p.add_argument("--no-rr-buildings", action="store_true",
                   help="skip the RoadRunner building boxes layer (from the .rrhd)")
    p.add_argument("--panel-frames", action="store_true",
                   help="only render clean ego-centred per-keyframe stills "
                        "(frames_panel/, for build_aligned_geo.py)")
    p.add_argument("--dpi", type=int, default=100)
    args = p.parse_args()

    scenes = discover_scenes()
    if args.list or not (args.scene or args.all):
        print("Scenes with a generated geo map:")
        for i, b in enumerate(scenes):
            print(f"  [{i}] {b}")
        if not (args.scene or args.all):
            print("\nPick one with --scene <id> or run --all.")
        return

    if args.all:
        todo = scenes
    else:
        todo = [b for b in scenes if args.scene in b]
        if not todo:
            sys.exit(f"no scene matching '{args.scene}' — try --list")

    for base in todo:
        out_dir = RESULTS / base
        out_dir.mkdir(parents=True, exist_ok=True)
        rep = Replay(base, args)
        print(f"[{base}] {len(rep.roads)} roads, {len(rep.tracks)} agents, "
              f"{rep.n_key} keyframes ({(rep.n_key - 1) * rep.dt:.1f} s)")
        if args.panel_frames:
            pf = rep.save_panel_frames(out_dir)
            print(f"  panel    -> {pf[0].parent.relative_to(ROOT)} ({len(pf)} stills)")
            continue
        ov = rep.save_overview(out_dir)
        print(f"  overview -> {ov.relative_to(ROOT)}")
        gif, n = rep.save_replay(out_dir)
        print(f"  replay   -> {gif.relative_to(ROOT)}  ({n} frames)")


if __name__ == "__main__":
    main()
