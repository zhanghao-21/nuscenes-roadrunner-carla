#!/usr/bin/env python
"""Replay nuScenes scenes in CARLA on the RoadRunner-generated geo maps.

Self-contained CARLA counterpart of replay_geo.py (stdlib + carla only, no
imports from nuscenes2xodr): loads ``output_roadrunner/<base>/<base>_geo.xodr``
into a running CARLA 0.9.15 server with ``generate_opendrive_world``, spawns
the ego at the recorded nuScenes start pose, and replays the recorded agents
(cars, trucks, buses, bikes, pedestrians) in synchronous mode — Catmull-Rom
smoothed ego path, interpolated agent keyframes, per-actor bounding-box
grounding, damped chase camera, and a past/future trajectory overlay.

Run with the Python 3.7 env that has the carla wheel:

    # terminal A: start the simulator
    D:/research/nuscenes_carla/carla_0.9.15/CarlaUE4.exe

    # terminal B:
    conda activate carla915
    python replay_geo_carla.py --list
    python replay_geo_carla.py --scene 0103
    python replay_geo_carla.py --scene 0061 --record       # save chase-cam PNGs
    python replay_geo_carla.py --all --record

With --record, one chase-camera PNG per 2 Hz keyframe is saved to
``replay_geo/results/<base>/carla_frames/`` and, if Pillow is available in
this env, the frames are also assembled into
``results/<base>/<base>_carla.gif`` (otherwise run make_gifs.py in the base
Python 3.12 env afterwards).

Coordinate note: CARLA is left-handed; when it ingests OpenDRIVE it flips Y,
so an OpenDRIVE pose (x, y, yaw) becomes CARLA (x, -y, -yaw).
"""

import argparse
import glob
import json
import math
import os
import re
import struct
import sys
import time

try:
    import queue
except ImportError:            # py2 fallback, not expected
    import Queue as queue

import carla

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RR_DIR = os.path.join(ROOT, "output_roadrunner")
# results root; override with REPLAY_GEO_RESULTS (used by main.py -> OUTPUT/)
RESULTS = os.environ.get(
    "REPLAY_GEO_RESULTS",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))


# --------------------------------------------------------------- coordinates
def carla_from_opendrive(x, y, yaw_rad):
    """OpenDRIVE (right-handed) -> CARLA (left-handed): flip Y and yaw."""
    return x, -y, -math.degrees(yaw_rad)


def _lerp(a, b, r):
    return a + (b - a) * r


def _lerp_angle(a, b, r):
    """Interpolate yaw (radians) along the shortest arc."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + d * r


def _catmull(p0, p1, p2, p3, t):
    """Catmull-Rom: C1-smooth position through p1..p2."""
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ((2 * p1) + (-p0 + p2) * t
                  + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)


# --------------------------------------------------------------- categories
_BP_CANDIDATES = {
    "pedestrian":  ["walker.pedestrian.*"],
    "bicycle":     ["vehicle.bh.crossbike", "vehicle.diamondback.century",
                    "vehicle.gazelle.omafiets"],
    "motorcycle":  ["vehicle.harley-davidson.low_rider", "vehicle.yamaha.yzf",
                    "vehicle.kawasaki.ninja", "vehicle.vespa.zx125"],
    "bus":         ["vehicle.mitsubishi.fusorosa"],
    "truck":       ["vehicle.carlamotors.carlacola", "vehicle.carlamotors.firetruck"],
    "car":         ["vehicle.audi.a2", "vehicle.audi.tt", "vehicle.tesla.model3",
                    "vehicle.nissan.patrol", "vehicle.toyota.prius",
                    "vehicle.seat.leon", "vehicle.citroen.c3"],
}

# distinct colours for surrounding vehicles (never red -> red is the ego)
_AGENT_COLORS = ["0,90,255", "0,160,255", "0,200,120", "220,180,0",
                 "150,0,255", "255,140,0", "210,210,210", "40,40,40"]

# trajectory overlay colours per group; history is drawn dimmer than future
_TRAJ_BASE = {
    "ego":        (255, 40, 40),    # red
    "vehicle":    (0, 160, 255),    # blue
    "pedestrian": (0, 220, 60),     # green
    "cyclist":    (255, 60, 200),   # magenta
}


def _category_kind(cat):
    if cat.startswith("human.pedestrian"):
        return "pedestrian"
    for k in ("bicycle", "motorcycle", "bus", "truck"):
        if k in cat:
            return k
    return "car"


def _traj_group(kind):
    if kind == "pedestrian":
        return "pedestrian"
    if kind in ("bicycle", "motorcycle"):
        return "cyclist"
    return "vehicle"


def _traj_color(group, is_future):
    r, g, b = _TRAJ_BASE.get(group, _TRAJ_BASE["vehicle"])
    f = 1.0 if is_future else 0.35
    return carla.Color(int(r * f), int(g * f), int(b * f))


def _pick_blueprint(world, kind, key):
    """Deterministically pick a blueprint for a kind, given a stable key."""
    lib = world.get_blueprint_library()
    for pattern in _BP_CANDIDATES.get(kind, _BP_CANDIDATES["car"]):
        bps = lib.filter(pattern)
        if bps:
            bp = bps[key % len(bps)]
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", "nuscenes_agent")
            if bp.has_attribute("color"):
                bp.set_attribute("color", _AGENT_COLORS[key % len(_AGENT_COLORS)])
            return bp
    return lib.filter("vehicle.*")[0]


def _z_offset(actor):
    """z to add to the actor's origin so its bounding-box bottom rests on z=0."""
    try:
        bb = actor.bounding_box
        return bb.extent.z - bb.location.z
    except Exception:
        return 0.0


# nuScenes-style 6-camera rig, ego-relative CARLA coords (x fwd, +y right,
# z up, yaw deg clockwise) - approximates the real nuScenes mounts
NUSCENES_CAMS = [
    # name,            x,     y,    z,    yaw,    fov
    ("CAM_FRONT",       1.70,  0.00, 1.51,    0.0,  70),
    ("CAM_FRONT_LEFT",  1.55, -0.50, 1.51,  -55.0,  70),
    ("CAM_FRONT_RIGHT", 1.55,  0.50, 1.51,   55.0,  70),
    ("CAM_BACK",       -1.50,  0.00, 1.56,  180.0, 110),
    ("CAM_BACK_LEFT",   0.00, -0.55, 1.56, -110.0,  70),
    ("CAM_BACK_RIGHT",  0.00,  0.55, 1.56,  110.0,  70),
]


# --------------------------------------------------------------- rrhd boxes
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


def load_rrhd_buildings(base):
    """Building boxes placed in the RoadRunner scene (<base>_city.rrhd).

    Decodes the BldgN_M StaticObject oriented bounding boxes written by
    nuscenes_ground_buildings.m: rows (cx, cy, hx, hy, yaw, height) in
    patch-local metres (hx/hy half-extents, yaw radians, height = 2*cz).
    Same frame as the geo.xodr / meta sidecar.
    """
    path = os.path.join(RR_DIR, base, base + "_city.rrhd")
    if not os.path.isfile(path):
        return []
    with open(path, "rb") as f:
        b = f.read()
    boxes = []
    for m in re.finditer(rb"\x0a(.)Bldg", b, re.DOTALL):
        ln = m.group(1)[0] if not isinstance(m.group(1), str) else ord(m.group(1))
        name = b[m.start() + 2: m.start() + 2 + ln]
        if not re.match(rb"Bldg[0-9]+_[0-9]+$", name):
            continue
        i = m.start() + 2 + ln
        if i >= len(b) or b[i:i + 1] != b"\x12":
            continue
        glen, j = _pb_varint(b, i + 1)
        g = b[j:j + glen]
        fields, k = {}, 0
        while k < len(g):
            key, k = _pb_varint(g, k)
            f2, wt = key >> 3, key & 7
            if wt == 2:
                l2, k = _pb_varint(g, k)
                fields[f2] = g[k:k + l2]
                k += l2
            elif wt == 1:
                k += 8
            else:
                _, k = _pb_varint(g, k)
        c = _pb_triple(fields.get(1, b""))
        d = _pb_triple(fields.get(2, b""))
        yaw = 0.0
        if 3 in fields:
            inner = fields[3]
            key, k = _pb_varint(inner, 0)
            if key & 7 == 2:
                l2, k = _pb_varint(inner, k)
                yaw = _pb_triple(inner[k:k + l2]).get(3, 0.0)
        boxes.append((c.get(1, 0.0), c.get(2, 0.0), d.get(1, 0.0),
                      d.get(2, 0.0), yaw, 2.0 * c.get(3, 0.0)))
    return boxes


def draw_rr_buildings_wire(world, boxes, max_n):
    """Outline the RoadRunner building boxes with persistent debug lines.

    Instant (no actor spawning): one wireframe box per building, drawn with
    world.debug.draw_box (life_time=0 -> persists until the world is reloaded).
    Debug lines render in the spectator and the RGB camera sensors alike.
    """
    items = boxes[:max_n] if max_n else boxes
    color = carla.Color(77, 204, 221)          # cyan, matching the top-down
    for (bx, by, hx, hy, byaw, bh) in items:
        cx, cy, cyaw = carla_from_opendrive(bx, by, byaw)
        h = bh if bh > 0 else 10.0
        bb = carla.BoundingBox(
            carla.Location(cx, cy, h / 2.0),
            carla.Vector3D(hx, hy, h / 2.0))
        world.debug.draw_box(bb, carla.Rotation(yaw=cyaw), thickness=0.12,
                             color=color, life_time=0.0)
    print("Drew %d wireframe RoadRunner building boxes." % len(items))


_BUILDING_PROPS = ["static.prop.container", "static.prop.clothcontainer",
                   "static.prop.busstop"]


def spawn_rr_buildings(world, boxes, layers, max_n):
    """Fill each RoadRunner building box with a tiled static-prop mesh.

    CARLA can't spawn arbitrary-size buildings, so (like the original loader)
    a box-shaped prop is tiled across each oriented box footprint and stacked
    up to `layers` high, capped by the recorded building height.
    """
    lib = world.get_blueprint_library()
    bp = None
    for name in _BUILDING_PROPS:
        found = lib.filter(name)
        if found:
            bp = found[0]
            break
    if bp is None:
        print("No building prop blueprint available - skipping buildings.")
        return []
    probe = world.try_spawn_actor(bp, carla.Transform(carla.Location(0, 0, 300)))
    if probe is None:
        print("Could not probe building prop - skipping buildings.")
        return []
    ext = probe.bounding_box.extent
    pw, pl, ph = max(ext.x * 2, 0.5), max(ext.y * 2, 0.5), max(ext.z * 2, 0.5)
    probe.destroy()

    items = boxes[:max_n] if max_n else boxes
    actors = []
    for (bx, by, hx, hy, byaw, bh) in items:
        cx, cy, cyaw = carla_from_opendrive(bx, by, byaw)
        yr = math.radians(cyaw)
        ux, uy = math.cos(yr), math.sin(yr)        # box local axes (CARLA frame)
        vx, vy = -math.sin(yr), math.cos(yr)
        nx = max(1, int(round(2.0 * hx / pw)))
        ny = max(1, int(round(2.0 * hy / pl)))
        nz = max(1, min(layers, int(bh // ph) if bh > 0 else layers))
        for iz in range(nz):
            for ix in range(nx):
                for iy in range(ny):
                    ox = (ix - (nx - 1) / 2.0) * pw
                    oy = (iy - (ny - 1) / 2.0) * pl
                    tf = carla.Transform(
                        carla.Location(cx + ox * ux + oy * vx,
                                       cy + ox * uy + oy * vy,
                                       ph / 2.0 + iz * ph),
                        carla.Rotation(yaw=cyaw))
                    a = world.try_spawn_actor(bp, tf)
                    if a is not None:
                        actors.append(a)
    print("Spawned %d '%s' props for %d RoadRunner building boxes."
          % (len(actors), bp.id, len(items)))
    return actors


# --------------------------------------------------------------- camera rig
class _CamRig(object):
    """Approximate nuScenes 6-camera rig attached to the ego."""

    def __init__(self, world, ego, width, height):
        lib = world.get_blueprint_library()
        self.items = []
        for name, x, y, z, yaw, fov in NUSCENES_CAMS:
            bp = lib.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(width))
            bp.set_attribute("image_size_y", str(height))
            bp.set_attribute("fov", str(fov))
            cam = world.spawn_actor(
                bp, carla.Transform(carla.Location(x, y, z),
                                    carla.Rotation(yaw=yaw)),
                attach_to=ego)
            q = queue.Queue()
            cam.listen(q.put)
            self.items.append({"name": name, "sensor": cam, "q": q})
        print("Attached approximate nuScenes rig: %d cams (%dx%d)."
              % (len(self.items), width, height))

    def drain(self, save_dir=None, kf_idx=None):
        """Pop one frame per camera for this tick; save when a target given."""
        for it in self.items:
            try:
                img = it["q"].get(timeout=2.0)
            except queue.Empty:
                continue
            if save_dir is not None:
                img.save_to_disk(os.path.join(
                    save_dir, it["name"], "%03d.png" % kf_idx))

    def destroy(self):
        for it in self.items:
            try:
                it["sensor"].stop()
                it["sensor"].destroy()
            except Exception:
                pass


# --------------------------------------------------------------- scene files
def discover_scenes():
    bases = []
    if os.path.isdir(RR_DIR):
        for d in sorted(os.listdir(RR_DIR)):
            if os.path.isfile(os.path.join(RR_DIR, d, d + "_geo.xodr")):
                bases.append(d)
    return bases


def load_scene(base):
    xodr_path = os.path.join(RR_DIR, base, base + "_geo.xodr")
    meta_path = os.path.join(RR_DIR, base + "_meta.json")
    with open(xodr_path, "r") as f:
        xodr = f.read()
    with open(meta_path, "r") as f:
        meta = json.load(f)
    return xodr, meta


# --------------------------------------------------------------- recorder
class _ChaseRecorder(object):
    """Free-floating RGB camera teleported to the chase pose each tick."""

    def __init__(self, world, width, height):
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", "90")
        self.sensor = world.spawn_actor(
            bp, carla.Transform(carla.Location(z=100.0)))
        self.q = queue.Queue()
        self.sensor.listen(self.q.put)

    def place(self, tf):
        self.sensor.set_transform(tf)

    def drain(self, path=None):
        """Consume this tick's image; save it when a path is given."""
        img = None
        try:
            img = self.q.get(timeout=2.0)
            while True:                     # keep only the newest
                img = self.q.get_nowait()
        except queue.Empty:
            pass
        if path is not None and img is not None:
            img.save_to_disk(path)

    def destroy(self):
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except Exception:
            pass


# --------------------------------------------------------------- gif
def write_gif(frames_dir, gif_path, duration_ms, scale=0.5):
    """Assemble the recorded PNGs into a GIF (needs Pillow in this env)."""
    try:
        from PIL import Image
    except ImportError:
        print("Pillow not installed in this env - build the GIF later with:")
        print("  python make_gifs.py          (base Python 3.12 env)")
        return
    pngs = sorted(glob.glob(os.path.join(frames_dir, "*.png")))
    if not pngs:
        return
    images = []
    for p in pngs:
        img = Image.open(p).convert("RGB")
        if scale != 1.0:
            img = img.resize((int(img.width * scale), int(img.height * scale)),
                             Image.LANCZOS)
        images.append(img.quantize(colors=256))
    images[0].save(gif_path, save_all=True, append_images=images[1:],
                   duration=int(duration_ms), loop=0, optimize=False)
    print("GIF ->", gif_path, "(%d frames)" % len(images))


# --------------------------------------------------------------- replay
def replay_scene(client, base, args):
    print("=" * 60)
    print("Scenario:", base, "(geo map)")
    xodr, meta = load_scene(base)

    params = carla.OpendriveGenerationParameters(
        vertex_distance=args.vertex_distance,
        max_road_length=args.max_road_length,
        wall_height=args.wall_height,
        additional_width=args.extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True,
    )
    print("Generating world from %s_geo.xodr (this can take a moment)..." % base)
    world = client.generate_opendrive_world(xodr, params)
    cmap = world.get_map()
    print("Map loaded:", cmap.name, "| spawn points:", len(cmap.get_spawn_points()))

    if args.weather and args.weather.lower() != "none":
        try:
            world.set_weather(getattr(carla.WeatherParameters, args.weather))
        except AttributeError:
            print("Unknown weather preset:", args.weather)

    building_actors = []
    if args.buildings:
        rr_boxes = load_rrhd_buildings(base)
        if not rr_boxes:
            print("No RoadRunner building boxes for", base)
        elif args.solid_buildings:      # slow: tiles real props
            building_actors = spawn_rr_buildings(
                world, rr_boxes, args.building_layers, args.max_buildings)
        else:                           # fast default: wireframe outlines
            draw_rr_buildings_wire(world, rr_boxes, args.max_buildings)

    # --- spawn the ego at the recorded start pose, snapped onto the road
    e = meta["ego_start_local"]
    cx, cy, cyaw = carla_from_opendrive(e["x"], e["y"], e["yaw"])
    wp = cmap.get_waypoint(carla.Location(x=cx, y=cy, z=0.5),
                           project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is not None:
        spawn = carla.Transform(
            carla.Location(wp.transform.location.x, wp.transform.location.y,
                           wp.transform.location.z + 0.5),
            wp.transform.rotation)
    else:
        spawn = carla.Transform(carla.Location(x=cx, y=cy, z=0.5),
                                carla.Rotation(yaw=cyaw))
    bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    if bp.has_attribute("color"):
        bp.set_attribute("color", "200,0,0")            # red ego
    ego = world.try_spawn_actor(bp, spawn)
    if ego is None:
        print("Could not spawn ego at", spawn.location, "- skipping scene.")
        return
    print("Spawned ego at", spawn.location, "yaw", spawn.rotation.yaw)

    # --- replay setup
    traj = meta["ego_trajectory_local"]
    frames = [] if args.no_agents else meta.get("agent_frames", [])
    fdt = float(meta.get("frame_dt", 0.5))
    dt = fdt / max(args.replay_speed, 1e-3)
    substeps = max(1, args.substeps)
    sub_dt = dt / substeps
    n = len(traj)

    allowed = None
    if args.max_agents and frames:
        seen, order = set(), []
        for fr in frames:
            for a in fr:
                if a["id"] not in seen:
                    seen.add(a["id"])
                    order.append(a["id"])
        allowed = set(order[:args.max_agents])

    # per-instance keyframe positions + display group, for the overlay
    agent_xy, agent_group = {}, {}
    for fi, fr in enumerate(frames):
        for a in fr:
            agent_xy.setdefault(a["id"], {})[fi] = (a["x"], a["y"])
            agent_group.setdefault(a["id"],
                                   _traj_group(_category_kind(a["category"])))

    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = max(sub_dt, 0.01)
    world.apply_settings(settings)

    ego.set_simulate_physics(False)
    ego_zoff = _z_offset(ego)
    actors, kinds, zoff = {}, {}, {}
    hidden = carla.Location(0.0, 0.0, -200.0)
    spectator = world.get_spectator()

    recorder = None
    rec_dir = None
    if args.record:
        rec_dir = os.path.join(RESULTS, base, "carla_frames")
        if not os.path.isdir(rec_dir):
            os.makedirs(rec_dir)
        recorder = _ChaseRecorder(world, args.rec_width, args.rec_height)

    rig = None
    cam_dir = None
    if args.cameras:
        cam_dir = os.path.join(RESULTS, base, "carla_cams")
        rig = _CamRig(world, ego, args.cam_width, args.cam_height)

    # ground height cached on a 1 m grid (waypoint queries dominate frame cost)
    gz_cache = {}

    def gz(x, y):
        key = (round(x), round(y))
        v = gz_cache.get(key)
        if v is None:
            w = cmap.get_waypoint(carla.Location(x, y, 0.0), project_to_road=True)
            v = w.transform.location.z if w else 0.0
            gz_cache[key] = v
        return v

    def place(actor, x, y, yaw_deg, zo):
        actor.set_transform(carla.Transform(
            carla.Location(x, y, gz(x, y) + zo), carla.Rotation(yaw=yaw_deg)))

    def ensure_spawned(a):
        iid = a["id"]
        if iid in actors or (allowed is not None and iid not in allowed):
            return
        kind = kinds.setdefault(iid, _category_kind(a["category"]))
        ax, ay, ayaw = carla_from_opendrive(a["x"], a["y"], a["yaw"])
        bp2 = _pick_blueprint(world, kind, sum(bytearray(iid.encode())))
        tf = carla.Transform(carla.Location(ax, ay, gz(ax, ay) + 1.0),
                             carla.Rotation(yaw=ayaw))
        act = world.try_spawn_actor(bp2, tf)
        if act is None:                     # retry higher (spawn collision)
            tf.location.z = gz(ax, ay) + 3.0
            act = world.try_spawn_actor(bp2, tf)
        if act is None:
            return
        try:
            act.set_simulate_physics(False)
        except Exception:
            pass
        zoff[iid] = _z_offset(act)
        actors[iid] = act

    # --- trajectory overlay (debug lines at traj_hz over the 2 Hz keyframes)
    step = max(1, int(round((1.0 / args.traj_hz) / fdt)))
    hn = int(round(args.hist_seconds * args.traj_hz))
    fn = int(round(args.future_seconds * args.traj_hz))
    traj_life = dt * 1.5

    def draw_polyline(pos, indices, group, is_future):
        avail = []
        for k in indices:
            p = pos(k)
            if p is None:
                continue
            px, py = p[0], -p[1]            # OpenDRIVE -> CARLA (flip Y)
            avail.append((k, carla.Location(px, py, gz(px, py) + 0.25)))
        col = _traj_color(group, is_future)
        for (k1, l1), (k2, l2) in zip(avail, avail[1:]):
            if k2 - k1 == step:
                world.debug.draw_line(l1, l2, thickness=0.04, color=col,
                                      life_time=traj_life)
        for _, loc in avail:
            world.debug.draw_point(loc, size=0.08, color=col,
                                   life_time=traj_life)

    def draw_trajectories(i, present):
        if args.no_traj:
            return
        hist_idx = list(range(i - hn * step, i + 1, step))
        fut_idx = list(range(i, i + fn * step + 1, step))

        def ego_pos(k):
            return (traj[k]["x"], traj[k]["y"]) if 0 <= k < n else None
        draw_polyline(ego_pos, hist_idx, "ego", False)
        draw_polyline(ego_pos, fut_idx, "ego", True)
        for iid in present:
            xy = agent_xy.get(iid)
            if not xy:
                continue
            grp = agent_group[iid]
            draw_polyline(xy.get, hist_idx, grp, False)
            draw_polyline(xy.get, fut_idx, grp, True)

    n_inst = len(agent_xy)
    print("Replaying %d keyframes x %d substeps with up to %d agents. Ctrl+C to stop."
          % (n, substeps,
             n_inst if allowed is None else min(n_inst, args.max_agents)))

    def run_once():
        cam_yaw = [None]
        back, up = 8.0, 4.0
        for i in range(n):
            P0 = traj[max(i - 1, 0)]
            P1 = traj[i]
            P2 = traj[min(i + 1, n - 1)]
            P3 = traj[min(i + 2, n - 1)]
            a0 = {a["id"]: a for a in (frames[i] if i < len(frames) else [])}
            a1 = {a["id"]: a for a in
                  (frames[min(i + 1, n - 1)] if frames else [])}
            for a in a0.values():
                ensure_spawned(a)
            present = set(a0)
            draw_trajectories(i, present)
            for s in range(substeps):
                r = s / float(substeps)
                ex = _catmull(P0["x"], P1["x"], P2["x"], P3["x"], r)
                ey = _catmull(P0["y"], P1["y"], P2["y"], P3["y"], r)
                eyaw = _lerp_angle(P1["yaw"], P2["yaw"], r)
                ecx, ecy, ecyaw = carla_from_opendrive(ex, ey, eyaw)
                place(ego, ecx, ecy, ecyaw, ego_zoff)
                for iid, a in a0.items():
                    act = actors.get(iid)
                    if act is None:
                        continue
                    b = a1.get(iid, a)      # hold pose if it leaves next frame
                    gx, gy, gyaw = carla_from_opendrive(
                        _lerp(a["x"], b["x"], r), _lerp(a["y"], b["y"], r),
                        _lerp_angle(a["yaw"], b["yaw"], r))
                    place(act, gx, gy, gyaw, zoff.get(iid, 0.0))
                for iid, act in actors.items():
                    if iid not in present:
                        act.set_transform(carla.Transform(hidden))
                # damped chase camera (EMA on yaw removes swing when stopped)
                if cam_yaw[0] is None:
                    cam_yaw[0] = ecyaw
                else:
                    cam_yaw[0] += ((ecyaw - cam_yaw[0] + 180.0) % 360.0
                                   - 180.0) * 0.2
                yawr = math.radians(cam_yaw[0])
                cam_tf = carla.Transform(
                    carla.Location(ecx - back * math.cos(yawr),
                                   ecy - back * math.sin(yawr),
                                   gz(ecx, ecy) + up),
                    carla.Rotation(pitch=-15.0, yaw=cam_yaw[0]))
                spectator.set_transform(cam_tf)
                if recorder is not None:
                    recorder.place(cam_tf)
                world.tick()
                if recorder is not None:
                    recorder.drain(os.path.join(rec_dir, "%03d.png" % i)
                                   if s == 0 else None)
                if rig is not None:      # save the 6 views at the keyframe pose
                    if s == 0:
                        rig.drain(save_dir=cam_dir, kf_idx=i)
                    else:
                        rig.drain()
                time.sleep(sub_dt)

    try:
        run_once()
        while args.loop:
            run_once()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Cleaning up %d replay agents..." % len(actors))
        if recorder is not None:
            recorder.destroy()
        if rig is not None:
            rig.destroy()
        client.apply_batch([carla.command.DestroyActor(a)
                            for a in list(actors.values()) + building_actors])
        try:
            ego.destroy()
        except Exception:
            pass
        world.apply_settings(orig_settings)     # restore async mode
    if cam_dir:
        print("6-cam frames ->", cam_dir)
    if rec_dir:
        print("Chase-cam frames ->", rec_dir)
        if not args.no_gif:
            write_gif(rec_dir,
                      os.path.join(RESULTS, base, base + "_carla.gif"),
                      duration_ms=fdt * 1000.0, scale=args.gif_scale)
    print("Done:", base)


# --------------------------------------------------------------- CLI
def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--scene", help="scene id substring, e.g. 0103 or scene-0061")
    p.add_argument("--all", action="store_true", help="replay every geo scene back-to-back")
    p.add_argument("--list", action="store_true", help="list available scenes and exit")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=120.0)
    # replay behaviour
    p.add_argument("--replay-speed", type=float, default=1.0, help="1.0 = real time")
    p.add_argument("--substeps", type=int, default=10, help="interp steps per 2 Hz keyframe")
    p.add_argument("--max-agents", type=int, default=0, help="cap agents (0 = all)")
    p.add_argument("--no-agents", action="store_true", help="replay the ego only")
    p.add_argument("--loop", action="store_true", help="loop the replay until Ctrl+C")
    # trajectory overlay
    p.add_argument("--no-traj", action="store_true", help="hide the trajectory overlay")
    p.add_argument("--hist-seconds", type=float, default=2.0)
    p.add_argument("--future-seconds", type=float, default=6.0)
    p.add_argument("--traj-hz", type=float, default=2.0)
    # world generation
    p.add_argument("--vertex-distance", type=float, default=2.0)
    p.add_argument("--max-road-length", type=float, default=50.0)
    p.add_argument("--wall-height", type=float, default=0.0)
    p.add_argument("--extra-width", type=float, default=0.6)
    p.add_argument("--weather", default="ClearNoon")
    # buildings (from the RoadRunner .rrhd boxes)
    p.add_argument("--buildings", action="store_true",
                   help="show the RoadRunner building boxes (wireframe outlines "
                        "by default - instant)")
    p.add_argument("--solid-buildings", action="store_true",
                   help="with --buildings: build solid blocks from tiled props "
                        "instead of outlines (slow)")
    p.add_argument("--building-layers", type=int, default=3,
                   help="max prop layers to stack per building (solid mode)")
    p.add_argument("--max-buildings", type=int, default=0,
                   help="cap the number of building boxes (0 = all)")
    # nuScenes-style 6-camera rig
    p.add_argument("--cameras", action="store_true",
                   help="attach the 6-cam rig; save per-keyframe PNGs to "
                        "results/<base>/carla_cams/<CAM>/")
    p.add_argument("--cam-width", type=int, default=800)
    p.add_argument("--cam-height", type=int, default=450)
    # recording
    p.add_argument("--record", action="store_true",
                   help="save one chase-cam PNG per keyframe to results/<base>/carla_frames")
    p.add_argument("--rec-width", type=int, default=1280)
    p.add_argument("--rec-height", type=int, default=720)
    p.add_argument("--no-gif", action="store_true",
                   help="don't assemble the recorded frames into a GIF")
    p.add_argument("--gif-scale", type=float, default=0.5,
                   help="GIF resize factor (0.5 -> 640x360)")
    args = p.parse_args()

    scenes = discover_scenes()
    if args.list or not (args.scene or args.all):
        print("Scenes with a generated geo map:")
        for i, b in enumerate(scenes):
            print("  [%d] %s" % (i, b))
        if not (args.scene or args.all):
            print("\nPick one with --scene <id> or run --all.")
        return

    if args.all:
        todo = scenes
    else:
        todo = [b for b in scenes if args.scene in b]
        if not todo:
            sys.exit("no scene matching '%s' - try --list" % args.scene)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    print("CARLA server version:", client.get_server_version())

    for base in todo:
        replay_scene(client, base, args)


if __name__ == "__main__":
    main()
