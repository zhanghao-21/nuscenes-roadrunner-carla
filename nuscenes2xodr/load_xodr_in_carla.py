"""
Load a generated .xodr into a running CARLA 0.9.15 server (standalone OpenDRIVE
mode) and spawn the nuScenes ego vehicle at the scene's start pose.

Run this with the Python 3.7 conda env that has the bundled carla wheel:

    conda activate carla915
    # 1) start the simulator first (separate terminal):
    #    D:/research/nuscenes_carla/carla_0.9.15/CarlaUE4.exe
    # 2) then:
    python load_xodr_in_carla.py \
        --xodr output/singapore-onenorth_scene0061.xodr

Coordinate note: CARLA is left-handed; when it ingests OpenDRIVE it flips Y.
So an OpenDRIVE point (x, y, yaw) becomes CARLA (x, -y, -yaw).
"""

import argparse
import math
import os
import json

import numpy as np
import carla


def carla_from_opendrive(x, y, yaw_rad):
    """OpenDRIVE (right-handed) -> CARLA (left-handed): flip Y and yaw."""
    return x, -y, -math.degrees(yaw_rad)


def _carla_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """CARLA/UE left-handed rotation matrix; forward = first column."""
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    return np.array([
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
        [sp, -cp * sr, cp * cr],
    ])


def _carla_euler_from_matrix(R):
    """Inverse of _carla_rotation_matrix: (roll, pitch, yaw) in degrees."""
    pitch = math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    roll = math.atan2(-R[2, 1], R[2, 2])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


DEFAULT_OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
# phase0 calibration sidecars (real K/extrinsics per scene); when one exists
# for the scene, the --cameras rig is CALIBRATED by default (see _CameraRig)
DEFAULT_CALIB_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "experiments",
    "phase0_calib_gt", "output"))


def discover_scenarios(outdir):
    """List generated .xodr files ordered by nuScenes scene number."""
    import glob
    import re
    files = glob.glob(os.path.join(outdir, "*.xodr"))

    def key(f):
        m = re.search(r"scene-?(\d+)", os.path.basename(f))
        return int(m.group(1)) if m else 1 << 30
    return sorted(files, key=key)


def resolve_xodr(args):
    """Pick the .xodr from --xodr, or --scenario-id / --scene against outdir."""
    if args.xodr:
        return args.xodr
    scenarios = discover_scenarios(args.outdir)
    if not scenarios:
        raise SystemExit("No .xodr files in %s - run the converter first." % args.outdir)
    if args.scene is not None:
        key = str(args.scene)
        for f in scenarios:
            if key in os.path.basename(f):
                return f
        raise SystemExit("No scenario matching '%s'. Use --list." % args.scene)
    if not 0 <= args.scenario_id < len(scenarios):
        raise SystemExit("scenario-id %d out of range 0..%d. Use --list."
                         % (args.scenario_id, len(scenarios) - 1))
    return scenarios[args.scenario_id]


def print_scenarios(outdir):
    scenarios = discover_scenarios(outdir)
    print("Available scenarios in %s:" % outdir)
    for i, f in enumerate(scenarios):
        print("  [%d] %s" % (i, os.path.basename(f)))


def main():
    ap = argparse.ArgumentParser()
    # scenario selection (use ONE of these; --xodr wins if given)
    ap.add_argument("--scenario-id", type=int, default=0,
                    help="pick scenario 0..N from the output dir (nuScenes order)")
    ap.add_argument("--scene", default=None,
                    help="pick scenario by scene number/name, e.g. 0103 or scene-0103")
    ap.add_argument("--xodr", default=None, help="explicit path to a .xodr file")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR,
                    help="folder holding the generated .xodr files")
    ap.add_argument("--list", action="store_true",
                    help="list available scenarios and exit")
    ap.add_argument("--all-scenarios", action="store_true",
                    help="run every scenario in the output dir back-to-back "
                         "(each plays once; great with --follow/--replay --cameras)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--no-ego", action="store_true",
                    help="just load the map, don't spawn the ego")
    ap.add_argument("--vertex-distance", type=float, default=2.0)
    ap.add_argument("--max-road-length", type=float, default=500.0)
    ap.add_argument("--wall-height", type=float, default=0.0)
    ap.add_argument("--extra-width", type=float, default=0.6)
    # driving
    ap.add_argument("--no-drive", action="store_true",
                    help="spawn but don't enable autopilot / follow loop")
    ap.add_argument("--manual", action="store_true",
                    help="drive the ego yourself (keyboard/controller) in a "
                         "pygame window, instead of replay/autopilot")
    ap.add_argument("--joystick", action="store_true",
                    help="manual mode: use a connected gamepad/steering wheel")
    ap.add_argument("--follow", action="store_true",
                    help="drive the recorded ego path physically with CARLA's "
                         "PID controller (planning=recorded path, control=PID)")
    ap.add_argument("--carla-agents",
                    default=r"D:/research/nuscenes_carla/carla_0.9.15/PythonAPI/carla",
                    help="path to CARLA PythonAPI 'agents' package (for --follow)")
    ap.add_argument("--replay", default=True, action="store_true",
                    help="drive along the recorded nuScenes ego trajectory "
                         "instead of CARLA autopilot")
    ap.add_argument("--no-agents", action="store_true",
                    help="replay: don't spawn the other nuScenes agents")
    ap.add_argument("--max-agents", type=int, default=0,
                    help="replay: cap number of other agents (0 = no cap)")
    ap.add_argument("--replay-speed", type=float, default=1.0,
                    help="replay playback speed (1.0 = real time, <1 slower)")
    ap.add_argument("--substeps", type=int, default=10,
                    help="interpolation steps between 2 Hz keyframes (smoothness)")
    # trajectory overlay
    ap.add_argument("--no-traj", action="store_true",
                    help="don't draw the ego history/future trajectory lines")
    ap.add_argument("--hist-seconds", type=float, default=2.0,
                    help="seconds of past ego trajectory to draw")
    ap.add_argument("--future-seconds", type=float, default=6.0,
                    help="seconds of future ego trajectory to draw")
    ap.add_argument("--traj-hz", type=float, default=2.0,
                    help="sampling rate (Hz) of the drawn trajectory points")
    # nuScenes-style 6-camera rig
    ap.add_argument("--cameras", action="store_true",
                    help="attach 6 nuScenes-style cameras to the ego and save "
                         "their views each keyframe (replay only)")
    ap.add_argument("--cam-dir", default=os.path.join(DEFAULT_OUTDIR, "cameras"),
                    help="output folder for saved camera frames")
    ap.add_argument("--cam-width", type=int, default=800,
                    help="camera image width (approx rig only; the calibrated "
                         "rig always renders at the nuScenes native size)")
    ap.add_argument("--cam-height", type=int, default=450,
                    help="camera image height (approx rig only)")
    ap.add_argument("--calib-dir", default=DEFAULT_CALIB_DIR,
                    help="folder with phase0 <map>_<scene>_calib_gt.json "
                         "sidecars; when the scene's sidecar exists, --cameras "
                         "uses the CALIBRATED rig (real per-camera FOV + "
                         "extrinsics, native 1600x900)")
    ap.add_argument("--approx-rig", action="store_true",
                    help="force the legacy approximate attached camera rig "
                         "even when a calibration sidecar exists")
    ap.add_argument("--no-loop", action="store_true",
                    help="replay each scene once instead of looping")
    # nuScenes-style LIDAR_TOP (Velodyne HDL-32E), matching plot_lidar.py's source
    ap.add_argument("--lidar", action="store_true",
                    help="attach a nuScenes-style (HDL-32E) LiDAR to the ego and "
                         "save one full sweep per keyframe (replay/follow only)")
    ap.add_argument("--lidar-dir", default=os.path.join(DEFAULT_OUTDIR, "lidar_carla"),
                    help="output folder for saved LiDAR sweeps (.ply per keyframe)")
    ap.add_argument("--lidar-range", type=float, default=70.0,
                    help="LiDAR max range in metres (nuScenes LIDAR_TOP ~70)")
    ap.add_argument("--lidar-channels", type=int, default=32,
                    help="LiDAR beam count (nuScenes HDL-32E = 32)")
    ap.add_argument("--lidar-points", type=int, default=34000,
                    help="points per full 360 sweep (nuScenes ~34k); scaled by the "
                         "tick rate to set CARLA's points_per_second")
    ap.add_argument("--loop", default=True, action="store_true",
                    help="replay: loop forever until Ctrl+C")
    # scenery
    ap.add_argument("--buildings", action="store_true",
                    help="render synthesized building blocks from the sidecar")
    ap.add_argument("--max-buildings", type=int, default=0,
                    help="cap number of buildings (0 = all)")
    ap.add_argument("--building-layers", type=int, default=3,
                    help="how many prop layers to stack per building (height)")
    ap.add_argument("--wireframe-buildings", action="store_true",
                    help="draw buildings as wireframe boxes instead of solid props")
    ap.add_argument("--no-crosswalks", action="store_true",
                    help="don't draw crosswalk zebra stripes")
    ap.add_argument("--weather", default="ClearNoon",
                    help="WeatherParameters preset name (e.g. ClearNoon, "
                         "CloudySunset); 'none' to leave unchanged")
    ap.add_argument("--tm-port", type=int, default=8000,
                    help="Traffic Manager port (autopilot mode)")
    ap.add_argument("--speed-diff", type=float, default=30.0,
                    help="autopilot: %% slower than speed limit (higher=slower)")
    args = ap.parse_args()
    if args.no_loop:
        args.loop = False

    if args.list:
        print_scenarios(args.outdir)
        return

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    print("CARLA server:", client.get_server_version())

    if args.all_scenarios:
        args.loop = False                    # each scenario must finish to advance
        scenarios = discover_scenarios(args.outdir)
        if not scenarios:
            print("No scenarios in %s - run the converter first." % args.outdir)
            return
        for i, xp in enumerate(scenarios):
            print("\n========== [%d/%d] %s =========="
                  % (i + 1, len(scenarios), os.path.basename(xp)))
            try:
                run_scenario(client, xp, args)
            except KeyboardInterrupt:
                print("\nInterrupted; stopping batch.")
                break
            except Exception as exc:
                print("  scenario failed:", exc)
        print("All scenarios done.")
    else:
        try:
            run_scenario(client, resolve_xodr(args), args)
        except KeyboardInterrupt:
            print("\nStopped by user.")


def run_scenario(client, xodr_path, args):
    """Load one .xodr world, spawn the ego, and drive it in the selected mode."""
    print("Scenario:", os.path.basename(xodr_path))
    with open(xodr_path, "r", encoding="utf-8") as f:
        xodr = f.read()

    meta = {}
    meta_path = os.path.splitext(xodr_path)[0] + "_meta.json"
    if os.path.exists(meta_path):
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

    params = carla.OpendriveGenerationParameters(
        vertex_distance=args.vertex_distance,
        max_road_length=args.max_road_length,
        wall_height=args.wall_height,
        additional_width=args.extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True,
    )
    print("Generating world from OpenDRIVE (this can take a moment)...")
    world = client.generate_opendrive_world(xodr, params)
    cmap = world.get_map()
    print("Map loaded:", cmap.name, "| spawn points:",
          len(cmap.get_spawn_points()))

    # nicer lighting
    if args.weather and args.weather.lower() != "none":
        try:
            world.set_weather(getattr(carla.WeatherParameters, args.weather))
        except AttributeError:
            print("Unknown weather preset:", args.weather)

    # crosswalk zebra stripes (CARLA doesn't paint OpenDRIVE crosswalk objects)
    if not args.no_crosswalks and meta.get("crosswalks"):
        _draw_crosswalks(world, meta["crosswalks"])

    # synthesized buildings (nuScenes has no building layer)
    if args.buildings and meta.get("buildings"):
        if args.wireframe_buildings:
            _draw_buildings_wire(world, meta["buildings"], args.max_buildings)
        else:
            _spawn_buildings(world, meta["buildings"], args.max_buildings,
                             args.building_layers)

    if args.no_ego or not meta.get("ego_start_local"):
        # park the spectator above the network and exit
        sps = cmap.get_spawn_points()
        if sps:
            loc = sps[0].location
            world.get_spectator().set_transform(carla.Transform(
                carla.Location(loc.x, loc.y, loc.z + 60.0),
                carla.Rotation(pitch=-90.0)))
        print("Map-only mode: done.")
        return

    e = meta["ego_start_local"]
    cx, cy, cyaw = carla_from_opendrive(e["x"], e["y"], e["yaw"])
    loc = carla.Location(x=cx, y=cy, z=0.5)

    # snap to the nearest drivable waypoint so the car spawns on the road
    wp = cmap.get_waypoint(loc, project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    if wp is not None:
        spawn = carla.Transform(
            carla.Location(wp.transform.location.x,
                           wp.transform.location.y,
                           wp.transform.location.z + 0.5),
            wp.transform.rotation)
    else:
        spawn = carla.Transform(loc, carla.Rotation(yaw=cyaw))

    bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    if bp.has_attribute("color"):
        bp.set_attribute("color", "200,0,0")        # red ego vehicle
    ego = world.try_spawn_actor(bp, spawn)
    if ego is None:
        print("Could not spawn ego at", spawn.location,
              "- try a different scene region or check the mesh.")
        return
    print("Spawned ego at", spawn.location, "yaw", spawn.rotation.yaw)
    _chase_cam(world, ego)

    if args.no_drive:
        print("Spawned (no-drive). The world stays loaded on the server.")
        return

    try:
        if args.manual:
            _manual_drive(client, world, ego, meta, args)
        elif args.follow and meta.get("ego_trajectory_local"):
            _follow_trajectory(client, world, ego, meta, args)
        elif args.replay and meta.get("ego_trajectory_local"):
            _replay_trajectory(client, world, ego, meta, args)
        else:
            _autopilot_drive(client, world, ego, args.tm_port, args.speed_diff)
    finally:
        # leave the car on the server; only stop autopilot cleanly
        try:
            ego.set_autopilot(False)
        except Exception:
            pass
    print("Done:", os.path.basename(xodr_path))


# static-prop meshes used as solid building blocks (first available wins)
_BUILDING_PROPS = ["static.prop.container", "static.prop.clothcontainer",
                   "static.prop.busstop"]


def _spawn_buildings(world, buildings, max_n, layers):
    """Build solid blocks by tiling a box-shaped static-prop mesh.

    CARLA can't spawn arbitrary-sized buildings, so we tile a real prop (a
    shipping container by default) across each synthesized footprint and stack
    it `layers` high. Falls back to wireframe boxes if no prop is available.
    """
    lib = world.get_blueprint_library()
    bp = None
    for name in _BUILDING_PROPS:
        found = lib.filter(name)
        if found:
            bp = found[0]
            break
    if bp is None:
        print("No building prop available; drawing wireframe boxes instead.")
        return _draw_buildings_wire(world, buildings, max_n)

    # measure the prop's real size from a throwaway spawn
    probe = world.try_spawn_actor(bp, carla.Transform(carla.Location(0, 0, 300)))
    if probe is None:
        print("Could not probe building prop; using wireframe boxes.")
        return _draw_buildings_wire(world, buildings, max_n)
    ext = probe.bounding_box.extent
    pw, pl, ph = max(ext.x * 2, 0.5), max(ext.y * 2, 0.5), max(ext.z * 2, 0.5)
    probe.destroy()

    items = buildings[:max_n] if max_n else buildings
    spawned = 0
    for b in items:
        cx, cy, cyaw = carla_from_opendrive(b["x"], b["y"], math.radians(b["yaw"]))
        gz = _ground_z(world, cx, cy, default=0.0)
        nx = max(1, int(round(b["sx"] / pw)))
        ny = max(1, int(round(b["sy"] / pl)))
        nz = max(1, min(layers, int(b["height"] // ph)))
        x0 = cx - (nx - 1) * pw / 2.0
        y0 = cy - (ny - 1) * pl / 2.0
        for iz in range(nz):
            for ix in range(nx):
                for iy in range(ny):
                    tf = carla.Transform(
                        carla.Location(x0 + ix * pw, y0 + iy * pl,
                                       gz + ph / 2.0 + iz * ph),
                        carla.Rotation(yaw=cyaw))
                    if world.try_spawn_actor(bp, tf) is not None:
                        spawned += 1
    print("Spawned %d '%s' props for %d building blocks."
          % (spawned, bp.id, len(items)))


def _draw_buildings_wire(world, buildings, max_n):
    """Fallback: persistent wireframe boxes conveying building volumes."""
    items = buildings[:max_n] if max_n else buildings
    color = carla.Color(120, 120, 130)
    for b in items:
        cx, cy, cyaw = carla_from_opendrive(b["x"], b["y"], math.radians(b["yaw"]))
        gz = _ground_z(world, cx, cy, default=0.0)
        h = b["height"]
        bb = carla.BoundingBox(
            carla.Location(cx, cy, gz + h / 2.0),
            carla.Vector3D(b["sx"] / 2.0, b["sy"] / 2.0, h / 2.0))
        world.debug.draw_box(bb, carla.Rotation(yaw=cyaw), thickness=0.25,
                             color=color, life_time=0.0)
    print("Drew %d wireframe building blocks." % len(items))


def _draw_crosswalks(world, polys):
    """Draw zebra stripes for each crosswalk polygon (persistent debug lines)."""
    white = carla.Color(235, 235, 235)
    bar, gap = 0.5, 0.6
    n = 0
    for poly in polys:
        pts = [(x, -y) for x, y in poly]   # OpenDRIVE -> CARLA (flip Y)
        if len(pts) < 4:
            continue
        c0 = pts[0]
        ux, uy = pts[1][0] - c0[0], pts[1][1] - c0[1]
        vx, vy = pts[-1][0] - c0[0], pts[-1][1] - c0[1]
        lu = math.hypot(ux, uy)
        lv = math.hypot(vx, vy)
        if lu < 0.3 or lv < 0.3:
            continue
        uhx, uhy = ux / lu, uy / lu
        vhx, vhy = vx / lv, vy / lv
        gz = _ground_z(world, c0[0], c0[1], default=0.0) + 0.06
        k = 0.0
        while k < lv:
            off = k + bar / 2.0
            bx, by = c0[0] + vhx * off, c0[1] + vhy * off
            world.debug.draw_line(
                carla.Location(bx, by, gz),
                carla.Location(bx + uhx * lu, by + uhy * lu, gz),
                thickness=bar / 2.0, color=white, life_time=0.0)
            k += bar + gap
        n += 1
    print("Drew %d crosswalks (zebra stripes)." % n)


class _AgentReplayer:
    """Spawns the recorded nuScenes agents and moves them by wall-clock time.

    Shared by manual mode so the surrounding traffic keeps replaying while you
    drive. Agents are physics-off and teleported (interpolated) each update.
    """

    def __init__(self, world, meta, args):
        self.world = world
        self.frames = [] if args.no_agents else meta.get("agent_frames", [])
        self.fdt = meta.get("frame_dt", 0.5)
        self.actors, self.kinds, self.zoff, self.gz_cache = {}, {}, {}, {}
        self.hidden = carla.Location(0.0, 0.0, -200.0)
        self.allowed = None
        if args.max_agents and self.frames:
            seen, order = set(), []
            for fr in self.frames:
                for a in fr:
                    if a["id"] not in seen:
                        seen.add(a["id"]); order.append(a["id"])
            self.allowed = set(order[:args.max_agents])

    def _gz(self, x, y):
        key = (round(x), round(y))
        v = self.gz_cache.get(key)
        if v is None:
            v = _ground_z(self.world, x, y, 0.0)
            self.gz_cache[key] = v
        return v

    def _place(self, actor, x, y, yaw, zo):
        actor.set_transform(carla.Transform(
            carla.Location(x, y, self._gz(x, y) + zo), carla.Rotation(yaw=yaw)))

    def _ensure(self, a):
        iid = a["id"]
        if iid in self.actors or (self.allowed is not None and iid not in self.allowed):
            return
        kind = self.kinds.setdefault(iid, _category_kind(a["category"]))
        ax, ay, ayaw = carla_from_opendrive(a["x"], a["y"], a["yaw"])
        bp = _pick_blueprint(self.world, kind, sum(bytearray(iid.encode())))
        tf = carla.Transform(carla.Location(ax, ay, self._gz(ax, ay) + 1.0),
                             carla.Rotation(yaw=ayaw))
        act = self.world.try_spawn_actor(bp, tf)
        if act is None:
            tf.location.z = self._gz(ax, ay) + 3.0
            act = self.world.try_spawn_actor(bp, tf)
        if act is None:
            return
        try:
            act.set_simulate_physics(False)
        except Exception:
            pass
        self.zoff[iid] = _z_offset(act)
        self.actors[iid] = act

    def update(self, sim_t, loop=True):
        """Place all agents for wall-clock time sim_t; returns keyframe index."""
        nfr = len(self.frames)
        if nfr == 0:
            return 0
        if nfr == 1:
            i, r = 0, 0.0
        else:
            total = (nfr - 1) * self.fdt
            tt = (sim_t % total) if (loop and total > 0) else min(sim_t, total)
            fpos = tt / self.fdt
            i = min(int(fpos), nfr - 2)
            r = fpos - i
        a0 = {a["id"]: a for a in self.frames[i]}
        a1 = {a["id"]: a for a in self.frames[min(i + 1, nfr - 1)]}
        for a in a0.values():
            self._ensure(a)
        present = set(a0)
        for iid, a in a0.items():
            act = self.actors.get(iid)
            if act is None:
                continue
            b = a1.get(iid, a)
            gx, gy, gyaw = carla_from_opendrive(
                _lerp(a["x"], b["x"], r), _lerp(a["y"], b["y"], r),
                _lerp_angle(a["yaw"], b["yaw"], r))
            self._place(act, gx, gy, gyaw, self.zoff.get(iid, 0.0))
        for iid, act in self.actors.items():
            if iid not in present:
                act.set_transform(carla.Transform(self.hidden))
        return i

    def destroy(self, client):
        client.apply_batch([carla.command.DestroyActor(a)
                            for a in self.actors.values()])


class _TrajDrawer:
    """Draws colored history/future trajectories for the ego + agents.

    Reusable across replay/follow: call draw(i) with the current keyframe index.
    """

    def __init__(self, world, meta, args):
        self.world = world
        self.args = args
        self.traj = meta["ego_trajectory_local"]
        frames = [] if args.no_agents else meta.get("agent_frames", [])
        self.agent_xy, self.agent_group = {}, {}
        for fi, fr in enumerate(frames):
            for a in fr:
                self.agent_xy.setdefault(a["id"], {})[fi] = (a["x"], a["y"])
                self.agent_group.setdefault(
                    a["id"], _traj_group(_category_kind(a["category"])))
        fdt = meta.get("frame_dt", 0.5)
        self.step = max(1, int(round((1.0 / args.traj_hz) / fdt)))
        self.hn = int(round(args.hist_seconds * args.traj_hz))
        self.fn = int(round(args.future_seconds * args.traj_hz))
        self.gz_cache = {}

    def _gz(self, x, y):
        key = (round(x), round(y))
        v = self.gz_cache.get(key)
        if v is None:
            v = _ground_z(self.world, x, y, 0.0)
            self.gz_cache[key] = v
        return v

    def _poly(self, pos, indices, group, is_future, life):
        avail = []
        for k in indices:
            p = pos(k)
            if p is None:
                continue
            cx, cy = p[0], -p[1]
            avail.append((k, carla.Location(cx, cy, self._gz(cx, cy) + 0.25)))
        col = _traj_color(group, is_future)
        for (k1, l1), (k2, l2) in zip(avail, avail[1:]):
            if k2 - k1 == self.step:
                self.world.debug.draw_line(l1, l2, thickness=0.04, color=col,
                                           life_time=life)
        for _, loc in avail:
            self.world.debug.draw_point(loc, size=0.08, color=col, life_time=life)

    def draw(self, i, life=0.3):
        if self.args.no_traj:
            return
        n = len(self.traj)
        hist = list(range(i - self.hn * self.step, i + 1, self.step))
        fut = list(range(i, i + self.fn * self.step + 1, self.step))

        def ego_pos(k):
            return (self.traj[k]["x"], self.traj[k]["y"]) if 0 <= k < n else None
        self._poly(ego_pos, hist, "ego", False, life)
        self._poly(ego_pos, fut, "ego", True, life)
        for iid, xy in self.agent_xy.items():
            if i not in xy:                 # only agents present at this step
                continue
            grp = self.agent_group[iid]
            self._poly(xy.get, hist, grp, False, life)
            self._poly(xy.get, fut, grp, True, life)


def _segment_progress(ego_loc, a, b):
    """Fraction (0..1) of the ego's projection along segment a->b."""
    abx, aby = b.x - a.x, b.y - a.y
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-6:
        return 0.0
    t = ((ego_loc.x - a.x) * abx + (ego_loc.y - a.y) * aby) / ab2
    return max(0.0, min(1.0, t))


def _draw_goal(world, loc, life):
    """Highlight the ego's current planned destination waypoint."""
    red = carla.Color(255, 40, 40)
    top = carla.Location(loc.x, loc.y, loc.z + 2.2)
    world.debug.draw_line(carla.Location(loc.x, loc.y, loc.z + 0.1), top,
                          thickness=0.03, color=red, life_time=life)
    world.debug.draw_point(carla.Location(loc.x, loc.y, loc.z + 0.3),
                           size=0.10, color=red, life_time=life)
    world.debug.draw_string(carla.Location(loc.x, loc.y, loc.z + 2.4), "GOAL",
                            draw_shadow=False, color=red, life_time=life)


class _Target:
    """Minimal waypoint-like object (has .transform) for VehiclePIDController."""
    __slots__ = ("transform",)

    def __init__(self, loc, yaw):
        self.transform = carla.Transform(loc, carla.Rotation(yaw=yaw))


def _follow_trajectory(client, world, ego, meta, args):
    """Physically drive the recorded ego path with CARLA's PID controller.

    Planning = the recorded nuScenes ego trajectory (used as the route);
    control  = carla agents' VehiclePIDController (throttle/brake/steer) with
    real vehicle physics. Surrounding recorded agents keep replaying.
    """
    import sys
    import time
    if args.carla_agents and args.carla_agents not in sys.path:
        sys.path.append(args.carla_agents)
    from agents.navigation.controller import VehiclePIDController

    traj = meta["ego_trajectory_local"]
    fdt = meta.get("frame_dt", 0.5)
    cmap = world.get_map()

    # build the route (exact recorded points, grounded) + per-point target speed
    route, speeds, prev = [], [], None
    for p in traj:
        cx, cy, cyaw = carla_from_opendrive(p["x"], p["y"], p["yaw"])
        wp = cmap.get_waypoint(carla.Location(cx, cy, 0.0), project_to_road=True)
        z = wp.transform.location.z if wp else 0.0
        loc = carla.Location(cx, cy, z)
        route.append((loc, cyaw))
        speeds.append(0.0 if prev is None
                      else min(60.0, loc.distance(prev) / fdt * 3.6))  # km/h
        prev = loc
    if speeds:
        speeds[0] = speeds[1] if len(speeds) > 1 else 10.0
    if len(route) < 2:
        print("Trajectory too short to follow."); return

    dt = 0.05
    controller = VehiclePIDController(
        ego,
        args_lateral={"K_P": 1.0, "K_I": 0.1, "K_D": 0.0, "dt": dt},
        args_longitudinal={"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": dt})

    # synchronous, fixed-step control loop
    orig_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    world.apply_settings(settings)

    ego.set_simulate_physics(True)
    start_loc, start_yaw = route[0]
    ego.set_transform(carla.Transform(
        carla.Location(start_loc.x, start_loc.y, start_loc.z + 0.3),
        carla.Rotation(yaw=start_yaw)))

    replayer = _AgentReplayer(world, meta, args)
    drawer = _TrajDrawer(world, meta, args)
    budget = int(6.0 / dt)          # max ticks to reach each target (skip if stuck)
    reach = 3.0                     # metres: consider a target reached

    # camera rig -> saved to a SEPARATE folder so it doesn't clash with replay
    scene_name = meta.get("scene", "scene")
    default_cam = os.path.join(DEFAULT_OUTDIR, "cameras")
    cam_root = (os.path.join(DEFAULT_OUTDIR, "cameras_follow")
                if os.path.normpath(args.cam_dir) == os.path.normpath(default_cam)
                else args.cam_dir)
    rig = (_CameraRig(world, ego, args, _load_calib_cams(meta, args))
           if args.cameras else None)
    if rig is not None:
        print("Saving follow-mode cameras to", os.path.join(cam_root, scene_name))
    lidar = _build_lidar(world, ego, args) if args.lidar else None
    lidar_root = os.path.join(args.lidar_dir + "_follow"
                              if os.path.normpath(args.lidar_dir)
                              == os.path.normpath(os.path.join(DEFAULT_OUTDIR,
                                                               "lidar_carla"))
                              else args.lidar_dir, scene_name)

    def save_rig(kf_idx):
        if rig is not None:
            rig.drain(save_dir=os.path.join(cam_root, scene_name),
                      kf_idx=kf_idx)
        if lidar is not None:
            lidar[1].get().save_to_disk(
                os.path.join(lidar_root, "%03d.ply" % kf_idx))

    def drain_rig():
        if rig is not None:
            rig.drain()
        if lidar is not None:
            lidar[1].get()

    print("Following recorded path with PID control (%d points). Ctrl+C to stop."
          % len(route))
    try:
        for idx in range(1, len(route)):
            target_loc, target_yaw = route[idx]
            tgt = _Target(target_loc, target_yaw)
            used = 0
            while True:                       # at least one tick per keyframe
                control = controller.run_step(speeds[idx], tgt)
                ego.apply_control(control)
                # keep agents + trajectories in step with the ego's progress
                r = _segment_progress(ego.get_location(), route[idx - 1][0],
                                      target_loc)
                kf = (idx - 1) + r
                replayer.update(kf * fdt, loop=False)
                if used % 4 == 0:                       # redraw overlay ~5 Hz
                    drawer.draw(int(round(kf)), life=0.3)
                _draw_goal(world, target_loc, life=dt * 3.0)   # current destination
                _chase_cam(world, ego)
                if rig is not None:              # calibrated rig follows the ego
                    tf = ego.get_transform()
                    rig.place(tf.location.x, tf.location.y, tf.rotation.yaw,
                              _ground_z(world, tf.location.x, tf.location.y, 0.0))
                world.tick()
                time.sleep(dt)
                used += 1
                if rig is not None or lidar is not None:   # save at keyframe idx-1
                    (save_rig(idx - 1) if used == 1 else drain_rig())
                if (ego.get_location().distance(target_loc) <= reach
                        or used >= budget):
                    break
            if idx % 5 == 0:
                v = ego.get_velocity()
                spd = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
                print("  point %d/%d  target %.0f km/h  actual %.0f km/h"
                      % (idx, len(route) - 1, speeds[idx], spd))
        ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        world.tick()
        if rig is not None or lidar is not None:
            save_rig(len(route) - 1)            # final keyframe
        print("Reached end of recorded path.")
    finally:
        world.apply_settings(orig_settings)     # restore async
        if rig is not None:
            rig.destroy()
        if lidar is not None:
            try:
                lidar[0].stop(); lidar[0].destroy()
            except Exception:
                pass
        try:
            replayer.destroy(client)
        except Exception:
            pass


def _manual_drive(client, world, ego, meta, args):
    """Drive the ego yourself in a pygame window (keyboard, optional gamepad).

    Keyboard: W/Up throttle, S/Down brake, A/D or Left/Right steer,
    Space handbrake, Q toggle reverse, P autopilot, ESC quit. The recorded
    nuScenes agents keep replaying around you (unless --no-agents).
    """
    import numpy as np
    import pygame

    W, H = 1280, 720
    ego.set_simulate_physics(True)

    # a chase camera rendered into the pygame window
    lib = world.get_blueprint_library()
    bp = lib.find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(W))
    bp.set_attribute("image_size_y", str(H))
    bp.set_attribute("fov", "90")
    cam = world.spawn_actor(
        bp, carla.Transform(carla.Location(x=-6.0, z=3.0),
                            carla.Rotation(pitch=-12.0)), attach_to=ego)
    frame = {"surface": None}

    def on_img(image):
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        frame["surface"] = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
    cam.listen(on_img)

    pygame.init()
    display = pygame.display.set_mode((W, H))
    pygame.display.set_caption("CARLA manual drive - %s" % meta_scene(args))
    font = pygame.font.SysFont("consolas", 18)
    clock = pygame.time.Clock()

    js = None
    if args.joystick and pygame.joystick.get_count() > 0:
        js = pygame.joystick.Joystick(0)
        js.init()
        print("Using joystick:", js.get_name())

    steer_cache = 0.0
    reverse = False
    autopilot = False
    replayer = _AgentReplayer(world, meta, args)
    sim_t = 0.0
    if replayer.frames:
        print("Replaying %d recorded agents around you."
              % len({a["id"] for fr in replayer.frames for a in fr}))
    print("Manual drive: W/S throttle-brake, A/D steer, Space handbrake, "
          "Q reverse, P autopilot, ESC quit.")
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                    if event.key == pygame.K_q:
                        reverse = not reverse
                    if event.key == pygame.K_p:
                        autopilot = not autopilot
                        ego.set_autopilot(autopilot, args.tm_port)

            control = carla.VehicleControl()
            if not autopilot:
                keys = pygame.key.get_pressed()
                throttle = 0.75 if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
                brake = 0.9 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0
                if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                    steer_cache -= 0.04
                elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                    steer_cache += 0.04
                else:
                    steer_cache *= 0.6
                if js is not None:              # gamepad overrides when moved
                    try:
                        ax = js.get_axis(0)
                        if abs(ax) > 0.05:
                            steer_cache = ax
                        rt = (js.get_axis(5) + 1.0) / 2.0
                        lt = (js.get_axis(4) + 1.0) / 2.0
                        if rt > 0.02:
                            throttle = rt
                        if lt > 0.02:
                            brake = lt
                    except Exception:
                        pass
                steer_cache = max(-1.0, min(1.0, steer_cache))
                control.throttle = throttle
                control.brake = brake
                control.steer = round(steer_cache, 3)
                control.hand_brake = bool(pygame.key.get_pressed()[pygame.K_SPACE])
                control.reverse = reverse
                ego.apply_control(control)

            kf = replayer.update(sim_t, loop=args.loop)
            _chase_cam(world, ego)
            world.wait_for_tick()

            if frame["surface"] is not None:
                display.blit(frame["surface"], (0, 0))
            v = ego.get_velocity()
            speed = 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            lines = [
                "AUTOPILOT" if autopilot else "MANUAL",
                "speed  %5.1f km/h" % speed,
                "gear   %s" % ("R" if reverse else "D"),
                "steer  %+.2f" % steer_cache,
                "agents %d  frame %d" % (len(replayer.actors), kf),
                "W/S throttle-brake  A/D steer  Space hbrake",
                "Q reverse   P autopilot   ESC quit",
            ]
            for li, txt in enumerate(lines):
                display.blit(font.render(txt, True, (255, 255, 0)),
                             (12, 12 + 22 * li))
            pygame.display.flip()
            dt_ms = clock.tick(30)
            sim_t += (dt_ms / 1000.0) * args.replay_speed
    finally:
        try:
            cam.stop(); cam.destroy()
        except Exception:
            pass
        try:
            replayer.destroy(client)
        except Exception:
            pass
        pygame.quit()
        try:
            ego.set_autopilot(False)
        except Exception:
            pass
        print("Manual drive ended.")


def meta_scene(args):
    return os.path.basename(args.xodr) if args.xodr else "scenario %d" % args.scenario_id


def _autopilot_drive(client, world, ego, tm_port, speed_diff):
    """Hand the ego to the Traffic Manager and follow it with the camera."""
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(False)
    ego.set_autopilot(True, tm_port)
    # drive a bit below the limit and don't get stuck at lights
    tm.vehicle_percentage_speed_difference(ego, speed_diff)
    try:
        tm.ignore_lights_percentage(ego, 100.0)
        tm.ignore_signs_percentage(ego, 100.0)
    except Exception:
        pass
    print("Autopilot ON. Watch the CARLA window. Ctrl+C here to stop.")
    while True:
        world.wait_for_tick()
        _chase_cam(world, ego)


# nuScenes category -> ordered CARLA blueprint id candidates (first available wins)
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


def _category_kind(cat):
    if cat.startswith("human.pedestrian"):
        return "pedestrian"
    for k in ("bicycle", "motorcycle", "bus", "truck"):
        if k in cat:
            return k
    if cat.startswith("vehicle."):
        return "car"          # car / construction / trailer / emergency -> car
    return "car"


# distinct colours for surrounding vehicles (never red -> red is the ego)
_AGENT_COLORS = ["0,90,255", "0,160,255", "0,200,120", "220,180,0",
                 "150,0,255", "255,140,0", "210,210,210", "40,40,40"]


def _pick_blueprint(world, kind, key):
    """Deterministically pick a blueprint for a kind, given a stable key."""
    lib = world.get_blueprint_library()
    for pattern in _BP_CANDIDATES.get(kind, _BP_CANDIDATES["car"]):
        bps = lib.filter(pattern)
        if bps:
            bp = bps[key % len(bps)]
            if bp.has_attribute("role_name"):
                bp.set_attribute("role_name", "nuscenes_agent")
            if bp.has_attribute("color"):       # non-red so ego stays distinct
                bp.set_attribute("color", _AGENT_COLORS[key % len(_AGENT_COLORS)])
            return bp
    return lib.filter("vehicle.*")[0]


def _ground_z(world, x, y, default=0.3):
    wp = world.get_map().get_waypoint(
        carla.Location(x, y, 0.0), project_to_road=True)
    return wp.transform.location.z if wp else default


def _lerp(a, b, r):
    return a + (b - a) * r


def _lerp_angle(a, b, r):
    """Interpolate yaw (radians) along the shortest arc."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + d * r


def _catmull(p0, p1, p2, p3, t):
    """Centripetal-ish Catmull-Rom: smooth (C1) position through p1..p2."""
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ((2 * p1) + (-p0 + p2) * t
                  + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * t3)


# trajectory colours per category; history is drawn dimmer than the future
_TRAJ_BASE = {
    "ego":        (255, 40, 40),    # red
    "vehicle":    (0, 160, 255),    # blue
    "pedestrian": (0, 220, 60),     # green
    "cyclist":    (255, 60, 200),   # magenta
}


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


def _z_offset(actor):
    """z to add to the actor's origin so its bounding-box bottom rests on z=0."""
    try:
        bb = actor.bounding_box
        return bb.extent.z - bb.location.z
    except Exception:
        return 0.0


# nuScenes-style 6-camera rig, expressed in CARLA ego-relative coordinates
# (x forward, +y right, z up, yaw degrees clockwise). Approximates the nuScenes
# mounting: front + two front side cams (~55 deg) + wide back + two rear side
# cams (~110 deg).
NUSCENES_CAMS = [
    # name,            x,     y,    z,    yaw,    fov
    ("CAM_FRONT",       1.70,  0.00, 1.51,   0.0,  70),
    ("CAM_FRONT_LEFT",  1.55, -0.50, 1.51, -55.0,  70),
    ("CAM_FRONT_RIGHT", 1.55,  0.50, 1.51,  55.0,  70),
    ("CAM_BACK",       -1.50,  0.00, 1.56, 180.0, 110),
    ("CAM_BACK_LEFT",   0.00, -0.55, 1.56,-110.0,  70),
    ("CAM_BACK_RIGHT",  0.00,  0.55, 1.56, 110.0,  70),
]


# nuScenes LIDAR_TOP is a Velodyne HDL-32E mounted at the roof centre. This is
# the same sensor whose recorded sweeps plot_lidar.py renders, so we match its
# optics here: 32 beams, +10/-30 deg vertical FOV, 360 deg horizontal, ~70 m.
# Placement is ego-relative CARLA (x forward, +y right, z up), approximating the
# nuScenes calibrated_sensor extrinsic (~[0.94, 0, 1.84]).
NUSCENES_LIDAR = dict(x=0.90, y=0.00, z=1.84, upper_fov=10.0, lower_fov=-30.0)


def _build_lidar(world, ego, args):
    """Spawn a nuScenes-style HDL-32E LiDAR on the ego; return (sensor, queue).

    In synchronous mode the rotation frequency is pinned to the tick rate so each
    world.tick() yields exactly one full 360 sweep, and points_per_second is
    scaled by that rate so every sweep carries ~args.lidar_points points (nuScenes
    LIDAR_TOP is ~34k points/sweep).
    """
    try:
        from queue import Queue
    except ImportError:
        from Queue import Queue          # py2 fallback (unused on 3.7)
    lib = world.get_blueprint_library()
    bp = lib.find("sensor.lidar.ray_cast")
    delta = world.get_settings().fixed_delta_seconds or 0.05
    rot_hz = 1.0 / delta                 # one full sweep per tick (sync mode)
    L = NUSCENES_LIDAR
    bp.set_attribute("channels", str(args.lidar_channels))
    bp.set_attribute("range", str(args.lidar_range))
    bp.set_attribute("upper_fov", str(L["upper_fov"]))
    bp.set_attribute("lower_fov", str(L["lower_fov"]))
    bp.set_attribute("rotation_frequency", str(rot_hz))
    bp.set_attribute("points_per_second", str(int(args.lidar_points * rot_hz)))
    for opt, val in (("dropoff_general_rate", "0.0"),
                     ("dropoff_intensity_limit", "0.0"),
                     ("dropoff_zero_intensity", "0.0"),
                     ("atmosphere_attenuation_rate", "0.004")):
        if bp.has_attribute(opt):
            bp.set_attribute(opt, val)
    sensor = world.spawn_actor(
        bp, carla.Transform(carla.Location(L["x"], L["y"], L["z"])),
        attach_to=ego)
    q = Queue()
    sensor.listen(q.put)
    print("Attached HDL-32E LiDAR (%d ch, %.0f m, ~%d pts/sweep). Saving to %s"
          % (args.lidar_channels, args.lidar_range, args.lidar_points,
             args.lidar_dir))
    return sensor, q


def _load_calib_cams(meta, args):
    """Camera specs from the phase0 calib sidecar, or None if unavailable.

    The sidecar (experiments/phase0_calib_gt) carries, per camera, the real
    nuScenes intrinsics/extrinsics precomputed for CARLA: the left-handed
    placement matrix M, translation t_lh, and the blueprint config
    (native width/height + FOV = 2*atan(W/2fx)).
    """
    if args.approx_rig:
        return None
    scene, map_name = meta.get("scene"), meta.get("map")
    if not scene or not map_name:
        return None
    path = os.path.join(args.calib_dir, "%s_%s_calib_gt.json" % (map_name, scene))
    if not os.path.exists(path):
        print("No calib sidecar %s -> using the approximate rig "
              "(run experiments/phase0_calib_gt/export_calib_gt.py for a "
              "calibrated one)." % path)
        return None
    with open(path, "r") as f:
        sensors = json.load(f)["sensors"]
    return {ch: spec for ch, spec in sensors.items() if spec["kind"] == "cam"}


class _CameraRig(object):
    """nuScenes 6-camera rig, calibrated when possible.

    Calibrated mode (default when the phase0 sidecar exists): sensors are
    FREE-FLOATING (not attached -- CARLA's vehicle pivot is not the nuScenes
    ego origin) and teleported each tick to ego_pose composed with the real
    extrinsic, rendering at the native 1600x900 with per-camera FOV from the
    real fx. Saved views then share the real cameras' projection, so they are
    directly usable for calibration-sensitive evaluation (paper Sec. 3.4).

    Approximate mode (--approx-rig, or no sidecar): the legacy attached rig
    from NUSCENES_CAMS at --cam-width x --cam-height.
    """

    def __init__(self, world, ego, args, calib_cams=None):
        try:
            from queue import Queue
        except ImportError:
            from Queue import Queue      # py2 fallback (unused on 3.7)
        lib = world.get_blueprint_library()
        self.items = []                  # dicts: name, sensor, q [, M, t]
        self.calibrated = calib_cams is not None
        if self.calibrated:
            park = carla.Transform(carla.Location(0.0, 0.0, 60.0))
            for name in sorted(calib_cams):
                spec = calib_cams[name]
                bp = lib.find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(spec["carla"]["width"]))
                bp.set_attribute("image_size_y", str(spec["carla"]["height"]))
                bp.set_attribute("fov", str(spec["carla"]["fov"]))
                cam = world.spawn_actor(bp, park)
                q = Queue()
                cam.listen(q.put)
                self.items.append({"name": name, "sensor": cam, "q": q,
                                   "M": np.array(spec["M"]),
                                   "t": np.array(spec["t_lh"])})
            print("Attached CALIBRATED rig: %d cams, native res, real "
                  "K/extrinsics. Saving to %s" % (len(self.items), args.cam_dir))
        else:
            for name, x, y, z, yaw, fov in NUSCENES_CAMS:
                bp = lib.find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(args.cam_width))
                bp.set_attribute("image_size_y", str(args.cam_height))
                bp.set_attribute("fov", str(fov))
                cam = world.spawn_actor(
                    bp, carla.Transform(carla.Location(x, y, z),
                                        carla.Rotation(yaw=yaw)),
                    attach_to=ego)
                q = Queue()
                cam.listen(q.put)
                self.items.append({"name": name, "sensor": cam, "q": q})
            print("Attached approximate rig: %d cams (%dx%d). Saving to %s"
                  % (len(self.items), args.cam_width, args.cam_height,
                     args.cam_dir))

    def place(self, cx, cy, cyaw_deg, gz):
        """Teleport calibrated sensors to ego pose ∘ extrinsic (no-op when
        attached). (cx, cy, cyaw): CARLA ego pose; gz: local ground height --
        the nuScenes ego frame origin sits on the ground."""
        if not self.calibrated:
            return
        Rz = _carla_rotation_matrix(0.0, 0.0, cyaw_deg)
        base = np.array([cx, cy, gz])
        for it in self.items:
            R = Rz.dot(it["M"])
            t = Rz.dot(it["t"]) + base
            roll, pitch, yaw = _carla_euler_from_matrix(R)
            it["sensor"].set_transform(carla.Transform(
                carla.Location(float(t[0]), float(t[1]), float(t[2])),
                carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)))

    def drain(self, save_dir=None, kf_idx=None):
        """Pop one frame per camera for this tick; save when a target given."""
        for it in self.items:
            img = it["q"].get()
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


def _replay_trajectory(client, world, ego, meta, args):
    """Replay ego + recorded nuScenes agents smoothly, in synchronous mode.

    Synchronous stepping makes set_transform deterministic (no flicker), the
    camera is driven from the intended pose (not a stale read-back), keyframes
    are interpolated for smooth motion, and every actor is grounded by its own
    bounding box so pedestrians don't sink and vehicles don't float.
    """
    import time
    traj = meta["ego_trajectory_local"]
    frames = [] if args.no_agents else meta.get("agent_frames", [])
    dt = meta.get("frame_dt", 0.5) / max(args.replay_speed, 1e-3)
    fdt = meta.get("frame_dt", 0.5)
    substeps = max(1, args.substeps)
    sub_dt = dt / substeps
    n = len(traj)

    # cap which instances we may spawn (by first appearance order)
    allowed = None
    if args.max_agents and frames:
        seen, order = set(), []
        for fr in frames:
            for a in fr:
                if a["id"] not in seen:
                    seen.add(a["id"])
                    order.append(a["id"])
        allowed = set(order[:args.max_agents])

    # per-instance trajectory (frame_idx -> x,y) + category group, for the overlay
    agent_xy, agent_group = {}, {}
    for fi, fr in enumerate(frames):
        for a in fr:
            agent_xy.setdefault(a["id"], {})[fi] = (a["x"], a["y"])
            agent_group.setdefault(a["id"],
                                   _traj_group(_category_kind(a["category"])))

    # synchronous mode -> deterministic, flicker-free stepping
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

    scene_name = meta.get("scene", "scene")
    rig = (_CameraRig(world, ego, args, _load_calib_cams(meta, args))
           if args.cameras else None)
    lidar = _build_lidar(world, ego, args) if args.lidar else None

    # cache ground height on a 1 m grid (waypoint queries are the main per-frame
    # cost; caching keeps frame timing even, which removes timing-induced jitter)
    gz_cache = {}

    def gz(x, y):
        key = (round(x), round(y))
        v = gz_cache.get(key)
        if v is None:
            v = _ground_z(world, x, y, 0.0)
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
        bp = _pick_blueprint(world, kind, sum(bytearray(iid.encode())))
        tf = carla.Transform(carla.Location(ax, ay, gz(ax, ay) + 1.0),
                             carla.Rotation(yaw=ayaw))
        act = world.try_spawn_actor(bp, tf)
        if act is None:                     # retry higher (spawn collision)
            tf.location.z = gz(ax, ay) + 3.0
            act = world.try_spawn_actor(bp, tf)
        if act is None:
            return
        try:
            act.set_simulate_physics(False)
        except Exception:
            pass
        zoff[iid] = _z_offset(act)
        actors[iid] = act

    # trajectory overlay parameters (drawn at args.traj_hz over the keyframes)
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
            cx, cy = p[0], -p[1]            # OpenDRIVE -> CARLA (flip Y)
            avail.append((k, carla.Location(cx, cy, gz(cx, cy) + 0.25)))
        col = _traj_color(group, is_future)
        for (k1, l1), (k2, l2) in zip(avail, avail[1:]):
            if k2 - k1 == step:
                world.debug.draw_line(l1, l2, thickness=0.04, color=col,
                                      life_time=traj_life)
        for _, loc in avail:               # dot at each sampled (2 Hz) point
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
        cam_yaw = None
        back, up = 8.0, 4.0
        for i in range(n):
            # smooth ego path with Catmull-Rom over neighbouring keyframes
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
                cx, cy, cyaw = carla_from_opendrive(ex, ey, eyaw)
                place(ego, cx, cy, cyaw, ego_zoff)
                if rig is not None:      # calibrated rig follows the ego pose
                    rig.place(cx, cy, cyaw, gz(cx, cy))
                for iid, a in a0.items():
                    act = actors.get(iid)
                    if act is None:
                        continue
                    b = a1.get(iid, a)      # hold position if it leaves next frame
                    gx, gy, gyaw = carla_from_opendrive(
                        _lerp(a["x"], b["x"], r), _lerp(a["y"], b["y"], r),
                        _lerp_angle(a["yaw"], b["yaw"], r))
                    place(act, gx, gy, gyaw, zoff.get(iid, 0.0))
                for iid, act in actors.items():
                    if iid not in present:
                        act.set_transform(carla.Transform(hidden))
                # smoothed chase camera (EMA on yaw removes swing when stopped)
                if cam_yaw is None:
                    cam_yaw = cyaw
                else:
                    cam_yaw += ((cyaw - cam_yaw + 180.0) % 360.0 - 180.0) * 0.2
                yawr = math.radians(cam_yaw)
                spectator.set_transform(carla.Transform(
                    carla.Location(cx - back * math.cos(yawr),
                                   cy - back * math.sin(yawr), gz(cx, cy) + up),
                    carla.Rotation(pitch=-15.0, yaw=cam_yaw)))
                world.tick()
                # drain camera queues every tick (keep them in sync); save the
                # 6 views once per keyframe (at the keyframe pose, substep 0)
                if rig is not None:
                    if s == 0:
                        rig.drain(save_dir=os.path.join(args.cam_dir, scene_name),
                                  kf_idx=i)
                    else:
                        rig.drain()
                # one full LiDAR sweep per tick; save the keyframe pose (substep 0)
                if lidar is not None:
                    _, lq = lidar
                    scan = lq.get()
                    if s == 0:
                        scan.save_to_disk(os.path.join(
                            args.lidar_dir, scene_name, "%03d.ply" % i))
                time.sleep(sub_dt)

    try:
        run_once()
        while args.loop:
            run_once()
    finally:
        print("Cleaning up %d replay agents..." % len(actors))
        if rig is not None:
            rig.destroy()
        if lidar is not None:
            try:
                lidar[0].stop()
                lidar[0].destroy()
            except Exception:
                pass
        client.apply_batch([carla.command.DestroyActor(a)
                            for a in actors.values()])
        world.apply_settings(orig_settings)   # restore async mode
        try:
            ego.set_simulate_physics(True)
        except Exception:
            pass
    print("Replay finished.")


def _chase_cam(world, ego):
    """Place the spectator behind and above the ego (third-person chase)."""
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    back, up = 8.0, 4.0
    spec = world.get_spectator()
    spec.set_transform(carla.Transform(
        carla.Location(tf.location.x - back * math.cos(yaw),
                       tf.location.y - back * math.sin(yaw),
                       tf.location.z + up),
        carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)))


if __name__ == "__main__":
    main()
