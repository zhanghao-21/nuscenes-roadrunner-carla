"""
CARLA <-> SUMO co-simulation (MAINLINE ONLY) with:
- CARLA ego spawn at chosen spawn index / transform (optionally delayed)
- MAIN vehicles spawn at t=0 (some fixed-route, some random-route)
- Non-fixed MAIN vehicles are rerouted at route ends
- MARTS multi-modal prediction (NEIGHBORS ONLY) + CARLA visualization
- Prediction-based inverse TTC dashboard (distribution over K modes)
- Ego is NOT predicted; only ego PLANNED trajectory is drawn and used for TTC
"""

import argparse
import glob
import json
import logging
import os
import random
import re
import shutil
import sys
import tempfile
import time
from typing import List, Optional, Set, Dict, Tuple

import numpy as np
import lxml.etree as ET

from pyqtgraph.Qt import QtWidgets
import pyqtgraph as pg

import torch
from HGT_model.utils import setup_seed, load_config
from HGT_model.models.mart_s import MARTS

# --- find carla egg ---
try:
    sys.path.append(
        glob.glob(
            '../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg'
            % (sys.version_info.major, sys.version_info.minor,
               'win-amd64' if os.name == 'nt' else 'linux-x86_64')
        )[0]
    )
except IndexError:
    pass

# --- find SUMO tools ---
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import carla  # noqa: E402
import sumolib  # noqa: E402
import traci  # noqa: E402
# ADD THIS LINE:
from agents.navigation.global_route_planner import GlobalRoutePlanner
from sumo_integration.carla_simulation import CarlaSimulation  # noqa: E402
from sumo_integration.sumo_simulation import SumoSimulation  # noqa: E402
from run_synchronization import SimulationSynchronization  # noqa: E402
from util.netconvert_carla import netconvert_carla  # noqa: E402


# ================================================================================================
# Helpers (kept minimal)
# ================================================================================================

def write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, additional_traci_clients=0):
    root = ET.Element('configuration')
    input_tag = ET.SubElement(root, 'input')
    ET.SubElement(input_tag, 'net-file', {'value': net_file})
    ET.SubElement(input_tag, 'route-files', {'value': vtypes_file})

    gui_tag = ET.SubElement(root, 'gui_only')
    ET.SubElement(gui_tag, 'gui-settings-file', {'value': viewsettings_file})

    ET.SubElement(root, 'num-clients', {'value': str(additional_traci_clients + 1)})

    tree = ET.ElementTree(root)
    tree.write(cfg_file, pretty_print=True, encoding='UTF-8', xml_declaration=True)


def _parse_edge_allowlist(edge_ids_csv: str, edge_file: Optional[str]) -> Set[str]:
    allow: Set[str] = set()
    if edge_ids_csv:
        allow.update([p.strip() for p in edge_ids_csv.split(',') if p.strip()])
    if edge_file:
        with open(edge_file, 'r', encoding='utf-8') as f:
            for line in f:
                eid = line.strip()
                if eid and (not eid.startswith('#')):
                    allow.add(eid)
    return allow


def _validate_allowlist(allowlist: Set[str], sumo_edges) -> Set[str]:
    if not allowlist:
        return set()
    net_edge_ids = {e.getID() for e in sumo_edges}
    missing = sorted(allowlist - net_edge_ids)
    if missing:
        logging.warning("Some edge IDs are not in the SUMO net and will be ignored: %s", missing)
    valid = {eid for eid in allowlist if eid in net_edge_ids}
    if not valid:
        raise RuntimeError("Edge allowlist became empty after validation.")
    return valid


def _parse_routes(route_edges_csv: str, route_file: Optional[str]) -> List[List[str]]:
    routes: List[List[str]] = []
    if route_edges_csv:
        r = [x.strip() for x in route_edges_csv.split(',') if x.strip()]
        if r:
            routes.append(r)
    if route_file:
        with open(route_file, 'r', encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if (not s) or s.startswith('#'):
                    continue
                r = [x.strip() for x in s.split(',') if x.strip()]
                if r:
                    routes.append(r)
    return routes


def _validate_route_edges(sumo_net, route: List[str]) -> None:
    for eid in route:
        sumo_net.getEdge(eid)  # raises if missing
    for a, b in zip(route[:-1], route[1:]):
        ea = sumo_net.getEdge(a)
        outgoing_ids = {e.getID() for e in ea.getOutgoing()}
        if b not in outgoing_ids:
            raise RuntimeError(f"Route not connected: '{a}' does not connect to '{b}'")


def _candidate_spawn_edges(sumo_edges, vclass: str, edge_allowlist: Optional[Set[str]]):
    if edge_allowlist:
        return [e for e in sumo_edges if (e.getID() in edge_allowlist and e.allows(vclass))]
    return [e for e in sumo_edges if e.allows(vclass)]


def _safe_get_vehicle_param(veh_id: str, key: str, default: str = "0") -> str:
    try:
        val = traci.vehicle.getParameter(veh_id, key)
        return val if val != "" else default
    except Exception:
        return default


def _spawn_one_vehicle_with_route(veh_id: str, route_id: str, type_id: str, route_edges: List[str], depart_lane: str):
    if route_id in traci.route.getIDList():
        traci.route.remove(route_id)
    if veh_id in traci.vehicle.getIDList():
        traci.vehicle.remove(veh_id)
    traci.route.add(route_id, route_edges)
    traci.vehicle.add(veh_id, route_id, typeID=type_id, departLane=depart_lane)


def parse_transform(s: str) -> carla.Transform:
    parts = [p.strip() for p in s.split(',') if p.strip()]
    if len(parts) not in (4, 6):
        raise ValueError('ego-spawn-transform must be "x,y,z,yaw[,pitch,roll]"')
    x, y, z, yaw = map(float, parts[:4])
    pitch = float(parts[4]) if len(parts) == 6 else 0.0
    roll = float(parts[5]) if len(parts) == 6 else 0.0
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw, pitch=pitch, roll=roll))


def spawn_carla_ego(world,
                    blueprint_filter: str = 'vehicle.audi.etron',
                    spawn_index: int = 0,
                    spawn_transform: str = ''):
    bp_lib = world.get_blueprint_library()
    ego_bp = bp_lib.filter(blueprint_filter)[0]
    ego_bp.set_attribute('role_name', 'hero')

    if spawn_transform:
        tf = parse_transform(spawn_transform)
    else:
        sps = world.get_map().get_spawn_points()
        if not sps:
            raise RuntimeError("No CARLA spawn points available.")
        if spawn_index < 0 or spawn_index >= len(sps):
            raise ValueError(f"ego-spawn-index {spawn_index} out of range [0, {len(sps)-1}]")
        tf = sps[spawn_index]

    ego = world.spawn_actor(ego_bp, tf)
    return ego, tf


def get_nearby_vehicles(world, ego_actor, radius_m, max_neighbors):
    ego_loc = ego_actor.get_location()
    cands = []
    for a in world.get_actors().filter('vehicle.*'):
        if a.id == ego_actor.id:
            continue
        try:
            d = a.get_location().distance(ego_loc)
        except Exception:
            continue
        if d <= radius_m:
            cands.append((d, a))
    cands.sort(key=lambda x: x[0])
    return [a for _, a in cands[:max_neighbors]]

# =========================
# ADD THESE HELPERS (top-level, near other helpers)
# =========================

def destroy_all_carla_vehicles(client: carla.Client, world: carla.World, keep_ids: Optional[Set[int]] = None):
    """
    Destroy all CARLA vehicle actors in the current world.
    keep_ids: optional set of actor.id to keep (e.g., keep ego if you want)
    """
    keep_ids = keep_ids or set()
    vehicles = world.get_actors().filter('vehicle.*')
    ids = [a.id for a in vehicles if a.id not in keep_ids]
    if not ids:
        return
    # Batch destroy is faster / more reliable than per-actor destroy
    cmds = [carla.command.DestroyActor(x) for x in ids]
    client.apply_batch(cmds)


def destroy_all_sumo_vehicles():
    """
    Remove all vehicles currently known to TraCI.
    """
    try:
        for vid in list(traci.vehicle.getIDList()):
            try:
                traci.vehicle.remove(vid)
            except Exception:
                pass
    except Exception:
        pass

# ================================================================================================
# Prediction-based invTTC + dashboard
# ================================================================================================

class TTCComputer:
    """Inverse TTC distribution based on predicted trajectories (rectangular hit test)."""
    def __init__(self, rx, ry, dt, pred_len, k_plot):
        self.rx = float(rx)
        self.ry = float(ry)
        self.dt = float(dt)
        self.pred_len = int(pred_len)
        self.k_plot = int(k_plot)

    def compute_inv_ttc(self, planned: np.ndarray, pred_neighbors: np.ndarray):
        """
        planned: (pred_len, 2)   ego planned
        pred_neighbors: (N, K, pred_len, 2)
        returns: list length N, each (k_plot,)
        """
        t_array = np.arange(1, self.pred_len + 1) * self.dt
        inv_ttc_list = []

        for n in range(pred_neighbors.shape[0]):
            neighbor_preds = pred_neighbors[n]  # (K, pred_len, 2)
            invs = []
            K = min(self.k_plot, neighbor_preds.shape[0])

            for m in range(K):
                traj = neighbor_preds[m]
                dx = np.abs(planned[:, 0] - traj[:, 0])
                dy = np.abs(planned[:, 1] - traj[:, 1])
                hits = np.where((dx <= self.rx) & (dy <= self.ry))[0]
                if hits.size > 0:
                    ttc = float(t_array[int(hits[0])])
                    invs.append(1.0 / ttc if ttc > 0 else 0.0)
                else:
                    invs.append(0.0)

            if K < self.k_plot:
                invs.extend([0.0] * (self.k_plot - K))

            inv_ttc_list.append(np.array(invs, dtype=float))

        return inv_ttc_list


class TTCDashboardDynamicK:
    """Each row: K-bar invTTC dist + max invTTC history."""
    def __init__(self, max_pairs=8, k_plot=8, history_len=200):
        self.max_pairs = int(max_pairs)
        self.k_plot = int(k_plot)
        self.history_len = int(history_len)

        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Prediction-based Inverse TTC (Mainline Only)")
        self.win.resize(1100, 650)
        self.win.move(0, 0)
        self.win.show()

        self.bar_plots = []
        self.line_plots = []
        self.bar_items = []
        self.line_curves = []
        self.line_data = []

        for row in range(self.max_pairs):
            p_bar = self.win.addPlot(row=row, col=0, title="Pair ego-(empty) ITTC Dist")
            p_bar.setLabel('bottom', 'Mode')
            p_bar.setLabel('left', 'Inv TTC')
            bg = pg.BarGraphItem(x=np.arange(self.k_plot), height=np.zeros(self.k_plot), width=0.6)
            p_bar.addItem(bg)

            p_line = self.win.addPlot(row=row, col=1, title="Pair ego-(empty) Max ITTC")
            p_line.setLabel('bottom', 'Update')
            p_line.setLabel('left', 'Max Inv TTC')
            curve = p_line.plot(np.zeros(self.history_len), pen='y')

            self.bar_plots.append(p_bar)
            self.line_plots.append(p_line)
            self.bar_items.append(bg)
            self.line_curves.append(curve)
            self.line_data.append(np.zeros(self.history_len, dtype=float))

        self.app.processEvents()

    def update(self, neighbor_ids, inv_ttc_list):
        for i in range(self.max_pairs):
            if i < len(neighbor_ids):
                nid = neighbor_ids[i]
                invs = inv_ttc_list[i] if i < len(inv_ttc_list) else np.zeros(self.k_plot)
                self.bar_plots[i].setTitle(f"Pair ego-{nid} ITTC Dist")
                self.line_plots[i].setTitle(f"Pair ego-{nid} Max ITTC")
                self.bar_items[i].setOpts(height=invs)

                hist = self.line_data[i]
                hist[:-1] = hist[1:]
                hist[-1] = float(np.max(invs)) if len(invs) else 0.0
                self.line_curves[i].setData(hist)
            else:
                self.bar_plots[i].setTitle("Pair ego-(empty) ITTC Dist")
                self.line_plots[i].setTitle("Pair ego-(empty) Max ITTC")
                self.bar_items[i].setOpts(height=np.zeros(self.k_plot))

                hist = self.line_data[i]
                hist[:-1] = hist[1:]
                hist[-1] = 0.0
                self.line_curves[i].setData(hist)

        self.app.processEvents()


def draw_history(world, traj_hist: Dict[int, List[Tuple[float, float, float]]], actors, life_time: float):
    for a in actors:
        buf = traj_hist.get(a.id, [])
        if not buf:
            continue
        x, y, z = buf[-1]
        for i in range(1, len(buf)):
            x0, y0, z0 = buf[i - 1]
            x1, y1, z1 = buf[i]
            world.debug.draw_line(
                carla.Location(x=x0, y=y0, z=z0 + 1.0),
                carla.Location(x=x1, y=y1, z=z1 + 1.0),
                thickness=0.05,
                color=carla.Color(0, 255, 0),
                life_time=life_time
            )
        world.debug.draw_string(
            carla.Location(x=x, y=y, z=z + 2.0),
            f"ID={a.id}",
            draw_shadow=True,
            color=carla.Color(255, 255, 0),
            life_time=life_time
        )


def draw_neighbor_predictions_and_ego_plan(world,
                                          pred_neighbors: np.ndarray,
                                          planned_np: np.ndarray,
                                          planned_all_neighbors: np.ndarray,
                                          K_plot: int,
                                          pred_len: int,
                                          life_time: float):
    """
    pred_neighbors: (N, K, pred_len, 2) neighbors only
    planned_np: (pred_len,2) ego plan (draw only this for ego)
    planned_all_neighbors: (N, pred_len,2) neighbor baseline for selecting "best modes"
    """
    h = 1.0

    # Draw neighbor predictions (blue)
    if pred_neighbors is not None and pred_neighbors.size > 0:
        N = pred_neighbors.shape[0]
        K = pred_neighbors.shape[1]
        for n in range(N):
            errors = np.mean(
                np.sum((pred_neighbors[n] - planned_all_neighbors[n][None, :, :]) ** 2, axis=-1),
                axis=1
            )  # (K,)
            best_k = np.argsort(errors)[:min(K_plot, K)]

            for k in best_k:
                for i in range(1, pred_len):
                    x0, y0 = float(pred_neighbors[n, k, i - 1, 0]), float(pred_neighbors[n, k, i - 1, 1])
                    x1, y1 = float(pred_neighbors[n, k, i, 0]), float(pred_neighbors[n, k, i, 1])
                    world.debug.draw_line(
                        carla.Location(x=x0, y=y0, z=h),
                        carla.Location(x=x1, y=y1, z=h),
                        thickness=0.10,
                        color=carla.Color(66, 27, 123),
                        life_time=life_time
                    )

    # Draw ego planned trajectory only (magenta)
    for i in range(1, pred_len):
        x0, y0 = planned_np[i - 1]
        x1, y1 = planned_np[i]
        world.debug.draw_line(
            carla.Location(x=float(x0), y=float(y0), z=h),
            carla.Location(x=float(x1), y=float(y1), z=h),
            thickness=0.10,
            color=carla.Color(162,217,77),
            life_time=life_time
        )


# ================================================================================================
# Main
# ================================================================================================

def main(args):
    tmpdir = tempfile.mkdtemp()

    # CARLA
    carla_simulation = CarlaSimulation(args.host, args.port, args.step_length)
    world = carla_simulation.client.get_world()
    current_map = world.get_map()

    xodr_file = os.path.join(tmpdir, current_map.name + '.xodr')
    current_map.save_to_disk(xodr_file)

    # SUMO
    net_file = os.path.join(tmpdir, current_map.name + '.net.xml')
    netconvert_carla(xodr_file, net_file, guess_tls=True)

    basedir = os.path.dirname(os.path.realpath(__file__))
    cfg_file = os.path.join(tmpdir, current_map.name + '.sumocfg')
    vtypes_file = os.path.join(basedir, 'examples', 'carlavtypes.rou.xml')
    viewsettings_file = os.path.join(basedir, 'examples', 'viewsettings.xml')
    write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, args.additional_traci_clients)

    sumo_net = sumolib.net.readNet(net_file)
    sumo_simulation = SumoSimulation(
        cfg_file,
        args.step_length,
        host=args.sumo_host,
        port=args.sumo_port,
        sumo_gui=args.sumo_gui,
        client_order=args.client_order,
    )

    synchronization = SimulationSynchronization(
        sumo_simulation,
        carla_simulation,
        args.tls_manager,
        args.sync_vehicle_color,
        args.sync_vehicle_lights,
    )

    # Ego delayed spawn state
    ego = None
    ego_tf = None
    tm = carla_simulation.client.get_trafficmanager(8000)
    sim_time = 0.0
    ego_spawned = False

    # MARTS model
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu
    opts = load_config(args.config)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    setup_seed(args.seed)

    model = MARTS(opts).to(device)
    ckpt_path = os.path.join(
        './HGT_model/checkpoints',
        os.path.basename(args.config).split('.')[0],
        args.dataset + '_ckpt_best.pth'
    )
    ckpt = torch.load(ckpt_path, map_location=device)
    model.load_state_dict(ckpt['state_dict'], strict=True)
    model.eval()

    # TTC + dashboard
    obs_len = int(args.obs_len)
    pred_len = int(args.pred_len)
    K_plot = int(args.k_plot)
    scale = float(args.scale)

    draw_life_time = float(args.draw_life_time)

    traj_hist: Dict[int, List[Tuple[float, float, float]]] = {}
    ttc_comp = TTCComputer(args.rx, args.ry, dt=args.step_length, pred_len=pred_len, k_plot=K_plot)
    dashboard = TTCDashboardDynamicK(max_pairs=args.max_neighbors, k_plot=K_plot, history_len=200)

    # MAIN spawning config (SUMO)
    with open('data/vtypes.json', 'r', encoding='utf-8') as f:
        vtypes = json.load(f)['carla_blueprints']

    blueprints = list(vtypes.keys())
    blueprints = list(filter(re.compile(args.filterv).search, blueprints))
    if args.safe:
        blueprints = [x for x in blueprints if vtypes[x]['vClass'] not in ('motorcycle', 'bicycle')]
    if not blueprints:
        raise RuntimeError("No blueprints available after filtering.")

    sumo_edges = sumo_net.getEdges()
    main_allow = _parse_edge_allowlist(args.main_edge_ids, args.main_edge_file)
    if main_allow:
        main_allow = _validate_allowlist(main_allow, sumo_edges)

    main_type_id = args.main_type_id or blueprints[0]
    if main_type_id not in vtypes:
        raise RuntimeError(f"--main-type-id '{main_type_id}' not found in data/vtypes.json")
    main_vclass = vtypes[main_type_id]['vClass']

    main_routes = _parse_routes(args.main_route_edges, args.main_route_file)
    use_fixed_main_routes = len(main_routes) > 0
    main_forced_start_edge_id = None

    if use_fixed_main_routes:
        for r in main_routes:
            _validate_route_edges(sumo_net, r)
        main_forced_start_edge_id = main_routes[0][0]
        e0 = sumo_net.getEdge(main_forced_start_edge_id)
        if not e0.allows(main_vclass):
            raise RuntimeError(f"Forced start edge '{main_forced_start_edge_id}' disallows vClass='{main_vclass}'.")
        logging.info("Fixed routes enabled. Non-fixed MAIN vehicles spawn at '%s'.", main_forced_start_edge_id)

    main_candidate_edges = _candidate_spawn_edges(sumo_edges, main_vclass, main_allow if main_allow else None)
    if args.num_main_vehicles > 0 and (not use_fixed_main_routes) and (not main_candidate_edges):
        raise RuntimeError("No candidate spawn edges for MAIN vehicles.")

    # Spawn MAIN vehicles at t=0
    logging.info("Spawning MAIN vehicles: total=%d, fixed_fraction=%.2f",
                 args.num_main_vehicles, args.main_fixed_fraction)

    for i in range(args.num_main_vehicles):
        veh_id = f"main_{i}"
        route_id = f"route_main_{i}"
        is_fixed = bool(use_fixed_main_routes and (random.random() < args.main_fixed_fraction))

        if is_fixed:
            route_edges = random.choice(main_routes)
            _spawn_one_vehicle_with_route(veh_id, route_id, main_type_id, route_edges, depart_lane="random")
            traci.vehicle.setParameter(veh_id, "custom.fixed_route", "1")
        else:
            if use_fixed_main_routes and main_forced_start_edge_id is not None:
                start_edge_id = main_forced_start_edge_id
            else:
                spawn_edge = random.choice(main_candidate_edges)
                start_edge_id = spawn_edge.getID()
            _spawn_one_vehicle_with_route(veh_id, route_id, main_type_id, [start_edge_id], depart_lane="random")
            traci.vehicle.setParameter(veh_id, "custom.fixed_route", "0")

    # Loop
    try:
        while True:
            loop_start = time.time()

            synchronization.tick()
            sim_time += args.step_length
            world = carla_simulation.client.get_world()

            # Spawn ego after delay (once)
            if (not ego_spawned) and (sim_time >= args.ego_spawn_delay):
                ego, ego_tf = spawn_carla_ego(
                    world,
                    blueprint_filter='vehicle.audi.etron',
                    spawn_index=args.ego_spawn_index,
                    spawn_transform=args.ego_spawn_transform
                )
                logging.info("Spawned ego at sim_time=%.2f: id=%s", sim_time, ego.id)

                tm.set_desired_speed(ego, float(args.ego_speed))
                tm.ignore_lights_percentage(ego, 100)
                tm.distance_to_leading_vehicle(ego, 3.0)
                # 2. Enable Auto Lane Change (It is usually True by default, but good to be safe)
                # 1. Allow the Traffic Manager to change lanes to follow the route or avoid obstacles
                tm.auto_lane_change(ego, True)

                # 2. DISABLE all "random" lane changes (the "wandering" behavior)
                tm.random_left_lanechange_percentage(ego, 5.0)
                # tm.random_right_lanechange_percentage(ego, 0.0)
                             
                # Turn on Autopilot first
                ego.set_autopilot(True, 8000)

                # --- NEW CODE: DESTINATION PLANNING ---
                if args.ego_dest_index >= 0:
                    spawn_points = world.get_map().get_spawn_points()
                    
                    if args.ego_dest_index < len(spawn_points):
                        dest_tf = spawn_points[args.ego_dest_index]
                        
                        # 1. Initialize Route Planner
                        grp = GlobalRoutePlanner(world.get_map(), 2.0) # 2.0m sampling resolution
                        
                        try:
                            logging.info(f"Calculating route from ID {args.ego_spawn_index} to ID {args.ego_dest_index}...")
                            
                            # 2. Trace route (Start -> End)
                            route = grp.trace_route(ego_tf.location, dest_tf.location)
                            
                            # 3. Extract just the locations for Traffic Manager
                            path_locations = [w[0].transform.location for w in route]
                            
                            # 4. Force Traffic Manager to follow this path
                            tm.set_path(ego, path_locations)
                            
                            logging.info(f"Route set! Length: {len(path_locations)} waypoints.")
                            
                            # Optional: Draw destination marker
                            # world.debug.draw_point(dest_tf.location, size=0.5, color=carla.Color(255,0,0), life_time=60.0)
                            # world.debug.draw_string(dest_tf.location, "DEST", color=carla.Color(255,0,0), life_time=60.0)

                        except Exception as e:
                            logging.error(f"Failed to compute route: {e}")
                    else:
                        logging.error(f"Destination Index {args.ego_dest_index} is out of range!")
                # --------------------------------------

                ego_spawned = True

            # If ego not spawned yet, pace the loop and continue
            if not ego_spawned:
                elapsed = time.time() - loop_start
                if elapsed < args.step_length:
                    time.sleep(args.step_length - elapsed)
                continue

            # Dynamic neighbors
            neighbors = get_nearby_vehicles(world, ego, args.neighbor_radius, args.max_neighbors)
            neighbor_ids = [a.id for a in neighbors]

            actors_all = [ego] + neighbors     # for history drawing
            actors_pred = neighbors            # MARTS predicts neighbors ONLY

            # Update history buffers (ego + neighbors)
            for a in actors_all:
                try:
                    tr = a.get_transform()
                except Exception:
                    continue
                x, y, z = tr.location.x, tr.location.y, tr.location.z
                traj_hist.setdefault(a.id, []).append((x, y, z))
                if len(traj_hist[a.id]) > obs_len:
                    traj_hist[a.id].pop(0)

            # Draw history always
            # draw_history(world, traj_hist, actors_all, life_time=0.5)

            # ----------------------------
            # Ego planned trajectory (always computed if ego has >=2 points)
            # ----------------------------
            ego_hist = traj_hist.get(ego.id, [])
            if len(ego_hist) < 2:
                dashboard.update(neighbor_ids, [np.zeros(K_plot) for _ in neighbor_ids])
                elapsed = time.time() - loop_start
                if elapsed < args.step_length:
                    time.sleep(args.step_length - elapsed)
                continue

            ego_xy = np.array([(xx, yy) for (xx, yy, _) in ego_hist], dtype=np.float32)
            ego_now = ego_xy[-1]
            ego_prev = ego_xy[-2]
            vel_step = ego_now - ego_prev  # per-step displacement
            planned_np = np.array([ego_now + vel_step * (i + 1) for i in range(pred_len)], dtype=np.float32)

            # If no neighbors, draw only ego plan and keep dashboard empty
            if len(actors_pred) == 0:
                dashboard.update([], [])
                draw_neighbor_predictions_and_ego_plan(
                    world,
                    pred_neighbors=np.zeros((0, K_plot, pred_len, 2), dtype=np.float32),
                    planned_np=planned_np,
                    planned_all_neighbors=np.zeros((0, pred_len, 2), dtype=np.float32),
                    K_plot=K_plot,
                    pred_len=pred_len,
                    life_time=draw_life_time
                )
                elapsed = time.time() - loop_start
                if elapsed < args.step_length:
                    time.sleep(args.step_length - elapsed)
                continue

            # ----------------------------
            # Prediction gate: neighbors must have obs_len history
            # ----------------------------
            short = [(ego.id, a.id, len(traj_hist.get(a.id, [])))
                     for a in actors_pred
                     if len(traj_hist.get(a.id, [])) < obs_len]

            if short:
                dashboard.update(neighbor_ids, [np.zeros(K_plot) for _ in neighbor_ids])
                if args.debug:
                    logging.info("Skip prediction: insufficient neighbor history (need %d). Short: %s", obs_len, short)
                elapsed = time.time() - loop_start
                if elapsed < args.step_length:
                    time.sleep(args.step_length - elapsed)
                continue

            # ----------------------------
            # Build model inputs (neighbors ONLY)
            # ----------------------------
            veh_num = len(actors_pred)

            seq_np = np.stack(
                [np.array([(xx, yy) for (xx, yy, _) in traj_hist[a.id]], dtype=np.float32)
                 for a in actors_pred],
                axis=0
            )  # (veh_num, obs_len, 2)

            # planned_all for mode selection (neighbors baseline only)
            planned_all_neighbors = np.zeros((veh_num, pred_len, 2), dtype=np.float32)
            for n in range(veh_num):
                last_n = seq_np[n, -1]
                # use last-step velocity (more stable than (last-first)/obs_len)
                vel_n = seq_np[n, -1] - seq_np[n, -2]
                planned_all_neighbors[n] = np.array([last_n + vel_n * (i + 1) for i in range(pred_len)], dtype=np.float32)

            # MARTS inference
            arr = torch.from_numpy(seq_np).unsqueeze(0).float().to(device)  # (1, veh_num, obs_len, 2)

            t_rel = arr - arr[:, :, obs_len - 1:obs_len, :]
            x_abs = t_rel[:, :, :obs_len, :]

            planned = torch.from_numpy(planned_np).unsqueeze(0).float().to(device)  # (1, pred_len, 2)

            x_rel = torch.zeros_like(x_abs)
            x_rel[:, :, 1:] = x_abs[:, :, 1:] - x_abs[:, :, :-1]
            x_rel[:, :, 0] = x_rel[:, :, 1]

            with torch.no_grad():
                x_abs_s = x_abs * scale
                x_rel_s = x_rel * scale

                if args.debug:
                    logging.debug("MARTS input shapes: x_abs=%s x_rel=%s planned=%s",
                                  tuple(x_abs_s.shape), tuple(x_rel_s.shape), tuple(planned.shape))

                y_pred, _, _ = model(x_abs_s, x_rel_s, planned)  # (1, veh_num, K, pred_len, 2)

                pos_cur = torch.from_numpy(seq_np[:, -1:, :]).unsqueeze(0).unsqueeze(2).float().to(device)
                pos_cur = pos_cur.repeat(1, 1, y_pred.shape[2], y_pred.shape[3], 1)

                # Your confirmed fix: direction convention
                # y_pred = -y_pred

                y_pred = y_pred * scale
                y_pred = torch.cumsum(y_pred, dim=3) + pos_cur

            pred_np_neighbors = y_pred[0].detach().cpu().numpy()  # (veh_num, K, pred_len, 2)

            # invTTC distribution (ego planned vs neighbors predicted)
            inv_ttc_list = ttc_comp.compute_inv_ttc(planned_np, pred_np_neighbors)
            dashboard.update(neighbor_ids, inv_ttc_list)

            # Draw neighbors predictions + ego planned ONLY
            draw_neighbor_predictions_and_ego_plan(
                world,
                pred_neighbors=pred_np_neighbors,
                planned_np=planned_np,
                planned_all_neighbors=planned_all_neighbors,
                K_plot=K_plot,
                pred_len=pred_len,
                life_time=draw_life_time
            )

            # Reroute non-fixed vehicles in SUMO at route ends
            for vehicle_id in traci.vehicle.getIDList():
                if _safe_get_vehicle_param(vehicle_id, "custom.fixed_route", default="0") == "1":
                    continue

                route = traci.vehicle.getRoute(vehicle_id)
                if not route:
                    continue

                idx = traci.vehicle.getRouteIndex(vehicle_id)
                vclass = traci.vehicle.getVehicleClass(vehicle_id)

                if idx >= (len(route) - 1):
                    try:
                        current_edge = sumo_net.getEdge(route[idx])
                    except Exception:
                        continue

                    outgoing_map = current_edge.getAllowedOutgoing(vclass)
                    next_edge_objs = list(outgoing_map.keys())
                    if not next_edge_objs:
                        continue

                    next_edge_id = random.choice(next_edge_objs).getID()
                    traci.vehicle.setRoute(vehicle_id, [current_edge.getID(), next_edge_id])

            # Real-time pacing
            elapsed = time.time() - loop_start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info("Cancelled by user.")
    # =========================
    # MODIFY YOUR main() FINALLY BLOCK (replace your existing finally block)
    # =========================
    finally:
        # 1) Best-effort: remove SUMO vehicles first (prevents resync from re-spawning in CARLA during shutdown)
        try:
            destroy_all_sumo_vehicles()
        except Exception:
            pass

        # 2) Best-effort: destroy CARLA vehicles (including synced SUMO vehicles + ego)
        try:
            # Re-fetch world in case it changed
            world_cleanup = carla_simulation.client.get_world()
            destroy_all_carla_vehicles(carla_simulation.client, world_cleanup, keep_ids=set())
        except Exception:
            pass

        # 3) Close synchronization / SUMO / CARLA side
        try:
            synchronization.close()
        except Exception:
            pass

        # 4) Remove temp files
        try:
            if os.path.exists(tmpdir):
                shutil.rmtree(tmpdir)
        except Exception:
            pass

        logging.info("Cleanup done: destroyed SUMO + CARLA vehicles and closed co-simulation.")



# ================================================================================================
# CLI
# ================================================================================================

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)

    # CARLA
    argparser.add_argument('--host', default='127.0.0.1')
    argparser.add_argument('-p', '--port', default=2000, type=int)

    # SUMO
    argparser.add_argument('--sumo-host', default=None)
    argparser.add_argument('--sumo-port', default=None, type=int)
    argparser.add_argument('--sumo-gui', action='store_true')

    # Sync
    argparser.add_argument('--step-length', default=0.05, type=float)
    argparser.add_argument('--additional-traci-clients', default=0, type=int)
    argparser.add_argument('--client-order', default=1, type=int)

    # Sync options
    argparser.add_argument('--sync-vehicle-lights', action='store_true')
    argparser.add_argument('--sync-vehicle-color', action='store_true')
    argparser.add_argument('--sync-vehicle-all', action='store_true')

    argparser.add_argument('--tls-manager', type=str, choices=['none', 'sumo', 'carla'], default='none')

    # Debug
    argparser.add_argument('--debug', action='store_true')

    # Vehicle filters
    argparser.add_argument('--safe', action='store_true')
    argparser.add_argument('--filterv', default='vehicle.*')

    # MAINLINE only
    argparser.add_argument('--num-main-vehicles', type=int, default=10)
    argparser.add_argument('--main-fixed-fraction', type=float, default=0.5)

    argparser.add_argument('--main-edge-ids', default='')
    argparser.add_argument('--main-edge-file', default=None)

    argparser.add_argument('--main-type-id', default=None)

    # Main routes (keep this feature)
    argparser.add_argument('--main-route-edges', default='')
    argparser.add_argument('--main-route-file', default=None)

    # Ego spawn
    argparser.add_argument('--ego-spawn-index', type=int, default=0)
    argparser.add_argument('--ego-spawn-transform', type=str, default='')
    argparser.add_argument('--ego-spawn-delay', type=float, default=0.0,
                           help='Spawn ego after this many seconds of simulation time (default: 0).')
    argparser.add_argument('--ego-speed', type=float, default=40.0,
                           help='TrafficManager desired speed for ego (TM units).')
    # ADD THIS LINE:
    argparser.add_argument('--ego-dest-index', type=int, default=-1, 
                           help='Spawn point ID for ego destination. -1 means random roaming.')
    
    # Prediction / TTC visualization
    argparser.add_argument('--seed', type=int, default=1)
    argparser.add_argument('--config', type=str, default='./HGT_model/configs/mart_carla.yaml')
    argparser.add_argument('--dataset', type=str, default='carla')
    argparser.add_argument('--gpu', type=str, default='0')

    argparser.add_argument('--obs-len', type=int, default=30)
    argparser.add_argument('--pred-len', type=int, default=50)
    argparser.add_argument('--k-plot', type=int, default=8)
    argparser.add_argument('--scale', type=float, default=2.5)

    argparser.add_argument('--rx', type=float, default=5.0)
    argparser.add_argument('--ry', type=float, default=2.0)
    argparser.add_argument('--neighbor-radius', type=float, default=50.0)
    argparser.add_argument('--max-neighbors', type=int, default=8)

    # Drawing
    argparser.add_argument('--draw-life-time', type=float, default=0.5,
                           help='CARLA debug primitive lifetime (seconds). Increase if you do not see lines.')

    args = argparser.parse_args()

    if args.main_fixed_fraction < 0.0 or args.main_fixed_fraction > 1.0:
        raise ValueError("--main-fixed-fraction must be in [0,1].")

    if args.sync_vehicle_all:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    logging.basicConfig(
        format='%(levelname)s: %(message)s',
        level=logging.DEBUG if args.debug else logging.INFO
    )

    main(args)
