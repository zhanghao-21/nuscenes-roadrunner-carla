# nuScenes map region → OpenDRIVE → CARLA 0.9.15

Convert the region covered by a single nuScenes scene into an OpenDRIVE (`.xodr`)
map and load it in CARLA. Defaults target the **first mini scene** (`scene-0061`,
map `singapore-onenorth`).

## Why two Python environments?

CARLA 0.9.15 ships **only a Python 3.7 binding** (`carla-0.9.15-cp37-cp37m`).
There is **no official `carla` wheel for Python 3.12**, so the *loader* must run
on Python 3.7. The *converter* uses only `nuscenes-devkit`/`shapely`/`numpy`
and runs fine on your existing Python 3.12.

| Step | Script | Python | Needs CARLA? |
|------|--------|--------|--------------|
| Convert map → `.xodr` | `convert_nuscenes_to_xodr.py` | 3.12 (existing) | no |
| Load `.xodr` + spawn ego | `load_xodr_in_carla.py` | **3.7** (`carla915`) | yes (running server) |

## 1. Create the CARLA client env (Python 3.7)

```bash
conda env create -f environment_carla37.yml
conda activate carla915
pip install D:/research/nuscenes_carla/carla_0.9.15/PythonAPI/carla/dist/carla-0.9.15-cp37-cp37m-win_amd64.whl
python -c "import carla; print('carla ok')"
```

## 2. Convert the scene region to OpenDRIVE (Python 3.12)

```bash
python convert_nuscenes_to_xodr.py            # first mini scene (scene-0061)
python convert_nuscenes_to_xodr.py --all-scenes   # every scene in the version
# options:
#   --scene-index 0           first mini scene (scene-0061)  [default]
#   --scene-name scene-0103   pick a scene by name instead
#   --all-scenes              convert all scenes (auto-named outputs)
#   --padding 50              metres around the ego-trajectory bbox
#   --resolution 0.5          centerline sampling step (m), smaller = smoother
#   --lane-width 3.5          fixed width (default: estimate per lane)
#   --crosswalks              also emit crosswalks (off by default)
#   --buildings               also synthesize buildings (off by default)
```

Outputs are auto-named `output/<map>_<scene>.xodr` (+ `_meta.json` sidecar with
origin offset, ego pose/trajectory and per-frame agents). Example:
`output/singapore-onenorth_scene-0061.xodr`.

## 3. Start the simulator, then load the map (Python 3.7)

```bash
# terminal A — start the server:
D:/research/nuscenes_carla/carla_0.9.15/CarlaUE4.exe
#   (low-spec: add  -quality-level=Low -RenderOffScreen)

# terminal B — pick a scenario by id (0..9, nuScenes mini order):
conda activate carla915
python load_xodr_in_carla.py --list             # show id -> scene mapping
python load_xodr_in_carla.py --scenario-id 0    # load scenario 0 (scene-0061)
python load_xodr_in_carla.py --scene 0103       # or pick by scene number
python load_xodr_in_carla.py --xodr output/boston-seaport_scene-0553.xodr  # or explicit path
#   --no-ego        load the map only
#   --extra-width   widen lanes if the mesh looks too thin
```

Run **all** scenarios back-to-back (each plays once) — handy for batch-rendering
cameras across the whole mini set:
```bash
python load_xodr_in_carla.py --all-scenarios --follow --cameras
python load_xodr_in_carla.py --all-scenarios --replay --cameras
```

Scenario ids (from `--list`):

| id | scene | map |
|----|-------|-----|
| 0 | scene-0061 | singapore-onenorth |
| 1 | scene-0103 | boston-seaport |
| 2 | scene-0553 | boston-seaport |
| 3 | scene-0655 | boston-seaport |
| 4 | scene-0757 | boston-seaport |
| 5 | scene-0796 | singapore-queenstown |
| 6 | scene-0916 | singapore-queenstown |
| 7 | scene-1077 | singapore-hollandvillage |
| 8 | scene-1094 | singapore-hollandvillage |
| 9 | scene-1100 | singapore-hollandvillage |

The loader generates the world via `client.generate_opendrive_world(...)`,
snaps the nuScenes ego start pose onto the nearest drivable lane, and spawns a
vehicle there. CARLA flips Y when ingesting OpenDRIVE, so the loader converts
`(x, y, yaw) → (x, -y, -yaw)`.

### Driving the ego

After spawning, the ego drives:

```bash
# CARLA Traffic Manager autopilot (default):
python load_xodr_in_carla.py --xodr output/singapore-onenorth_scene-0061.xodr

# Replay the real recorded nuScenes path AND the other scene agents
# (cars, trucks, buses, bikes, pedestrians) moving in sync:
python load_xodr_in_carla.py --xodr output/singapore-onenorth_scene-0061.xodr --replay
```

### Drive it yourself (manual control)

```bash
python load_xodr_in_carla.py --scenario-id 0 --manual
python load_xodr_in_carla.py --scenario-id 0 --manual --joystick   # gamepad/wheel
```

Opens a pygame window with a chase view and HUD. Controls: **W/S** throttle-brake,
**A/D** (or arrows) steer, **Space** handbrake, **Q** toggle reverse, **P** toggle
autopilot, **ESC** quit. `--manual` takes precedence over `--replay`. Needs
`pygame` in the `carla915` env (`pip install pygame==2.1.2`).

The recorded nuScenes agents keep replaying around you while you drive (real-time,
looped). Use `--no-agents` to drive on an empty map, `--max-agents N` to cap them,
`--replay-speed S` to slow/speed their playback.

### Follow the real path with CARLA's controller

```bash
python load_xodr_in_carla.py --scenario-id 0 --follow
```

Instead of teleporting the ego (replay) or driving it yourself (manual), this
drives the ego **physically** with CARLA's `VehiclePIDController`, using the
recorded nuScenes ego trajectory as the plan (target waypoints + per-segment
speed). The surrounding recorded agents keep replaying (kept in step with the
ego's progress along the path). The colored history/future trajectory overlay is
drawn (same colors as replay), plus a red **GOAL** marker on the waypoint the
controller is currently driving to. Runs in synchronous mode and needs the CARLA
`agents` package (`--carla-agents` points at `carla_0.9.15/PythonAPI/carla`, the
default). `--no-traj` hides the overlay.

Add `--cameras` to also save the 6-camera views in follow mode; they go to a
**separate** folder `output/cameras_follow/<scene>/<CAM>/<frame>.png` (one set
per keyframe) so they don't clash with replay's `output/cameras`.

Replay flags:
- `--no-agents`      replay the ego only (no other agents)
- `--max-agents N`   cap the number of other agents (by first appearance)
- `--replay-speed S` playback speed (1.0 = real time, e.g. 0.5 = half speed)
- `--substeps N`     interpolation steps between 2 Hz keyframes (default 10)
- `--loop`           loop the replay until Ctrl+C
- `--no-drive`       just spawn the ego, don't move it

Replay runs in **synchronous mode**: poses are applied deterministically each
tick (no camera flicker/jitter), the ego path is smoothed with Catmull-Rom
interpolation across keyframes (even velocity), ground height is cached for even
frame timing, the chase camera yaw is damped, and every actor is grounded by its
own bounding box (pedestrians don't sink, vehicles don't float). Async mode is
restored on exit.

**Trajectory overlay** — during replay, past/future trajectories are drawn for
the ego *and* every agent that has trajectory data, sampled at `--traj-hz`:
- `--hist-seconds 2`    seconds of past trajectory (drawn dimmer)
- `--future-seconds 6`  seconds of future trajectory (drawn bright)
- `--traj-hz 2`         sampling rate of the drawn points
- `--no-traj`           turn the overlay off

Colours: **ego = gold**, **vehicles = blue**, **pedestrians = green**,
**cyclists/motorcycles = magenta**; history is a darker shade of each, future is
the bright shade.

### nuScenes-style 6-camera rig (CALIBRATED by default)

Attach the 6 nuScenes cameras and save each view per keyframe:

```bash
python load_xodr_in_carla.py --scenario-id 0 --replay --cameras --no-loop
#   --cam-dir DIR      output folder (default output/cameras)
#   --calib-dir DIR    phase0 calib sidecars (default ../experiments/phase0_calib_gt/output)
#   --approx-rig       force the legacy approximate rig
#   --no-loop          play the scene once (replay loops forever by default)
```

When the scene's calibration sidecar exists (produced by
`experiments/phase0_calib_gt/export_calib_gt.py`), the rig is **calibrated**:
free-floating sensors teleported each tick to `ego_pose ∘ real extrinsic`,
rendering at the native **1600×900** with per-camera **FOV = 2·atan(W/2fx)**
from the real intrinsics (e.g. CAM_FRONT 64.56°, not the old 70°). Saved views
then share the real cameras' projection and are directly usable for
calibration-sensitive evaluation (BEVFormer etc. via
`experiments/exp2a_perception/pack_loader_renders.py`).

Without a sidecar (or with `--approx-rig`), the legacy attached rig is used:
`CAM_FRONT`, `CAM_FRONT_LEFT/RIGHT` (≈70°), `CAM_BACK` (≈110°),
`CAM_BACK_LEFT/RIGHT` (≈70°) at roughly the nuScenes mounts, rendered at
`--cam-width`×`--cam-height` (default 800×450). Frames are saved to
`output/cameras/<scene>/<CAM_NAME>/<frame>.png` (one set per 2 Hz keyframe),
mirroring the nuScenes per-camera folder layout.

The other agents come from the nuScenes `sample_annotation` keyframes (2 Hz),
exported into the `_meta.json` sidecar by the converter. By default only dynamic
agents (`vehicle.*`, `human.pedestrian.*`) are exported; pass `--include-static`
to `convert_nuscenes_to_xodr.py` to also export cones/barriers/debris.
nuScenes categories are mapped to representative CARLA blueprints; replay agents
have physics disabled and are teleported each frame, then destroyed on exit.

## Compare simulated vs. real camera views

`align_sim_vs_real.ipynb` builds a side-by-side figure per keyframe: the 6
**real** nuScenes camera images (top two rows) next to the 6 **simulated** camera
views (bottom two rows), aligned at the same time step, saved to
`output/aligned/<scene>/<frame>.png`.

Run order:
1. Generate simulated frames: `python load_xodr_in_carla.py --scenario-id 0 --replay --cameras`
2. Open the notebook in the **`nusc2xodr`** env, set `SCENE` (or `None` for all
   scenes with sim frames), and run all cells.

Missing simulated cameras render as a "missing" placeholder, so the real-data
side always works even before you run the loader.

The notebook builds **zero-padding, labelled 2×3 montages** of the 6 cameras and
animates them into GIFs per scene (2 Hz, no gaps), all in the notebook:
`nuscenes_real.gif`, `carla_sim.gif`, and `comparison.gif` (real over sim, with
REAL/SIM banners). Set **`MODE`** to choose which CARLA run to compare:
- `MODE = "replay"` → `output/cameras` → `output/aligned/<scene>/`
- `MODE = "follow"` → `output/cameras_follow` → `output/aligned_follow/<scene>/`

The real GIF is built for any scene; the sim/comparison GIFs need that scene's
camera frames (run the loader with `--cameras` in the matching mode). Set
`SCENE = None` to process every scene that has sim frames; shrink `CELL` for
smaller files.

## Align OpenStreetMap to the scenario

`align_osm.ipynb` georeferences each scenario region (nuScenes maps have known
lat/lon origins) and overlays it on **OpenStreetMap**: it projects the region +
ego trajectory to WGS84, fetches OSM roads/buildings + `traffic_signals` for that
bbox (Overpass API, stdlib only — needs internet), and plots OSM with the
nuScenes lanes (blue), ego trajectory (red), and real **traffic-signal
locations** (yellow) on top. Figures are saved to `output/osm_aligned/<scene>.png`.
The signal count/positions match what `--traffic-lights --osm-signals` uses.

Set `SCENE` (or `None` for all scenarios with a sidecar), `SHOW_LANES`, and run
in the **`nusc2xodr`** kernel. Uses only the scenario sidecars + `NuScenesMap`
(no camera frames needed), so it works for any converted scene.

## How the conversion works

1. Walk the scene's samples → ego trajectory → padded bounding box = region patch.
2. Select `lane` + `lane_connector` records intersecting the patch.
3. Discretize each centerline (arcline geometry) → polyline, shifted to a local origin.
4. Per-lane width = polygon area / centerline length (clamped 2–6 m).
5. Topology: each lane → a `<road>`; each connector → a `<road>` in a `<junction>`.
   Connectors sharing an incoming/outgoing lane are merged into one junction.
6. Emit OpenDRIVE 1.4 (planView = line chain, one driving lane id −1 centered on
   the centerline via `laneOffset = width/2`), with road `<link>`s and junctions.

## Map quality features

- **Lane markings** — driven by real nuScenes `lane_divider` segment types
  (dashed/solid/double, white/yellow) → OpenDRIVE `roadMark`. Junction interiors
  are left unmarked, matching the source map.
- **Smoother geometry** — centerlines are discretized at 0.5 m by default
  (`--resolution`), so curves are far less faceted.
- **Crosswalks** (opt-in) — pass `--crosswalks` to the converter to emit
  `ped_crossing` polygons (as OpenDRIVE `<object type="crosswalk">` *and* sidecar
  polygons). Since CARLA doesn't paint those in standalone mode, the loader draws
  them as **zebra stripes**; `--no-crosswalks` (loader) skips drawing. Off by
  default.
- **Buildings** (opt-in) — nuScenes has *no* building layer. Pass `--buildings`
  to the converter to synthesize footprints in the empty blocks between
  roads/walkways into the sidecar, then `--buildings` to the loader to build
  **solid blocks** by tiling a box-shaped static-prop mesh (shipping container by
  default), stacked `--building-layers` high (default 3). `--wireframe-buildings`
  for the outline look, `--max-buildings N` to limit. Off by default.
- **Traffic lights** (opt-in) — pass `--traffic-lights` to the converter to emit
  OpenDRIVE traffic-light `<signal>`s grouped into per-phase `<controller>`s (by
  approach axis). CARLA's OpenDRIVE importer turns these into functional, cycling
  `carla.TrafficLight` groups that autopilot and manual driving obey. Add
  `--osm-signals` to place the **correct number** of lights: it fetches the real
  OSM `traffic_signals` nodes for the region (needs internet) and puts one signal
  per node on the nearest approach lane (e.g. scene-0061 → 4 lights, not 26).
  `--osm-radius` tunes the match distance. Without `--osm-signals`, every
  junction is lit. (Replay/follow ego motion is scripted, so it ignores lights;
  the replayed agents do too.)
- **Weather** — the loader sets a `WeatherParameters` preset (`--weather
  ClearNoon` by default; `--weather none` to leave unchanged).

Example with everything on:
```bash
python load_xodr_in_carla.py --xodr output/singapore-onenorth_scene-0061.xodr \
    --replay --buildings --weather CloudyNoon
```

## Known limitations (v1)

- Geometry uses straight-line segments between 1 m samples (curves are faceted,
  not analytic arcs/spirals). Lower `--resolution` for smoother curves.
- Only `lane`/`lane_connector` are emitted; crosswalks, stop lines, traffic
  lights, sidewalks and elevation are dropped (z = 0, flat).
- Singapore is left-hand-traffic, but lanes are emitted as right-side (id −1) so
  CARLA treats them as normally drivable; travel direction follows the nuScenes
  lane direction.
- Many small single-connector junctions can appear where connectors don't share
  lanes inside the crop — valid OpenDRIVE, just not visually consolidated.
