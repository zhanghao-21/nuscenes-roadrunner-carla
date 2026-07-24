# replay_geo — replay nuScenes scenes on the generated geo maps

Replays each recorded nuScenes-mini scene on top of its **generated map**
(`output_roadrunner/<base>/<base>_geo.xodr`, the geo-referenced OpenDRIVE from
the RoadRunner pipeline). Two self-contained replayers, no imports from
`nuscenes2xodr`:

| script | what it does | env |
|--------|--------------|-----|
| `replay_geo.py` | offline top-down replay → GIF/PNG | Python 3.12 (numpy, matplotlib, Pillow) |
| `replay_geo_carla.py` | live 3D replay in CARLA 0.9.15 | Python 3.7 `carla915` (running server) |
| `make_gifs.py` | GIFs from recorded CARLA chase-cam frames | Python 3.12 |
| `build_aligned_geo.py` | REAL vs SIM aligned panels (like `aligned_full`) | Python 3.12 |

The `_geo.xodr` geometry is in the same patch-local frame as the sidecar's
ego/agent poses (only the geoReference header differs from the converter
output), so map and replay align by construction.

## Usage

```bash
python replay_geo.py --list              # show scenes that have a geo map
python replay_geo.py --scene 0103        # replay one scene (id substring)
python replay_geo.py --all               # replay all 10 scenes
```

Options:

| flag | default | meaning |
|------|---------|---------|
| `--substeps N` | 4 | interpolated frames per 2 Hz keyframe |
| `--fps N` | 8 | GIF playback rate |
| `--hist-seconds S` | 2 | past-trajectory trail (drawn dim) |
| `--future-seconds S` | 6 | future-trajectory trail (drawn bright) |
| `--follow` | off | camera follows the ego instead of full-map view |
| `--window M` | 80 | follow-view size in metres |
| `--save-frames` | off | also save one PNG per 2 Hz keyframe |
| `--dpi N` | 100 | render resolution |

## Outputs (`results/<base>/`)

- `<base>_map.png` — static overview: lane ribbons + boundary markings from the
  geo OpenDRIVE with every recorded trajectory overlaid.
- Both the overview and the replay draw the **OSM building footprints** from
  `output_roadrunner/<base>.osm` under the roads (same selection + projection
  as `nuscenes_ground_buildings.m`: ways tagged `building`, equirectangular
  projection anchored at the published nuScenes map origin, patch-corner
  offset). `--no-buildings` turns the layer off.
- On top of the footprints, the **RoadRunner building boxes** are drawn as cyan
  outlines (matching the MATLAB preview colour): the `BldgN_M` StaticObject
  oriented bounding boxes are decoded straight out of the
  `output_roadrunner/<base>/<base>_city.rrhd` HD-map protobuf — the boxes
  `nuscenes_ground_buildings.m` actually placed in the RoadRunner scene
  (concave footprints split into several boxes, boxes shrunk to clear the
  road). Scenes whose `.rrhd` has no buildings (e.g. scene-0655) just skip the
  layer. `--no-rr-buildings` turns it off.
- `<base>_replay.gif` — the replay: agents as oriented boxes (real nuScenes
  sizes from `wlh`), poses linearly interpolated between the 2 Hz keyframes.
- `frames/NNNN.png` — per-keyframe stills (with `--save-frames`).

Colours follow the project convention: **ego = gold**, **vehicles = blue**,
**pedestrians = green**, **cyclists/motorcycles = magenta**; each agent shows a
dim past trail and a bright future trail. Junction interiors are drawn darker
and unmarked, matching the source map.

## CARLA replay (`replay_geo_carla.py`)

Loads the geo OpenDRIVE into a running CARLA server with
`generate_opendrive_world` and replays ego + all recorded agents live, like
`nuscenes2xodr/load_xodr_in_carla.py --replay`: synchronous stepping,
Catmull-Rom-smoothed ego path, interpolated agent keyframes, per-actor
bounding-box grounding, damped chase camera, and the past/future trajectory
overlay (**ego = red** in CARLA, so it stays distinct from the blue agents).
CARLA flips Y when ingesting OpenDRIVE, so poses are converted
`(x, y, yaw) → (x, -y, -yaw)`.

```bash
# terminal A — start the simulator:
D:/research/nuscenes_carla/carla_0.9.15/CarlaUE4.exe

# terminal B:
conda activate carla915
python replay_geo_carla.py --list
python replay_geo_carla.py --scene 0103            # watch one scene
python replay_geo_carla.py --scene 0061 --record   # + save chase-cam PNGs
python replay_geo_carla.py --all --record          # all 10, back-to-back
```

Options mirror the original loader: `--replay-speed`, `--substeps` (default 10),
`--max-agents`, `--no-agents`, `--loop`, `--no-traj`, `--hist-seconds`,
`--future-seconds`, `--traj-hz`, `--extra-width`, `--weather`. With `--record`,
one chase-camera PNG per 2 Hz keyframe (1280×720, `--rec-width/height`) is
saved to `results/<base>/carla_frames/`.

### Buildings + 6-camera rig in CARLA

- `--buildings` decodes the RoadRunner building boxes from the scene's
  `.rrhd` and outlines them as **persistent cyan wireframe boxes** (debug
  lines — instant, no actor spawning; they render in the spectator and the
  camera sensors alike). Add `--solid-buildings` to instead build solid
  blocks by tiling a box-shaped static prop (shipping container), stacked up
  to `--building-layers` (default 3) high — much slower.
  `--max-buildings N` limits the count in either mode.
- `--cameras` attaches the approximate nuScenes 6-camera rig
  (`--cam-width`×`--cam-height`, default 800×450) and saves each view per
  2 Hz keyframe to `results/<base>/carla_cams/<CAM>/NNN.png`.

### Aligned REAL vs SIM panels (`build_aligned_geo.py`)

Builds one panel per keyframe in the style of
`nuscenes2xodr/output/aligned_full`: the 6 **real** nuScenes camera images
(top), the 6 **CARLA** views captured on the geo map with the RoadRunner
buildings (bottom), and the top-down replay frame (right column). The real
side reads the `v1.0-mini` tables directly with plain `json` (no
nuscenes-devkit); missing sim cameras render as placeholders, so the real +
top-down sides work before the CARLA run. Top-down frames are auto-generated
via `replay_geo.py --save-frames` when absent.

```bash
# 1) capture sim views (carla915 env, server running):
python replay_geo_carla.py --scene 0103 --buildings --cameras

# 2) build the panels (base Python 3.12 env):
python build_aligned_geo.py --scene 0103
```

Output: `results/<base>/aligned/frames/NNN.png` + `results/<base>/aligned/aligned.gif`.

### GIFs of the recorded frames

After recording, the frames are assembled into `results/<base>/<base>_carla.gif`
(real-time 2 Hz playback, half-size by default; `--gif-scale`, `--no-gif`).
If Pillow isn't installed in the `carla915` env, the CARLA script skips this
step and prints a hint — build the GIFs afterwards in the base Python 3.12 env:

```bash
python make_gifs.py                 # all scenes with recorded frames
python make_gifs.py --scene 0103    # one scene
python make_gifs.py --fps 4 --scale 1.0   # 2x speed, full resolution
```
