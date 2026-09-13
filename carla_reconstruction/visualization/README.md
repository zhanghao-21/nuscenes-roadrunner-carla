# Reconstruction comparison visualization

## Continuous Traffic Manager: simulation-only rain and obstacle cases

The TM runner can also capture **continuous closed-loop driving** with six
simulation RGB cameras and one simulated LiDAR. This is separate from the
recorded-pose comparison workflow below: it needs the manifest and trajectory
metadata, but does not need original nuScenes camera/LiDAR recordings.

With CARLA running, use the `carla_0915` environment and run these two separate
cases from the repository root. The ego and all classified moving surrounding
vehicles use TM; parked/single-observation vehicles remain fixed and pedestrians
follow recorded trajectories. Keep the same simulation seed and timing for both.

```bat
python carla_reconstruction\runtime\run_tm_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --ego-mode tm --replay-pedestrians --weather rain --build-sim-panels ^
  --output carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\tm_rain

python carla_reconstruction\runtime\run_tm_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --ego-mode tm --replay-pedestrians --weather clear --obstacle-distance 12 ^
  --build-sim-panels ^
  --output carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\tm_obstacle
```

Choose a new `--output` on repeat runs. Each output contains:

- `capture/carla_cams/`, `capture/lidar/`: indexed PNG images and Nx4 float32
  LiDAR arrays from the actual running simulation.
- `capture/capture_index.json`: matching sensor frame IDs/timestamps, same-frame
  ego poses, speed/control, weather parameters and obstacle placement.
- `sim_panels/figure.png`, `sim_panels/simulation.gif`, `sim_panels/frames/`:
  six cameras in two rows, with one north-up LiDAR panel on the right; no REAL
  rows and no schematic panel. The default still is the sample nearest 2.5 seconds
  for an obstacle case (showing the approach), or 4 seconds for a weather case.
- `summary.json`, `metrics.csv`, `run_config.json`: normal simulation metrics
  plus `environment_scenario` and `sensor_capture` audit records.

Useful controls:

- `--weather default|clear|rain`: preserve loaded weather, use ClearNoon, or
  use HardRainNoon. The previous weather is restored after the run.
- `--obstacle-distance 12`: add one fixed, collision-enabled
  `static.prop.streetbarrier` 12 metres along the directed driving lane from
  the known ego spawn position. The long side faces across the lane. Placement
  refuses ambiguous lane forks/dead ends rather than guessing a junction exit.
  `--obstacle-blueprint static.prop.<name>` selects another available prop.
  This is an explicitly synthetic obstacle, removed at cleanup; no saved map
  assets or source annotations are changed. TM's response to a generic prop
  is not guaranteed: inspect collision/distance results rather than assuming
  that TM recognizes it as a stopped vehicle.
- `--capture-sensors`: capture without automatically building panels.
- `--capture-interval 0.5 --capture-start 0.5`: sample at 2 Hz after a brief
  sensor startup period. Driving continues throughout. The last three physics
  ticks are excluded from sampling to leave room for the sensor pipeline.
- `--capture-width 640 --capture-height 360`: raw camera dimensions.
  Rendering must be enabled, and `--ego-mode tm` is required for this workflow.

Each tick waits for all seven sensors before advancing, then saves only selected
samples. Capture never calls `world.tick()` or sets vehicle transforms. Exact
sensor frame IDs and timestamps are checked again before panel generation.
The GIF uses capture timing, not wall-clock rendering time. Camera placement
is the same approximate nuScenes-style rig used by the comparison workflow.
LiDAR is CARLA's standard ray-cast sensor: this feature does **not** introduce
a rain-scattering sensor model or change tire friction. Rain is a weather and
rendering case, not a calibrated wet-road braking experiment.

Rebuild panels offline without rerunning simulation:

```bat
python -m carla_reconstruction.visualization.sim_panels ^
  --capture-dir carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\tm_rain\capture ^
  --output carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\tm_rain\sim_panels_v2 ^
  --cell-width 480 --window-m 60
```

## Recorded-pose comparison workflow

This optional workflow adapts the old `nuscenes2xodr/build_aligned_topdown.py`
and `build_aligned_full.py` layouts to the manifest-based reconstruction.
It does not modify the old scripts, map-generation/import pipeline, or
CARLA-only/SUMO closed-loop controllers.

## Inputs and layout

The capture reads a prepared `scene_manifest.json`, its trajectory metadata,
the original nuScenes dataset, and the imported CARLA `map.runtime_name`.
The dataset must contain its JSON tables, six camera images and `LIDAR_TOP`
keyframes; a manifest alone does not contain those sensor recordings.

Each camera group is arranged as:

| Front left | Front | Front right |
|---|---|---|
| Back left | Back | Back right |

REAL cameras appear above SIM cameras. The top-down layout puts a square
schematic to their right. The full layout inserts REAL LiDAR above SIM LiDAR
between the camera block and the schematic.

The schematic is ego-centred and north-up. Grey areas are driving lanes sampled
from the **loaded CARLA waypoint graph**, including curved roads and junctions.
Buildings, trees and static props come from the manifest; boxes show captured
actor poses. Ego is red, other motor vehicles blue, pedestrians green, and
cycles magenta. Traffic-control markers indicate locations, not signal states.
This is not an aerial camera image or a rendering of painted lane markings.
The camera images are the actual render of the loaded Decorated map.

## First capture and build

Use Anaconda Prompt at the repository root, with CARLA running and no other
simulation controller active. Capture loads the requested map and replaces the
current simulation session. No persistent map assets are changed.

```bat
conda activate carla_0915
python -m pip install -r carla_reconstruction\requirements-visualization.txt
python carla_reconstruction\runtime\capture_reconstruction.py --scene 0103
python carla_reconstruction\tools\build_aligned_topdown.py --scene 0103
python carla_reconstruction\tools\build_aligned_full.py --scene 0103
```

Default output root:
`carla_reconstruction/generated/visualizations/<full-scene-name>/`.

- `capture/carla_cams/<channel>/<index>.png`: six camera recordings.
- `capture/lidar/<index>.npy`: SIM sensor-local xyz/intensity.
- `capture/capture_index.json`: source identity, sensor frame IDs and transforms.
- `capture/map_geometry.json`, `loaded_map.xodr`: captured map geometry snapshots.
- `aligned_topdown/` and `aligned_full/`: `frames/*.png`, `aligned.gif`,
  `panel_index.json`.

Both builders run without a CARLA server. Change `--scene 0103` to another
unambiguous scene ID/full name, or use `--manifest <path>`. `--scene all`
selects immediate `generated/*/scene_manifest.json` entries, not safety variants.
All selected scenes need captures before their panels can be built.

## Repeat runs and useful options

Outputs must be new or empty to prevent mixing frames from different runs.
For example, make a separate presentation capture and full panel set:

```bat
python carla_reconstruction\runtime\capture_reconstruction.py ^
  --scene 0103 ^
  --output carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\capture_v2

python carla_reconstruction\tools\build_aligned_full.py ^
  --scene 0103 ^
  --capture-dir carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\capture_v2 ^
  --output carla_reconstruction\generated\visualizations\boston-seaport_scene-0103\aligned_full_v2 ^
  --cell-width 480 --window-m 100
```

The same `--capture-dir` can be used by the top-down builder, with a different
`--output`. Custom capture/output directories require a single-scene selection.

Capture options:

- `--start-frame 0 --frame-stride 1 --max-frames 3`: an explicit short subset;
  omit `--max-frames` to capture through the final source keyframe.
- `--cam-width 640 --cam-height 360`: per-camera render size.
- `--no-lidar`: skip SIM LiDAR when only the top-down layout is needed;
  the full builder requires a LiDAR-enabled capture.
- `--host`, `--port`, `--timeout`, `--sensor-timeout`, `--warmup-ticks`: connection
  and sensor acquisition settings. Defaults target CARLA on localhost:2000.

Builder options:

- `--cell-width 320`: width of each camera tile in the final panel.
- `--window-m 80`: width/height of the square top-down and LiDAR areas in metres.
- `--fps 4`: optional presentation playback override. Without it, the GIF follows
  original sample timing (normally approximately 2 Hz; strided captures retain
  the larger gaps). GIF delays are quantized to its 10 ms time resolution.
- `--allow-partial`: explicitly build available frames from an interrupted
  capture; scene/sample identities must still match. Normally recapture instead.

All three commands accept `--dataroot <dataset-root>`, `--version v1.0-mini`,
and `--generated-root <prepared-manifests-root>`. Use `--help` for the full list.

## Alignment and interpretation

The workflow validates the scene, keyframe count, recorded ego positions/yaws,
and original sample-token chain before connecting to CARLA. Each keyframe is
held stationary while camera rendering settles; all six cameras and optional
LiDAR must share a CARLA frame ID. A missing sensor fails with a bounded timeout
instead of shifting later frames or silently dropping actors/keyframes.

PNG inputs are joined through `capture_index.json`, not filename ordering or
decoded GIF frames. Manifest/meta hashes detect changed sources. The index
records requested frame subsets explicitly; a successful subset is not a claim
that every source keyframe was captured.

This is an **open-loop reconstruction comparison**, not physically continuous
driving. Original nuScenes cameras and LiDAR have slightly different acquisition
timestamps. The CARLA six-camera rig uses approximate nuScenes-style mounting
and field of view, not exact per-camera calibration; proxy vehicle assets and
LiDAR parameters also differ from the real sensors. Side-by-side views are for
qualitative comparison, not pixel-level or pointwise reconstruction metrics.
Both LiDAR views use full sensor transforms, a north-up coordinate convention,
and height relative to ego, with the same -2 to +8 m colour scale.

TM/SUMO simulations, behavior variants and HGT prediction dashboards continue
to use their existing runtime commands independently of this workflow.
