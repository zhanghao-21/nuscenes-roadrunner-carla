# Data-seeded closed-loop simulation

This folder adds two interactive traffic modes without changing the existing
open-loop `replay_persistent.py` baseline:

- `run_tm_closed_loop.py`: CARLA-native Traffic Manager baseline.
- `run_sumo_hybrid.py`: SUMO background traffic with CARLA-authority ego and
  selected critical actors.

The same nuScenes manifest supplies initial poses, reference routes, actor
appearance times, categories, and recorded speeds. All commands below are for
**Anaconda Prompt** (`cmd.exe`) and start in the local repository. No command
writes to the OneDrive source project.

```bat
cd /d D:\nuscenes-roadrunner-carla
conda activate carla_0915
```

## Actor authority

The hybrid configuration records who controls every class of actor:

| Authority | Purpose |
|---|---|
| `carla_replay` | Fixed reference baseline; not interactive |
| `carla_tm` | CARLA Traffic Manager and CARLA vehicle physics |
| `carla_reference` | Pure-pursuit/headway controller and CARLA physics |
| `carla_external` | An external autonomous-driving client controls the ego |
| `sumo` | SUMO car-following/lane-changing background traffic |

`carla_reference` also accepts timed braking events. This is the initial hook
for generated safety-critical actors.

## 1. Install the decorated-map Traffic Manager data

The decorated level requires a same-named `.bin`, just as it requires a
same-named `.xodr`. Re-run the local finalizer after updating this repository:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\finalize_decorated_map.ps1 ^
  -Manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  -AllowCarlaWrite
```

The script copies
`Nusc_boston_seaport_0103.bin` to
`Nusc_boston_seaport_0103_Decorated.bin` without changing the base file.

The saved level must not contain ungrouped `NSRC_TrafficLight_*` actors. The
decorator now rejects visual traffic-light placement because those Blueprints
crash CARLA's world observer without functional junction groups/controllers.

Verify the Python environment, decorated map data, CARLA bridge, and SUMO
installation with a read-only check:

```bat
python carla_reconstruction\tools\check_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json
```

## 2. Run the CARLA Traffic Manager baseline

Start the source-built CARLA server, then run one of these commands.

Recorded ego plus interactive surrounding vehicles (partially closed-loop):

```bat
python carla_reconstruction\runtime\run_tm_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --ego-mode replay --replay-pedestrians
```

Traffic Manager ego plus interactive surrounding vehicles (fully interactive
baseline, but not an AV-under-test):

```bat
python carla_reconstruction\runtime\run_tm_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --ego-mode tm --replay-pedestrians
```

Use `--ego-mode external` when another CARLA client will find the actor whose
`role_name` is `hero` and apply its own controls. Only this mode represents the
closed loop of the eventual autonomous-driving system under test.

Stationary recorded vehicles remain static. Moving vehicles receive the
recorded path, average recorded speed, collision avoidance, and configurable
following distance. Add `--auto-lane-change` only when free lane-changing is
desired; it is disabled by default to preserve the recorded route.

## 3. Prepare SUMO from the same scene

The official CARLA converter is used, but the OpenDRIVE input is first copied
to the ignored local generated folder. This avoids the quoting bug caused by
spaces and the apostrophe in the OneDrive source path.

```bat
set "SUMO_HOME=C:\Traffic software\SUMO"
python carla_reconstruction\tools\prepare_sumo.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json
```

Outputs are written under:

```text
carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo
```

They include `network.net.xml`, `routes.rou.xml`, `scene.sumocfg`,
`route_report.json`, and `hybrid_config.json`. Tracks farther than 8 m from a
compatible connected SUMO lane are skipped instead of being silently assigned
to an incorrect road. Inspect `route_report.json` before an experiment.

For the current scene-0103 conversion, offline validation includes 41 vehicle
routes. Another 26 vehicle tracks are conservatively skipped because they are
outside the 8 m matching boundary; 46 pedestrian tracks are intentionally not
sent to SUMO.

Optional standalone validation:

```bat
"%SUMO_HOME%\bin\sumo.exe" ^
  -c carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\scene.sumocfg ^
  --no-step-log true --duration-log.statistics true
```

## 4. Run the SUMO-CARLA hybrid

Start the CARLA server first. The hybrid runner owns the synchronous tick in
both simulators.

```bat
python carla_reconstruction\runtime\run_sumo_hybrid.py ^
  --config carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\hybrid_config.json ^
  --sumo-gui --ego-mode reference
```

The `reference` baseline uses CARLA physics, pure-pursuit route tracking, and
reactive headway control. For an AV client, replace `--ego-mode reference` with
`--ego-mode external`. The ego
state is synchronized into SUMO every tick, so SUMO vehicles can react to it;
SUMO vehicle poses are synchronized back into CARLA for rendering and sensors.

Do not run another CARLA client that calls `world.tick()` at the same time.
External agents may apply controls, but the hybrid runner remains the only tick
owner.

## 5. Designate a CARLA-physics critical actor

Actor IDs are listed in `route_report.json`. Rebuild the SUMO files while
excluding the selected actor from SUMO:

```bat
python carla_reconstruction\tools\prepare_sumo.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --keep-existing-net ^
  --critical-actor ACTOR_ID_FROM_ROUTE_REPORT
```

The generated `hybrid_config.json` now gives that actor
`carla_reference` authority. It follows the recorded route with CARLA physics,
reacts to a leading vehicle using a configurable time headway, and can execute
scenario events.

## 6. Generate safety-critical braking variants

```bat
python carla_reconstruction\tools\generate_safety_variants.py ^
  --config carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\hybrid_config.json ^
  --critical-actor ACTOR_ID_FROM_ROUTE_REPORT ^
  --count 20
```

The generator uses deterministic Latin-hypercube samples of brake start time,
duration, intensity, desired-speed scale, and time headway. Run a variant by
passing its JSON file to `run_sumo_hybrid.py --config`.

## Run outputs and current boundary

Each TM or hybrid run writes `metrics.csv` and `summary.json` under the ignored
`carla_reconstruction\generated\closed_loop_runs` folder. The first metrics
are collision count/impulse, minimum ego-to-actor distance, ego speed, and
line-of-sight closing TTC. Every summary records the manifest, map, timing,
seed/config, and actor authorities.

The SUMO routes are currently **data-seeded, not calibrated**. They use the
recorded route and departure time; recorded initial/mean speeds are retained in
`route_report.json`, while SUMO inserts actors from rest to avoid invalid
departures at junctions. The route types use IDM car following and SL2015 lane
changing because CARLA's bridge enables SUMO sublane simulation. Driver-model calibration against nuScenes speed,
headway, acceleration, and lane-change observations is the next research
stage.
