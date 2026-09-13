# Scene-specific overrides

## Scene 1100: duplicate traffic-light mast

The installed `Nusc_singapore_hollandvillage_1100_Decorated` map originally
defined four same-controller signals for the adjacent approach roads 7 and 8.
They produced two visible masts only 3.265 m apart (each mast also had an exact
coincident duplicate). The scene-only correction retains signal `20001` at
the outer of those two positions and removes redundant physical signals
`20004`, `20005`, and `20006`. Road 7 now uses an OpenDRIVE `signalReference`
to `20001`; both roads keep their original signal station and lane validity.
The controller remains `30000`, with no new timing or phase settings.

Only the installed **Decorated** `.xodr` is changed. Road geometry, junction
connections, lane markings, Unreal meshes, the other approach's lights,
original/source map files, other scenes and shared build/runtime code are not
modified. All 1,134 sampled CARLA waypoints were identical before and after.

To preview or reapply after explicitly rebuilding/reimporting this map, run
from Anaconda Prompt at the repository root:

```bat
python carla_reconstruction\scene_overrides\deduplicate_1100_traffic_lights.py ^
  --output carla_reconstruction\generated\diagnostics\1100_lights_review
```

Review the generated corrected XODR and `report.json`. To install, use a **new**
output directory and append `--apply`. The command checks this scene's approach
geometry, signal IDs, duplicate attributes, lane validity and common controller;
an unexpected map or partly modified signal layout is rejected. It creates an
exclusive timestamped `.pre-1100-light-dedup-*.bak` beside the installed XODR,
verifies the backup, and installs atomically. Reapplying an already-corrected
file is a no-op. It never saves or edits the Unreal level.

Stop any running simulation script and reload scene-1100 (or stop and restart
Play) to recreate the traffic-light actors from the corrected XODR. Normal
replay/TM/hybrid commands are unchanged; no CARLA asset reimport is required.
An already-generated SUMO network is not rewritten by this scene-only patch.
To roll back, stop simulation and restore the exact backup named in
`report.json` to its recorded target, then reload the map.

## Scene 0103: Traffic Manager ego intersection route

The CARLA-only runner automatically applies a narrow ego-route correction for
`boston-seaport_scene-0103` with `--ego-mode tm` on
`Nusc_boston_seaport_0103_Decorated`. Use the same command; no map rebuild,
Unreal refresh, SUMO preparation, or new variant generation is needed:

```bat
python carla_reconstruction\runtime\run_tm_closed_loop.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --ego-mode tm --replay-pedestrians
```

The raw ego trajectory contains intersection-interior points that CARLA 0.9.15
Traffic Manager can associate with a competing left-turn connector. That wrong
exit is occupied by a recorded parked vehicle, consistent with TM's
blocked-junction-exit braking. Extending the run to 120 seconds did not release
the ego. This is separate from the lane markings and editor route splines.

`tm_ego_route_0103.py` instead supplies five unambiguous, non-junction exit-lane
anchors once, before the first simulation tick. They select the recorded
corridor `17 -> 69 -> 9 -> 56 -> 12 -> 80 -> 11`. The helper verifies the actual
loaded map (including with `--reuse-world`), original ego track, anchor geometry,
and directed waypoint connections. An incompatible scene-0103 map/track produces
a clear error rather than applying hard-coded roads to a different layout.

This changes only the ego's initial route coordinates. TM still controls its
physics, speed, following, lane-change behavior, and traffic-light/sign/collision
responses, including the existing CARLA perturbation profiles. Surrounding
vehicles, pedestrians, replay/external ego modes, other scenes, SUMO hybrid
control, OpenDRIVE, and lane markings are unchanged. Baselines and variants
receive the same route correction; no behavior parameters are overwritten.

The last anchor is road 11, lane -1, s=25 m, not a terminal road. After the anchors
are consumed, **normal TM route continuation resumes**; the helper does not
continually re-upload the path, force the ego through obstacles, or guarantee
replay-equivalent motion beyond the corrected corridor. `--duration 120` is
optional extra observation time, not the fix.

The console prints `Ego route correction: 0103_tm_ego_exit_lane_anchors_v1`.
`run_config.json` and `summary.json` record the applied correction, anchors,
road sequence, and continuation policy under `scene_ego_route`. For a diagnostic
rollback, append `--disable-0103-ego-route` to restore the original raw-path
behavior for that run; this may reproduce the original stop.

## Scene 0103: visible interior lane separators

This opt-in exception applies only to `Nusc_boston_seaport_0103`, leaving the
shared pipeline and other scenes unchanged. Three same-direction lane-divider
curves have outgoing `NIL` annotations, and twelve opposing-traffic road dividers
have no paint styles. The strict source renderer intentionally skips these;
this is why road-edge lines can be visible while interior separation is absent.

The override shows the three curves as **single dashed white** and the twelve
opposing-traffic dividers as **double solid yellow**. These paint styles are
**assumptions, not verified ground truth**; NIL does not prove that paint was lost.
Every curve uses its existing source nodes and must be a longitudinal boundary
shared by exactly two non-connector lane polygons with matching traffic directions
for its proposed color. No automatic interpolation across junctions is performed.
Existing painted edges (including solid white boundaries), source dataset files,
OpenDRIVE, routes, vehicle behavior, and normal patch clipping remain unchanged.

In Anaconda Prompt:

```bat
python carla_reconstruction\tools\prepare_0103_marking_override.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json
```

Save your work and close CARLA/Unreal, then apply this map's visual overlay only:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\launch_build.ps1 ^
  -Manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  -MarkingMapJson carla_reconstruction\generated\boston-seaport_scene-0103\visual_overrides\0103_assumed_divider_styles.json ^
  -MarkingsOnly -AllowCarlaWrite
```

The launcher saves the existing decorated level and marking assets under
`generated/checkpoints/markings_Nusc_boston_seaport_0103_<timestamp>` before
replacement. The override's `.audit.json` lists each changed annotation, source
node IDs, adjacent lane IDs, and original map hash. A normal markings rebuild
without `-MarkingMapJson` restores strict source-only paint; repeat the command
above to retain the scene-specific improvement. Open the **Decorated** level to
see it; the original imported base level is intentionally untouched.

## Scene 0757: optional divider-style improvement

This opt-in exception applies only to `Nusc_boston_seaport_0757`. The normal
pipeline remains unchanged. Five existing nuScenes road-divider curves without
paint metadata are displayed as double-solid yellow. This style is a **visual
assumption, not verified ground truth**. Source coordinates, existing white
lane-divider styles, NIL annotations, OpenDRIVE, and traffic behavior are unchanged.

In Anaconda Prompt, prepare the separate override map:

```bat
python carla_reconstruction\tools\prepare_0757_marking_override.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0757\scene_manifest.json
```

Save your work and close CARLA/Unreal, then apply only this map's markings:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\launch_build.ps1 ^
  -Manifest carla_reconstruction\generated\boston-seaport_scene-0757\scene_manifest.json ^
  -MarkingMapJson carla_reconstruction\generated\boston-seaport_scene-0757\visual_overrides\0757_assumed_divider_styles.json ^
  -MarkingsOnly -AllowCarlaWrite
```

The launcher saves a rollback checkpoint before replacing the overlay. The
override's `.audit.json` identifies the five curves and the original data hash.
Do not use this override map for other scenes. A normal rebuild without
`-MarkingMapJson` returns scene 0757 to strict source-annotated markings (three
white strips); repeat the command above to retain this visual exception.
Existing source-road-width discrepancies are not repaired by this override.

## Scene 0103: refresh stale editor intersection routes

The installed `Nusc_boston_seaport_0103_Decorated` OpenDRIVE already passes all
41 junction-entrance and 41 junction-exit transitions in CARLA's waypoint API.
The red `RoutePlanner` splines saved inside the Unreal level are a separate
editor representation: an older import can leave them disconnected even though
the current runtime waypoint graph is connected. This helper adds 41 missing
junction splines sampled from that existing graph. It preserves all 31 original
route actors and their exact spline points, all 31 vehicle spawn points, and
the existing `OpenDriveActor`. No native route regeneration is used.

The additional actors are editor-only, hidden in game, collision-disabled, and
have overlap events disabled. Red linear splines show each connector, with short
bridges where old ordinary routes stopped up to one 2 m sample before the
junction. The helper checks those bridges against the declared incoming/outgoing
roads; it does not invent road connections. OpenDRIVE, lane markings, runtime
traffic behavior, other scenes, and the shared pipeline remain unchanged.

Use the standalone `refresh_0103_editor_routes.py` helper, not a full map or
markings rebuild. **Save your work and close all Unreal/CARLA processes first.**
Before applying, make a separate rollback copy of the installed
`C:\carla\Unreal\CarlaUE4\Content\Nusc_boston_seaport_0103\Maps\Nusc_boston_seaport_0103\Nusc_boston_seaport_0103_Decorated.umap`.
The helper does **not** create that backup automatically.

From the repository root in the `carla_0915` Anaconda Prompt, prepare a validated
reference using the installed XODR:

```bat
set "NUSC_0103_ROUTES_REFERENCE="
python carla_reconstruction\scene_overrides\refresh_0103_editor_routes.py ^
  --prepare-reference carla_reconstruction\generated\topology\0103_editor_routes_reference.json ^
  --carla-root C:\carla
```

Then run an audit without saving the map (adjust Unreal/CARLA paths if needed):

```bat
set "NUSC_0103_ROUTES_REFERENCE=%CD%\carla_reconstruction\generated\topology\0103_editor_routes_reference.json"
set "NUSC_0103_ROUTES_REPORT=%CD%\carla_reconstruction\generated\topology\0103_editor_routes_audit.json"
set "NUSC_0103_ROUTES_APPLY=0"
"C:\UnrealEngine\Engine\Binaries\Win64\UE4Editor-Cmd.exe" ^
  "C:\carla\Unreal\CarlaUE4\CarlaUE4.uproject" ^
  -ExecutePythonScript="%CD%\carla_reconstruction\scene_overrides\refresh_0103_editor_routes.py" ^
  -unattended -nop4
```

Inspect `missing_connector_routes_before` and `existing_spawn_points` in the
audit JSON. After the audit process exits, and with the backup in place, set
`NUSC_0103_ROUTES_APPLY=1`, change `NUSC_0103_ROUTES_REPORT` to a separate apply
report path, and rerun the same `UE4Editor-Cmd.exe` command. Existing spawn points
are preserved, not regenerated. The helper refuses to save if any connector is
missing, a bridge is ambiguous or too long, original route points changed, new
actors could interact with traffic, the XODR changed, or original map actors and
marking meshes/materials are not preserved. Repeating the operation updates only
its `NSRC_0103_EditorJunction_R*` actors instead of adding duplicates.

Confirm `applied: true` and an empty `missing_connector_routes_after` in the
apply report. Then set `NUSC_0103_ROUTES_APPLY=0`, use a new report path, and run
the command again in a fresh process to verify the saved splines reload with an
empty `missing_connector_routes_before`. Reopen the **Decorated** level to view
the connected red editor lines. An audit alone does not apply a fix. Clear
`NUSC_0103_ROUTES_REFERENCE`, `NUSC_0103_ROUTES_REPORT`, and
`NUSC_0103_ROUTES_APPLY` afterward using `set "VARIABLE_NAME="`.
