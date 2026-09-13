# Persistent nuScenes reconstruction in source-built CARLA

This folder adds a second CARLA workflow to the repository. The existing
`replay_geo` workflow creates an OpenDRIVE world at runtime. This workflow
imports a real map into the source-built Unreal project, decorates a persistent
`.umap`, and then replays the recorded nuScenes actors on that saved map.

All source code and generated staging artifacts live in
`D:\nuscenes-roadrunner-carla`. The preparation tools may read the original
OneDrive data, but never write to it. The Unreal build launcher requires an
explicit `-AllowCarlaWrite` switch before it can create a level under
`C:\carla`.

All Windows command examples below are written for **Anaconda Prompt**
(`cmd.exe`), not PowerShell. Start from the repository root:

```bat
cd /d D:\nuscenes-roadrunner-carla
```

## Data used

- Roads, lane topology, crosswalks, stop lines, and traffic-light landmarks:
  the existing nuScenes-to-OpenDRIVE conversion.
- Buildings: OSM footprints already decomposed and road-cleared by
  `nuscenes_ground_buildings.m`; the resulting boxes are decoded from the
  RoadRunner `.rrhd` file.
- Trees: explicit OSM tree nodes plus deterministic samples inside OSM parks,
  grass, gardens, forests, and recreation areas. OSM vegetation coverage is
  sparse, so these are deliberately conservative.
- Traffic lights: nuScenes traffic-light records carried through OpenDRIVE.
- Traffic signs: nuScenes has no general traffic-sign layer. Only supported OSM
  stop, yield, and speed-sign nodes are added, with provenance and confidence
  stored in the manifest.
- Other traffic props: raw nuScenes annotations provide barriers, traffic
  cones, debris, and bicycle racks. They are deduplicated by instance and added
  as persistent actors when a matching CARLA asset exists. Bicycle-rack records
  remain in the manifest but are skipped until an honest rack asset is supplied.
- Dynamic actors: the existing `ego_trajectory_local` and `agent_frames`
  metadata, including real nuScenes bounding-box dimensions and categories.

## 1. Verify the installed tools (read-only)

```bat
python carla_reconstruction\tools\check_install.py
```

The checked installation is CARLA 0.9.15.2 at `C:\carla` with Unreal Engine
4.26 at `C:\UnrealEngine`. `PythonScriptPlugin`, `EditorScriptingUtilities`,
and `CarlaTools` must be enabled.

## 2. Prepare one scene manifest

```bat
set "RR=D:\OneDrive - Texas A&M University\Wu, Keshu's files - nuscenes_1\output_roadrunner"
python carla_reconstruction\prepare_scene.py --scene 0103 --source-root "%RR%"
python carla_reconstruction\preview_manifest.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json
```

The raw dataset is auto-detected at the `v1.0-mini` sibling of
`output_roadrunner`; use `--nuscenes-dataroot` if it is elsewhere.

After validating one scene, prepare every discovered scene with:

```bat
python carla_reconstruction\prepare_scene.py --scene all --source-root "%RR%"
```

The manifest is the versioned boundary between ordinary Python and Unreal
Python. It records coordinates in patch-local OpenDRIVE metres and explicitly
states the CARLA/Unreal conversion (`x`, `-y`, `-yaw`, metres to centimetres).

## 3. Export and import the base road map

CARLA's persistent-map importer needs a same-named RoadRunner FBX and OpenDRIVE
file. The RoadRunner export can be driven from MATLAB instead of clicking
through the editor. The exporter uses **CARLA Filmbox**, not generic Filmbox,
and produces the FBX plus `.rrdata.xml` material metadata with meshes split by
semantic class. The local staging command requires that metadata and keeps the
original nuScenes-derived OpenDRIVE as the runtime authority.

The export block below is **MATLAB code, not an Anaconda Prompt command**.
Run it from the MATLAB Command Window (or save it as a `.m` script). Start
MATLAB, then paste the following example for scene-1094:

```matlab
addpath("D:\nuscenes-roadrunner-carla\carla_reconstruction");

manifest = "D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\singapore-onenorth_scene-0061\scene_manifest.json";
project = "D:\nuscenes-roadrunner-carla\nuscenes_test1";
out = "D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\fbx";
fbx = export_roadrunner_base(manifest, project, out);
```

Wait until MATLAB prints both `Exported CARLA road geometry` and `Exported
CARLA material metadata`. For scene-1094, the required outputs are
`Nusc_singapore_hollandvillage_1094.fbx` and
`Nusc_singapore_hollandvillage_1094.rrdata.xml` in `generated\fbx`.

As an alternative, the same MATLAB export can be launched from an Anaconda
Prompt at the repository root. This works because `matlab` is on this machine's
`PATH`; the quoted text after `-batch` is still evaluated by MATLAB:

```bat
matlab -batch "addpath('D:\nuscenes-roadrunner-carla\carla_reconstruction'); manifest='D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\singapore-hollandvillage_scene-1094\scene_manifest.json'; project='D:\nuscenes-roadrunner-carla\nuscenes_test1'; out='D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\fbx'; export_roadrunner_base(manifest, project, out);"
```

The exporter defaults to the installed RoadRunner R2024a location:
`C:\Traffic software\RoadRunner R2024a\bin\win64`. Pass a fourth argument only
if that installation moves. It also recreates the ignored `Assets`, `Exports`,
`Scenes`, and `Scenarios` folders in the local RoadRunner project when needed.

`prepare_scene.py --scene all` creates manifests only; it does not export FBX
files. Export every new scene separately, always pairing a manifest with the
FBX and `.rrdata.xml` generated from that same manifest.

After one scene's FBX export succeeds, return to the Anaconda Prompt at
`D:\nuscenes-roadrunner-carla` and stage that scene's CARLA import package.
The staging tool accepts one manifest/FBX pair per invocation. Example for
scene-1094:

```bat
python carla_reconstruction\tools\prepare_import_package.py ^
  --manifest carla_reconstruction\generated\singapore-onenorth_scene-0061\scene_manifest.json ^
  --fbx carla_reconstruction\generated\fbx\Nusc_singapore_hollandvillage_1100.fbx ^
  --output-root carla_reconstruction\generated\import
```

Repeat the MATLAB export and Python staging command for each new scene, changing
both paths together. Reusing the scene-0103 command only rebuilds scene-0103;
it does not discover the other manifests. The common `generated\import` output
root is intentional: each map is placed in its own uniquely named package
folder, so separately staged scenes do not overwrite one another.

Review the staged packages. Copy only the new package folders to
`C:\carla\Import` when ready, then run CARLA's normal `make import` process.
For scene-1094 the expected imported source level is
`/Game/Nusc_singapore_hollandvillage_1094/Maps/Nusc_singapore_hollandvillage_1094/Nusc_singapore_hollandvillage_1094`.
The staging tool intentionally refuses to write directly into the CARLA
repository. After import, run the decoration and finalization commands in the
next section once per manifest.

## 4. Decorate a copied Unreal level

Use Content Browser paths, not filesystem paths. The source level is the map
created by `make import`; the target must be a new asset so the imported base is
preserved.

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\launch_build.ps1 ^
  -Manifest carla_reconstruction\generated\singapore-onenorth_scene-0061\scene_manifest.json ^
  -AllowCarlaWrite
```

The source and target Content Browser paths come from the manifest; they can
still be overridden with `-SourceLevel` and `-TargetLevel`.

The full Unreal build removes only actors whose labels start with `NSRC_`, then
rebuilds the environment idempotently. **Visual markings now default to the
original nuScenes map-expansion divider geometry**, not OpenDRIVE boundaries
estimated from average lane widths. The generator reads `source.nuscenes_dataroot`
and `source.meta` from the manifest, subtracts the scene origin, and clips source
divider edges to the original scene patch. It preserves each node's outgoing-edge
style (including solid/dashed, white/yellow, double stripes, and zigzags), keeps
dash phase across source vertices and patch clipping, and removes only exactly
coincident duplicate edges. Nearby distinct boundaries are not merged.

`NIL`, missing, and unsupported styles are **not painted**. In particular,
nuScenes `road_divider` records often have geometry but no paint-style metadata;
they are reported as missing, not assumed to be yellow lines. This is not a
complete reconstruction of curb, stop-line, crosswalk, or arrow markings.
The old preparation's synthetic `none -> broken/solid` defaults remain in the
driving OpenDRIVE file, but are no longer used to paint this visual overlay.
Road geometry, topology, lane-change permissions, SUMO routes, and TM caches
are unchanged. Visual and OpenDRIVE lane-invasion semantics may therefore
differ; this correction is not a driving-network or lane-invasion-model repair.

Unreal imports the mesh at the map origin, verifies its bounds after the OBJ
importer's Y-axis conversion, and assigns CARLA lane-paint materials. A full
build also replaces RoadRunner's `BadDefault` road material with CARLA asphalt.
Generated OBJ/MTL files and the JSON audit are under
`carla_reconstruction\generated\markings`. Paint is non-colliding and raised
2 cm above the flat road. Elevated/banked OpenDRIVE maps fail explicitly until
surface-height projection is implemented.

To update **only the markings on an existing decorated map**, close the CARLA
server/editor and run this in Anaconda Prompt (CMD):

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\launch_build.ps1 ^
  -Manifest carla_reconstruction\generated\singapore-hollandvillage_scene-1094\scene_manifest.json ^
  -MarkingsOnly -AllowCarlaWrite
```

This backs up the decorated `.umap`, generated marking files, and marking assets
under `generated\checkpoints`, then replaces only `NSRC_LaneMarkings_*` actors.
Other actor identities, transforms, and material assignments are checked before
saving. It does not rebuild buildings/signs or change the imported base map.
**Do not rerun RoadRunner export, `make import`, or `prepare_sumo.py` for this
visual-only update.** Restart the usual simulation after the rebuild succeeds.
If PowerShell selects a different Python, add `-Python "C:\path\to\python.exe"`.

To generate/review the overlay without opening Unreal:

```bat
python carla_reconstruction\tools\generate_lane_markings.py ^
  --manifest carla_reconstruction\generated\singapore-hollandvillage_scene-1077\scene_manifest.json ^
  --output carla_reconstruction\generated\markings\Nusc_singapore_hollandvillage_1077_LaneMarkings.obj
```

The JSON audit records source hashes, omitted styles, preserved doubles, and
alignment against the sampled OpenDRIVE driving-lane envelope (25 cm tolerance).
An alignment warning is not silently fixed by moving source points; inspect the
reported locations. This check is not an FBX-surface or camera-image validation.
Dash length/gap (3 m / 6 m), stripe width (0.13 m), and double-stripe centerline
separation (0.24 m) remain explicit **visual assumptions**, not surveyed sizes.
Override these with `--dash-length`, `--dash-gap`, `--stripe-width`, and
`--double-separation` when independently measured values are available.
Zigzag dimensions are also identified as assumptions in the JSON report.

Missing map data causes an error instead of a silent fallback. Supply
`--map-json PATH` to the generator, or `-MarkingMapJson PATH` to the launcher,
to point to the correct map-expansion JSON. A valid patch with no known paint
styles produces an empty overlay; a markings-only rebuild removes the old
overlay in that case. Always check its report before applying.

The CARLA-staged OpenDRIVE copy also remaps any junction ID that collides with
a road ID. CARLA 0.9.15 otherwise interprets that junction successor as an
ordinary road and `Waypoint.next()` stops at the intersection entrance. The
OneDrive source XODR is never modified. When the finalizer repairs an existing
import, it backs up the old XODR and stale Traffic Manager `.bin` files; without
a cache, Traffic Manager builds its graph from the corrected map on first use.

For comparison or compatibility, explicitly select the old approximation with
`--source opendrive` in the generator or `-MarkingSource opendrive` in the
launcher. Only this legacy mode merges nearby boundaries within 0.30 m and
collapses double-dashed markings by default (`--preserve-double-dashed` keeps
the pair). The nuScenes source never collapses explicit double styles.

For an exact rollback, close CARLA/Unreal and restore the backed-up decorated
`.umap` to its original Maps folder and the files in `marking_assets` to the
map's `Static\Marking\<asset_name>` folder, along with the OBJ/MTL/JSON if desired.
Restore both the map and the mesh asset: the map references the marking asset
by path, so restoring the `.umap` alone does not restore the old paint.

Close any CARLA server or Unreal Editor instance that has the target decorated
map loaded before running the launcher. The script checks the `.umap` lock
before it starts and requires a success receipt from Unreal, so a Python error
or failed level save cannot be mistaken for a successful build.

Traffic-light Blueprint placement is disabled because functional CARLA lights
require trigger volumes, controllers, and junction groups. The launcher rejects
`-PlaceTrafficLights`: an ungrouped `BP_TrafficLightNew` crashes CARLA's
`WorldObserver` when a Python client connects. OpenDRIVE signal records remain
authoritative until functional group generation is implemented.

After inspecting the result in Unreal, register the copied level and its
same-named OpenDRIVE file in the imported package:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\finalize_decorated_map.ps1 ^
  -Manifest carla_reconstruction\generated\singapore-hollandvillage_scene-1077\scene_manifest.json ^
  -AllowCarlaWrite
```

The finalizer makes a one-time backup of the package registry before updating
it. Build pedestrian navigation if walkers are required, then package the map
using CARLA's normal tooling.

The waypoint graph can be checked offline without starting a CARLA server:

```bat
python carla_reconstruction\tools\validate_waypoint_topology.py ^
  --xodr C:\carla\Unreal\CarlaUE4\Content\Nusc_boston_seaport_0553\Maps\Nusc_boston_seaport_0553\OpenDrive\Nusc_boston_seaport_0553_Decorated.xodr
```

For this scene, a correct report contains 41 successful intersection entrances,
41 successful connector exits, and 774 junction waypoints at 1 m spacing.

Waypoints are runtime routing data and are not visible merely by opening the
level in Unreal Editor. Start **Play** in the editor (or start a CARLA server),
then draw the actual graph with:

```bat
python carla_reconstruction\runtime\draw_waypoint_topology.py --life-time 120
```

Cyan indicates ordinary lanes, orange indicates junction connector lanes, and
red indicates a true dead end at the edge of the reconstructed patch. The red
vehicle trajectories and wireframe actor boxes from the replay visualization
are separate overlays; they are not CARLA waypoints.

## 5. Replay and visualize trajectories

Start the source-built CARLA server with the decorated map available, then use
the CARLA 0.9.15 Python environment:

```bat
conda activate carla_0915
python carla_reconstruction\runtime\replay_persistent.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0553\scene_manifest.json ^
  --cameras --record
```

The default decorated map path also comes from the manifest. This loads the
persistent map with `client.load_world`, teleports the ego and
annotated road users along the recorded 2 Hz trajectories in synchronous mode,
draws history/future trails, and can save the approximate six-camera rig and a
chase camera. By default, frames use the existing visualization layout under
`replay_geo/results/<scene>/carla_cams` and `carla_frames`.

### 5.1. Aligned REAL/SIM reconstruction comparison (new pipeline)

Use the following manifest-based tools for the layouts previously produced by
`nuscenes2xodr/build_aligned_topdown.py` and `build_aligned_full.py`. They capture
the **imported Decorated map**, not a new bare OpenDRIVE world, and work with
any prepared scene whose map has been imported. The capture is separate from
`replay_persistent.py`; its indexed images are not interchangeable with legacy
replay folders or GIFs.

Run these commands in **Anaconda Prompt**, from the repository root. Start
CARLA first (press **Play** if using Unreal Editor), and stop any other replay,
Traffic Manager or SUMO controller before capturing:

```bat
conda activate carla_0915
python -m pip install -r carla_reconstruction\requirements-visualization.txt

python carla_reconstruction\runtime\capture_reconstruction.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json

python carla_reconstruction\tools\build_aligned_topdown.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json

python carla_reconstruction\tools\build_aligned_full.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json
```

The first command loads the manifest's map and captures six cameras plus LiDAR
at every recorded keyframe, including the final one. It temporarily takes over
the simulation; it does **not** edit or reimport map assets. The two builders
run offline afterward:

- `build_aligned_topdown.py`: REAL/SIM six-camera comparison + schematic top-down.
- `build_aligned_full.py`: the same, with REAL/SIM LiDAR panels in the middle.

Outputs are under
`carla_reconstruction/generated/visualizations/boston-seaport_scene-0103/`:
`capture/`, `aligned_topdown/` and `aligned_full/`. Each aligned folder contains
`aligned.gif`, individual `frames/*.png`, and an alignment audit `panel_index.json`.
Open the PNGs for full-resolution slide images and the GIF for animation.

For another scene, replace the manifest path, or use `--scene 0553` on each
command. `--scene all` processes all prepared manifests (every corresponding
map must already be imported for capture). Source images and point clouds are
read from `source.nuscenes_dataroot` in the manifest; `--dataroot` overrides that
location when the dataset has moved.

This comparison is **recorded-pose replay**, not a TM/SUMO closed-loop run or
HGT risk visualization. The camera rig is approximate; the top-down view shows
lane areas, environment objects and actor poses, not the actual painted lane
markings. GIF playback follows the source timestamps unless `--fps` is set.
Existing nonempty output folders are never overwritten: use a new `--output`
for another capture and pass that folder to the builders with `--capture-dir`.
See [visualization options and repeat-run examples](visualization/README.md).

## 6. Interactive traffic and safety-critical variants

The replay above remains the open-loop reference. A separate closed-loop
framework now provides two controller-pure safety-variant pipelines. In the
hybrid pipeline, SUMO controls every classified moving surrounding vehicle,
CARLA controls the ego, and recorded fixed vehicles remain visible in CARLA. In
the CARLA-only pipeline, every surrounding vehicle remains in CARLA; Traffic
Manager controls the movers and fixed/single-observation tracks remain static.
CARLA-only perturbations now have separate `--target ego` and
`--target surrounding` experiment families. Both use a Traffic Manager ego and
Traffic Manager moving surrounding vehicles; only the selected group's behavior
parameters change, and each family has its own matched baseline.
The CARLA-only generator now defaults to `--profile aggressive`; use
`--profile stress` for 100% selected-driver vehicle/light/sign hazard-ignore
settings, or `--profile mild` for the previous mild ranges. Stronger profiles
use higher speed targets, shorter following gaps and per-driver variation.
Physical collisions remain enabled and matched baselines are unchanged. New
files are saved under `carla_safety_variants/<target>/<profile>/`, preserving
old generated variants. Regenerate and run the new profile-folder config;
old files do not become aggressive automatically. See
[generation commands and parameter ranges](closed_loop/README.md#6-generate-separate-carla-only-ego-and-surrounding-perturbations).
For scene 0103 on its matching Decorated map, the CARLA-only TM ego automatically
uses a [scene-specific intersection route correction](scene_overrides/README.md#scene-0103-traffic-manager-ego-intersection-route)
to avoid a wrong-turn/blocked-exit stop. The existing run command is unchanged;
no map or SUMO rebuild is required, and surrounding behavior, safety checks,
other scenes, and the hybrid runtime are unaffected.

The CARLA-only closed-loop runner also supports the same optional HGT
prediction-risk display as the hybrid: add `--prediction-risk --draw-predictions`
to show the inverse-TTC dashboard and predicted trajectories, or
`--no-prediction-risk` to disable both. Dashboard and trajectory drawing can be
toggled independently; only recorded moving surrounding vehicles are included.
No SUMO preparation is needed. See
[CARLA-only HGT options and examples](closed_loop/README.md#optional-carla-only-hgt-prediction-and-dashboard).

The generators create a matched baseline plus deterministic behavior variants,
without selecting a special critical actor, and both runtimes write safety
metrics and application audits. SUMO route preparation now matches through internal
junction connectors, rejects opposite-direction/backward matches, preserves
the recorded edge sequence as a route prefix, and adds a seeded valid
continuation to a road end (or a duration-sized cyclic tail). It also audits
every mover's insert/mirror/complete lifecycle. A moving SUMO actor whose source
track ends in a persistent stationary tail is released from speed replay and
continues autonomously on its already prepared route; releasing the speed
target does not replace that route or immediately choose another turn. Runtime
extension is proactive near an unintended route tail and appends beyond the
existing route with a complete path to a genuine boundary or reusable cycle;
preparation and runtime both reject acyclic interior dead ends. Guarded
recovery first preserves the recorded immediate turn
and replaces only its generated suffix with a viable route to a map boundary
or cycle, following any forced prefix until a usable branch and never retrying
a suffix branch that already failed. If recorded-speed replay is active during
an unguarded persistent stop, recovery releases only that speed authority
first; any route mutation requires another full autonomous persistence window.
It acts while scheduled stops, close leaders, and red/yellow signals remain
under normal SUMO control. An
immediate different turn is only a last resort, installs a complete viable
continuation, and retries a distinct branch if that choice remains stuck.
Interior conversion stubs are rejected. A true map-boundary road end with no
valid outgoing connection is never extended.

Recorded parked vehicles remain at their true CARLA roadside poses, but their
CARLA-to-SUMO spawn notifications are filtered. SUMO therefore controls only
the classified moving background vehicles and cannot mistake a lane-mapped
shadow of an off-road parked car for a stopped leader. The CARLA ego remains
mirrored into SUMO because it is a dynamic participant. Generate hybrid
variants with `tools\generate_sumo_safety_variants.py` and CARLA-only variants
with `tools\generate_carla_safety_variants.py`.
Continue with the Anaconda Prompt instructions in
[closed_loop/README.md](closed_loop/README.md).

## Current validation boundary

The offline manifest generation and parser tests can run without Unreal. The
first Unreal execution should be done on one small scene (scene-0103) and
visually checked for coordinate alignment, building scale, collisions,
semantic labels, traffic-light triggers, and camera agreement before processing
all ten scenes.

Run the offline tests from the repository root with:

```bat
python -m unittest discover -s carla_reconstruction\tests -t carla_reconstruction -v
```
