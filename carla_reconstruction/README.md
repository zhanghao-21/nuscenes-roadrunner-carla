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

```matlab
manifest = "D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json";
project = "D:\nuscenes-roadrunner-carla\nuscenes_test1";
out = "D:\nuscenes-roadrunner-carla\carla_reconstruction\generated\fbx";
fbx = export_roadrunner_base(manifest, project, out);
```

The exporter defaults to the installed RoadRunner R2024a location:
`C:\Traffic software\RoadRunner R2024a\bin\win64`. Pass a fourth argument only
if that installation moves. It also recreates the ignored `Assets`, `Exports`,
`Scenes`, and `Scenarios` folders in the local RoadRunner project when needed.

Then stage the CARLA import package locally:

```bat
python carla_reconstruction\tools\prepare_import_package.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --fbx carla_reconstruction\generated\fbx\Nusc_boston_seaport_0103.fbx ^
  --output-root carla_reconstruction\generated\import
```

Review the staged package. Copy it to `C:\carla\Import` only when ready, then
run CARLA's normal `make import` process. For this example the imported source
level is
`/Game/Nusc_boston_seaport_0103/Maps/Nusc_boston_seaport_0103/Nusc_boston_seaport_0103`.
The staging tool intentionally refuses to write directly into the CARLA
repository.

## 4. Decorate a copied Unreal level

Use Content Browser paths, not filesystem paths. The source level is the map
created by `make import`; the target must be a new asset so the imported base is
preserved.

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File carla_reconstruction\launch_build.ps1 ^
  -Manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  -AllowCarlaWrite
```

The source and target Content Browser paths come from the manifest; they can
still be overridden with `-SourceLevel` and `-TargetLevel`.

The Unreal script removes only actors whose labels start with `NSRC_`, then
rebuilds the environment idempotently. Before Unreal starts, the launcher reads
the same OpenDRIVE `roadMark` records used by CARLA routing and generates a thin
visual marking mesh (solid, broken, double, white, and yellow). Unreal imports
that mesh at the map origin, lets the OBJ importer apply the same Y-axis
conversion used by the RoadRunner FBX, assigns CARLA lane-paint materials, and replaces
RoadRunner's white `BadDefault` road override with CARLA asphalt on the copied
level. The generated OBJ/MTL and statistics are under
`carla_reconstruction\generated\markings`; 2 cm paint elevation avoids
z-fighting without changing vehicle collision.

The CARLA-staged OpenDRIVE copy also remaps any junction ID that collides with
a road ID. CARLA 0.9.15 otherwise interprets that junction successor as an
ordinary road and `Waypoint.next()` stops at the intersection entrance. The
OneDrive source XODR is never modified. When the finalizer repairs an existing
import, it backs up the old XODR and stale Traffic Manager `.bin` files; without
a cache, Traffic Manager builds its graph from the corrected map on first use.

The nuScenes converter represents each lane as a separate OpenDRIVE road, so
the two roads beside a shared physical divider can describe the same paint
twice. The generator removes parallel boundary segments within 0.30 m before
creating the paint ribbons. In addition, its CARLA visual default renders a
nuScenes/OpenDRIVE `broken broken` record as one centered dashed stripe: the
source metadata supplies no surveyed separation, and expanding it with a
guessed offset looks duplicated in Unreal. This does not modify the OpenDRIVE
file used for routing. Pass `--preserve-double-dashed` directly to
`tools\generate_lane_markings.py` when a literal two-stripe visualization is
required. The generated statistics report
`duplicate_base_segments_removed`, `double_broken_definitions_collapsed`, and
`rendered_road_mark_definitions`.

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
  -Manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  -AllowCarlaWrite
```

The finalizer makes a one-time backup of the package registry before updating
it. Build pedestrian navigation if walkers are required, then package the map
using CARLA's normal tooling.

The waypoint graph can be checked offline without starting a CARLA server:

```bat
python carla_reconstruction\tools\validate_waypoint_topology.py ^
  --xodr C:\carla\Unreal\CarlaUE4\Content\Nusc_boston_seaport_0103\Maps\Nusc_boston_seaport_0103\OpenDrive\Nusc_boston_seaport_0103_Decorated.xodr
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
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --cameras --record
```

The default decorated map path also comes from the manifest. This loads the
persistent map with `client.load_world`, teleports the ego and
annotated road users along the recorded 2 Hz trajectories in synchronous mode,
draws history/future trails, and can save the approximate six-camera rig and a
chase camera. By default, frames use the existing visualization layout under
`replay_geo/results/<scene>/carla_cams` and `carla_frames`.

The existing real/sim/top-down panel and GIF tools can consume them directly:

```bat
set "NUSCENES_RR_OUTPUT=%RR%"
set "NUSCENES_DATAROOT=D:\OneDrive - Texas A&M University\Wu, Keshu's files - nuscenes_1\v1.0-mini"
python replay_geo\build_aligned_geo.py --scene 0103
python replay_geo\make_gifs.py --scene 0103
```

## 6. Interactive traffic and safety-critical variants

The replay above remains the open-loop reference. A separate closed-loop
framework now provides a CARLA Traffic Manager baseline, SUMO background
traffic, CARLA-physics ego/critical actors, deterministic critical-braking
variants, and safety metrics. Continue with the Anaconda Prompt instructions
in [closed_loop/README.md](closed_loop/README.md).

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
