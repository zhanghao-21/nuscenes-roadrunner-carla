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
through the editor. This imports only the OpenDRIVE road base; buildings and
other environment actors are added once by Unreal Python.

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
rebuilds buildings, vegetation, supported signs, and nuScenes static props.
This makes repeated builds idempotent. Traffic-light Blueprint placement is
disabled by default because functional CARLA lights also require trigger-volume
verification and junction grouping. Add `-PlaceTrafficLights` only for a
visual/trigger-layout review.

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
