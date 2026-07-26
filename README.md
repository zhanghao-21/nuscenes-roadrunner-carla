# nuScenes → RoadRunner / CARLA scene reconstruction

Rebuilds the nuScenes v1.0-mini driving scenes as simulation environments:
OpenDRIVE maps converted from the nuScenes map layers, city scenes in
MathWorks RoadRunner (roads, real lane markings, traffic signals, OSM
buildings), and live replays in CARLA with the recorded agents, plus
REAL-vs-SIM aligned camera comparisons.

## Quick start — `main.py`

All five pipeline stages run through one entry point:

```bash
python main.py --list                        # scenario roster (scene-01..10)
python main.py --stage 1 --scene scene-01    # RoadRunner scene, one scenario
python main.py --stage 2                     # RoadRunner full boston-seaport map
python main.py --stage 3 --scene all         # RoadRunner Scenario agent replay
python main.py --stage 4 --scene scene-0103  # CARLA replay: cams+topdown+chase
python main.py --stage 5 --scene all         # aligned REAL vs SIM panels
python main.py --stage all --scene all       # everything
```

| stage | what it does | needs |
|-------|--------------|-------|
| 1 `build-rr-scene` | per-scenario RoadRunner city scene ([nuscenes_ground_buildings.m](nuscenes_ground_buildings.m)) | MATLAB + RoadRunner (Scene Builder) |
| 2 `build-rr-fullmap` | one 1.9×1.3 km boston-seaport scene ([fullmap/build_full_scene.m](fullmap/build_full_scene.m)) | MATLAB + RoadRunner (Scene Builder) |
| 3 `replay-rr` | recorded agents replayed in RoadRunner Scenario ([replay_agents.m](replay_agents.m)) | MATLAB + RoadRunner (Scenario) |
| 4 `replay-carla` | CARLA replay on the geo map + buildings; saves 6-cam views, top-down, chase cam ([replay_geo/](replay_geo/)) | CARLA 0.9.15 server + `carla915` env |
| 5 `align` | real nuScenes cams + CARLA cams + top-down → per-keyframe panels + GIF ([replay_geo/build_aligned_geo.py](replay_geo/build_aligned_geo.py)) | base Python env |

`--scene` accepts `all`, a roster index (`scene-01`..`scene-10`), a nuScenes id
(`scene-0103`), or a name substring. Everything is written to `OUTPUT/`
(change with `--output`): Python results under `OUTPUT/results/<scenario>/`,
RoadRunner artifacts copied to `OUTPUT/roadrunner_*/`. Stage 4 must run
before stage 5 fills the SIM row (until then it renders placeholders).
`--dry-run` prints every command without executing.

Tool locations (when not on PATH / non-default):
`--matlab`, `--rr-install "C:/Program Files/RoadRunner R2026a/bin/win64"`,
`--carla-python "C:/envs/carla915/python.exe"` (default `conda run -n carla915
python`), `--host/--port` for the CARLA server.

## Environments

- **base Python 3.12**: `numpy`, `matplotlib`, `Pillow` (stages 4's top-down + 5)
- **carla915 (Python 3.7)**: `conda env create -f nuscenes2xodr/environment_carla37.yml`,
  then install the CARLA 0.9.15 wheel (see [nuscenes2xodr/README.md](nuscenes2xodr/README.md))
- **MATLAB** R2024b+ with Automated Driving Toolbox; RoadRunner R2026a with
  Scene Builder + Scenario licenses (stages 1–3)

## Repository layout

- [nuscenes2xodr/](nuscenes2xodr/) — nuScenes → OpenDRIVE converter + original
  CARLA loader (see its README for the conversion pipeline)
- [replay_geo/](replay_geo/) — self-contained replay/alignment tools used by
  stages 4–5 (see its README)
- [carla_reconstruction/](carla_reconstruction/) — persistent source-built
  CARLA/Unreal reconstruction: manifest preparation, buildings/vegetation/signs,
  saved-level editor automation, and trajectory replay on the packaged map
- [output_roadrunner/](output_roadrunner/) — converted per-scenario inputs:
  `.xodr`, `_meta.json` sidecars, OSM extracts, RoadRunner `_geo.xodr`/`.rrhd`
- [fullmap/](fullmap/) — full boston-seaport region inputs/outputs
- [nuscenes_test1/](nuscenes_test1/) — the RoadRunner project (scenes are
  saved into it by stages 1–3)
- [v1.0-mini/](v1.0-mini/) — the nuScenes mini dataset (tables, images, maps)
