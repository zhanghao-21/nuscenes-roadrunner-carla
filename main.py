#!/usr/bin/env python
"""nuScenes -> RoadRunner / CARLA reproduction pipeline.

One entry point for the five pipeline stages:

  1  build-rr-scene    Build the city scene in RoadRunner for each scenario
                       (roads + markings + signals + ground + OSM buildings)
                       [MATLAB: nuscenes_ground_buildings.m, RR Scene Builder]
  2  build-rr-fullmap  Build ONE large RoadRunner scene covering the whole
                       boston-seaport region
                       [MATLAB: fullmap/build_full_scene.m, RR Scene Builder]
  3  replay-rr         Replay the recorded agents in RoadRunner Scenario on
                       the per-scene city scene
                       [MATLAB: replay_agents.m, RR Scenario license]
  4  replay-carla      Replay each scene in CARLA on the geo map with the
                       RoadRunner buildings; saves the 6-camera views, the
                       ego-centred top-down view, and the chase-cam frames
                       [replay_geo/replay_geo_carla.py + replay_geo.py;
                        needs a running CARLA 0.9.15 server + carla915 env]
  5  align             Combine the real nuScenes camera views, the CARLA
                       camera views and the top-down view into aligned
                       per-keyframe panels + GIF
                       [replay_geo/build_aligned_geo.py]

Scene selection: --scene all (default), a roster index ("scene-01" ..
"scene-10", sorted scene order), a nuScenes id ("scene-0103"), or any
substring of the scenario name. All Python-stage outputs go to --output
(default: <project>/OUTPUT); RoadRunner scenes/scenarios are saved inside the
RoadRunner project (nuscenes_test1) and their file artifacts are copied to
--output as well.

Examples:
    python main.py --list
    python main.py --stage 1 --scene scene-01
    python main.py --stage 4 5 --scene scene-0103
    python main.py --stage all --scene all
    python main.py --stage 2                      # full map (no scene needed)
    python main.py --stage 4 --carla-python "C:/envs/carla915/python.exe"

Requirements per stage: 1-3 need MATLAB (Automated Driving Toolbox) +
RoadRunner; 4 needs a running CARLA server and the carla915 Python 3.7 env;
5 needs only the base Python env (numpy, matplotlib, Pillow) + v1.0-mini.
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RR_OUT = ROOT / "output_roadrunner"
REPLAY = ROOT / "replay_geo"

STAGE_NAMES = {
    "1": "build-rr-scene", "2": "build-rr-fullmap", "3": "replay-rr",
    "4": "replay-carla", "5": "align",
}


# ------------------------------------------------------------------ scenes
def roster():
    """Sorted scenario bases, e.g. boston-seaport_scene-0103 (roster order
    matches the MATLAB scripts: scene-01 .. scene-10)."""
    names = sorted(p.stem for p in RR_OUT.glob("*_scene-*.xodr")
                   if "_geo" not in p.stem)
    return names


def resolve_scenes(arg):
    bases = roster()
    if arg in (None, "", "all"):
        return bases
    # roster index scene-01 .. scene-NN
    if arg.startswith("scene-") and len(arg) == 8 and arg[6:].isdigit():
        i = int(arg[6:])
        if 1 <= i <= len(bases):
            return [bases[i - 1]]
    hits = [b for b in bases if arg in b]
    if not hits:
        sys.exit(f"no scenario matching '{arg}' - see python main.py --list")
    return hits


# ------------------------------------------------------------------ helpers
def run(cmd, dry, env_extra=None, cwd=None):
    printable = " ".join(str(c) for c in cmd)
    if env_extra:
        printable = " ".join(f"{k}={v}" for k, v in env_extra.items()) + " " + printable
    print(f"  $ {printable}")
    if dry:
        return
    env = None
    if env_extra:
        env = dict(os.environ)
        env.update({k: str(v) for k, v in env_extra.items()})
    subprocess.run([str(c) for c in cmd], check=True, env=env,
                   cwd=str(cwd) if cwd else None)


def run_matlab(args, script_dir, script, overrides):
    """matlab -batch "var='val'; ...; run('script.m')" (blocks until done)."""
    # MATLAB escapes apostrophes inside single-quoted text by doubling them.
    # This is required for shared folder paths such as "Wu, Keshu's files".
    def matlab_string(value):
        return str(value).replace("'", "''")

    pre = "".join(
        f"{k}='{matlab_string(v)}'; " for k, v in overrides.items()
    )
    code = f"{pre}run('{script}.m')"
    run([args.matlab, "-batch", code], args.dry_run, cwd=script_dir)


def copy_tree(src, dst, dry):
    if not Path(src).exists():
        print(f"  (nothing to copy: {src})")
        return
    print(f"  copy {src} -> {dst}")
    if not dry:
        if Path(src).is_dir():
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            Path(dst).mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)


def matlab_overrides(args):
    ov = {"base": str(ROOT),
          "nuscMapsDir": str(ROOT / "v1.0-mini" / "maps" / "expansion")}
    if args.rr_install:
        ov["installFolder"] = args.rr_install
    return ov


# ------------------------------------------------------------------ stages
def stage1_build_rr_scene(args, scenes):
    """City scene per scenario (RoadRunner Scene Builder via MATLAB)."""
    for base in scenes:
        print(f"[stage 1] build RoadRunner scene: {base}")
        ov = matlab_overrides(args)
        ov["sceneIdOverride"] = base.split("_")[-1]        # e.g. scene-0103
        run_matlab(args, ROOT, "nuscenes_ground_buildings", ov)
        copy_tree(RR_OUT / base, Path(args.output) / "roadrunner_scenes" / base,
                  args.dry_run)


def stage2_build_rr_fullmap(args):
    """One large boston-seaport scene (covers all four Boston scenarios)."""
    print("[stage 2] build RoadRunner full map: boston-seaport")
    run_matlab(args, ROOT / "fullmap", "build_full_scene", matlab_overrides(args))
    dst = Path(args.output) / "roadrunner_fullmap"
    for f in (ROOT / "fullmap").glob("boston-seaport_full*"):
        copy_tree(f, dst, args.dry_run)


def stage3_replay_rr(args, scenes):
    """RoadRunner Scenario replay per scenario (needs the stage-1 scene)."""
    for base in scenes:
        print(f"[stage 3] RoadRunner scenario replay: {base}")
        ov = matlab_overrides(args)
        ov["sceneIdOverride"] = base.split("_")[-1]
        run_matlab(args, ROOT, "replay_agents", ov)
        copy_tree(RR_OUT / base / "trajectories",
                  Path(args.output) / "roadrunner_scenarios" / base,
                  args.dry_run)


def stage4_replay_carla(args, scenes):
    """CARLA replay on the geo map: 6-cam views + top-down + chase cam."""
    env = {"REPLAY_GEO_RESULTS": Path(args.output) / "results"}
    carla_py = args.carla_python.split()
    for base in scenes:
        print(f"[stage 4] CARLA replay: {base}")
        run(carla_py + [REPLAY / "replay_geo_carla.py", "--scene", base,
                        "--buildings", "--cameras", "--record",
                        "--host", args.host, "--port", str(args.port)],
            args.dry_run, env_extra=env, cwd=REPLAY)
        # top-down: full-map replay GIF + ego-centred panel stills
        run([sys.executable, REPLAY / "replay_geo.py", "--scene", base,
             "--save-frames"], args.dry_run, env_extra=env, cwd=REPLAY)
        run([sys.executable, REPLAY / "replay_geo.py", "--scene", base,
             "--panel-frames"], args.dry_run, env_extra=env, cwd=REPLAY)
        # chase-cam GIF (in case the carla env has no Pillow)
        run([sys.executable, REPLAY / "make_gifs.py", "--scene", base],
            args.dry_run, env_extra=env, cwd=REPLAY)


def stage5_align(args, scenes):
    """Real nuScenes cams + CARLA cams + top-down -> aligned panels."""
    env = {"REPLAY_GEO_RESULTS": Path(args.output) / "results"}
    for base in scenes:
        print(f"[stage 5] align REAL vs SIM: {base}")
        run([sys.executable, REPLAY / "build_aligned_geo.py", "--scene", base],
            args.dry_run, env_extra=env, cwd=REPLAY)


# ------------------------------------------------------------------ main
def main():
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(__doc__.splitlines()[2:]))
    p.add_argument("--stage", nargs="+", default=["all"],
                   choices=list(STAGE_NAMES) + list(STAGE_NAMES.values()) + ["all"],
                   help="pipeline stage(s) to run: 1-5, their names, or all")
    p.add_argument("--scene", default="all",
                   help="all | scene-01..scene-10 (roster index) | scene-0103 "
                        "| name substring")
    p.add_argument("--output", default=str(ROOT / "OUTPUT"),
                   help="output folder (default: <project>/OUTPUT)")
    p.add_argument("--list", action="store_true",
                   help="list the scenario roster and exit")
    p.add_argument("--dry-run", action="store_true",
                   help="print the commands without running them")
    # tool locations
    p.add_argument("--matlab", default="matlab",
                   help="MATLAB launcher (default: matlab on PATH)")
    p.add_argument("--rr-install", default="",
                   help="RoadRunner bin folder, e.g. "
                        '"C:/Program Files/RoadRunner R2026a/bin/win64"')
    p.add_argument("--carla-python", default="conda run -n carla915 python",
                   help="python of the CARLA 0.9.15 client env (3.7)")
    p.add_argument("--host", default="127.0.0.1", help="CARLA server host")
    p.add_argument("--port", type=int, default=2000, help="CARLA server port")
    args = p.parse_args()

    if args.list:
        print("Scenario roster (index -> name):")
        for i, b in enumerate(roster(), 1):
            print(f"  scene-{i:02d} = {b}")
        return

    stages = []
    for s in args.stage:
        if s == "all":
            stages = ["1", "2", "3", "4", "5"]
            break
        for num, name in STAGE_NAMES.items():
            if s in (num, name):
                stages.append(num)
    scenes = resolve_scenes(args.scene)
    Path(args.output).mkdir(parents=True, exist_ok=True)

    print(f"stages: {', '.join(STAGE_NAMES[s] for s in stages)}")
    print(f"scenes: {', '.join(scenes)}")
    print(f"output: {args.output}\n")

    for s in stages:
        if s == "1":
            stage1_build_rr_scene(args, scenes)
        elif s == "2":
            stage2_build_rr_fullmap(args)
        elif s == "3":
            stage3_replay_rr(args, scenes)
        elif s == "4":
            stage4_replay_carla(args, scenes)
        elif s == "5":
            stage5_align(args, scenes)
    print("\ndone.")


if __name__ == "__main__":
    main()
