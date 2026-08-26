# Data-seeded closed-loop simulation

This folder adds two interactive traffic modes without changing the existing
open-loop `replay_persistent.py` baseline:

- `run_tm_closed_loop.py`: CARLA-native Traffic Manager baseline.
- `run_sumo_hybrid.py`: SUMO-controlled moving background traffic with
  CARLA-authority ego, selected critical actors, and recorded parked vehicles.

The same nuScenes manifest supplies initial poses, reference routes, actor
appearance times, categories, and recorded speeds. The hybrid deliberately
prestages recorded parked vehicles at startup by default; moving-vehicle
departures remain data-seeded as described in Section 3. All commands below
are for **Anaconda Prompt** (`cmd.exe`) and start in the local repository. No
command writes to the OneDrive source project.

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
| `carla_static` | Recorded parked vehicle fixed in CARLA with physics disabled |
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

Stationary recorded vehicles remain static. By default, a multi-sample vehicle
is moving when the maximum separation between any two observations is at least
2 m; this spatial extent avoids mistaking back-and-forth annotation jitter for
travel. A two-point track observed over at most 0.5 s is also moving when its
speed is at least 1 m/s. The shared `--minimum-track-distance` and
`--minimum-two-point-speed` options change those boundaries. Moving vehicles
receive the recorded path, average recorded speed, collision
avoidance, and configurable following distance. Add `--auto-lane-change` only
when free lane-changing is desired; it is disabled by default to preserve the
recorded route.

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
`route_report.json`, and `hybrid_config.json`. Preparation uses the same motion
classifier as the CARLA Traffic Manager baseline: moving tracks become SUMO
routes, while stationary multi-sample vehicle tracks are written to
`static_actors` and remain fixed in CARLA. Moving observations farther than 8 m
from a compatible connected SUMO lane are skipped instead of being silently
assigned to an incorrect road.

Inspect all three groups in `route_report.json`: `included` contains moving
SUMO vehicles, `carla_static` contains recorded parked vehicles, and `skipped`
contains non-vehicles, explicitly CARLA-controlled critical actors, or
insufficient records. One-point vehicle tracks are omitted in both baselines
because motion cannot be classified consistently from one observation.

By default, all `carla_static` actors are spawned at their recorded poses
before the main scenario clock starts. The runner then performs the configured
bridge warmup so CARLA registers the complete parked backdrop before moving
SUMO traffic is released. Their one-tick bridge spawn notifications are
deliberately filtered: the recorded vehicles remain physics-disabled and
visible at their true CARLA roadside poses, but SUMO does not create lane-mapped
shadows for them. This prevents an off-lane parked actor from becoming a false
SUMO car-following leader. Dynamic CARLA-owned actors such as the ego and
critical vehicle are still mirrored so SUMO traffic can react to them. Use
`--static-spawn-mode recorded` only when the original annotation appearance
times are specifically required instead of a complete parked-vehicle backdrop
at startup.

This policy assumes `carla_static` means a roadside parked backdrop. SUMO does
not model those parked bodies, so a mislabeled static vehicle that actually
protrudes into a travel lane will remain visible in CARLA but will not make a
SUMO mover yield. The run summary records this tradeoff explicitly under
`sumo_static_proxy_exclusion`.

A moving route's source departure group is its **first valid SUMO lane-match
timestamp**, not necessarily its first raw annotation timestamp. This prevents
an actor whose early observations are outside `--maximum-snap-distance` from
being inserted early at a position taken from a later observation. Actors with
the same first-match timestamp retain the same scheduled departure time and
are therefore offered to SUMO as a batch.

Without `--maximum-moving-departure-gap`, preparation preserves the recorded
gaps between those ordered moving-vehicle groups, apart from the small common
startup delay required for bridge warmup. This is the recommended fidelity
setting and is used by the primary command above. For an accelerated visual
demo, add `--maximum-moving-departure-gap 1.5`: only source gaps longer than
1.5 seconds are shortened; equal timestamps and shorter gaps are unchanged.
That option deliberately changes temporal alignment with the recorded
ego/reference scenario and can therefore change TTC and prediction-based
inverse-TTC values.

The scheduled departure is still a request. For the hybrid run, the default
`--moving-initial-pose recorded` policy places each due mover once at its first
matched recorded pose immediately before CARLA mirrors it. This is important
for actors first observed inside a junction: placing all of them at the end of
the incoming edge can create an artificial overlap and a long insertion delay.
The generated report labels this annotation as a vehicle-center pose; the
runner converts it to SUMO's front-center-bumper reference before `moveToXY`,
so the CARLA bridge converts it back to the intended recorded center.
Position control is released immediately after this one-time correction; SUMO
then owns the actor. A one-time SUMO warning about an implied speed after a
multi-metre `moveToXY` correction is expected for such an internal-junction
start; the correction status is recorded in `summary.json`. This is not a
teleport or a continuing pose replay. Use `--moving-initial-pose route` to
disable this correction.

The default `--moving-speed-policy recorded_profile` supplies the recorded
piecewise speed as a desired target while that actor still has observations.
SUMO retains safe-speed, braking, signal, right-of-way, and car-following
checks, so it may drive slower when the closed-loop traffic state requires it.
After the last recorded observation, the runner releases the speed target and
SUMO continues autonomously. It also detects a **terminal stationary tail**:
by default, if all remaining recorded positions stay within a 1.0 m extent for
at least 2.0 seconds, the speed replay is released at the start of that tail.
This prevents a moving actor whose source track ends parked from being held at
the center of its SUMO lane. The test uses the complete remaining track, not an
instantaneous zero speed, so a temporary mid-track pause is not classified as a
terminal stop. Once released, SUMO still obeys its own red lights, junction
priority, stop rules, leaders, and collision avoidance.

Releasing a terminal stationary tail changes only the desired-speed authority.
It preserves the vehicle's complete prepared SUMO route and does not
immediately select a random outgoing turn or replace the unobserved suffix. The
actor first continues autonomously along the same recorded prefix and prepared
continuation that it had before the release.

Use `--terminal-stop-policy preserve_recorded` when reproducing the terminal
stop is more important than keeping the SUMO lane open. The defaults can be
tuned with `--terminal-stop-maximum-extent` and
`--terminal-stop-minimum-duration`. `recorded_mean` keeps only a mean-speed cap,
and `unbounded` uses the generic type limits. With the default recorded-pose
policy, both non-profile speed policies still use the recorded initial velocity
as a two-tick insertion seed before returning desired-speed control to SUMO.

Route matching is continuity-constrained and includes SUMO's internal junction
lanes, including chained connectors and lane vehicle-class permissions. Thus a
recorded turn through an intersection remains part of the route instead of
being collapsed to one incoming edge. `recorded_edges` in `route_report.json`
is kept as an immutable prefix. The default
`--moving-route-continuation terminal` setting appends seeded, network-valid
`continuation_edges`, avoids generated
U-turns, and prefers a reachable map-boundary terminal. An interior edge with
no valid outgoing connection is rejected as a route end, including when it is
reachable through a longer acyclic branch. If no boundary is reachable but a
closed road cycle is, preparation filters out interior-dead-end branches and
creates a deterministic cyclic tail whose distance covers the remaining
simulation horizon at that mover's speed ceiling. The default
`--route-continuation-search-depth 64` is both the terminal-search depth and
minimum cyclic tail; a duration-sized cycle may deliberately contain more than
64 edges, with a separate 100,000-edge malformed-network safety guard. Change
`--route-continuation-seed` to obtain another reproducible choice of left,
right, or straight successors.

The hybrid runner also has a route-tail safeguard for older or manually
shortened route files. It acts proactively near an owned SUMO mover's final
normal edge, before route exhaustion can leave the actor stopped, and appends a
complete class-valid path to a genuine boundary or reusable cycle beyond the
existing route; it never appends only one more finite interior edge. It does
not replace the recorded route merely because speed replay ended. CARLA-owned actors mirrored
into SUMO are excluded, U-turn-only connections are rejected by default, and a
true map-boundary road end with no class-valid outgoing connection is recorded
and left unchanged.

Selecting an alternative turn is a separate, guarded recovery rather than the
normal stationary-tail release behavior. It is eligible only after an actor
has remained stopped for a continuous **unguarded** persistence window (1.0
second by default). If recorded-speed replay is still active, the first full
window releases only that desired-speed authority. Route recovery requires the
vehicle to remain stopped through a new full autonomous window, so recorded
replay release and a route change never happen in the same recovery action. A
scheduled stop, close leader, nearby red/yellow signal, immediate
stop-controlled link, or
immediate blocked/conflicting link resets that timer; an ordinary signal,
stop-rule wait, or queue therefore cannot trigger recovery.

The guard also follows the planned route through a normal edge that is too
short to store the vehicle plus its minimum gap. If the following junction is
still conflicted, the runner waits the same 1.0-second persistence window. It
then preserves the recorded immediate successor and changes only
the generated suffix to another seeded route that can reach a genuine map
boundary or a reusable cycle. When the immediate successor has a forced
continuation, the search follows that prefix and may diverge at a later branch.
Each failed branch is remembered so a retry cannot oscillate back to an old
suffix. Only if no distinct viable suffix exists may it
consider a different immediate outgoing edge, and that edge must also have a
complete viable path to a boundary or reusable cycle. Short interior conversion
stubs are rejected even when SUMO reports them as valid immediate turns. If
the selected immediate branch also remains stopped, it is remembered and a
new persistence window may select a distinct viable branch instead of
permanently suppressing recovery or oscillating. If the first viable suffix
remains blocked, the runner may try another distinct suffix, up to three
attempts per stopped current/planned link. Recovery changes route intent only:
it never commands a positive speed or bypasses SUMO's collision avoidance or
right-of-way rules. The thresholds live under
`background.route_continuation.persistent_stop_recovery` in
`hybrid_config.json`.

For each included moving actor, `route_report.json` records
`recorded_start_time`, `first_matched_time`, and the final scheduled `depart`.
Its `moving_departure_schedule` object records the common startup delay, the
optional maximum gap, and eager-insertion setting. Compare these values when a
vehicle seems to enter earlier or later than expected. It also records the
matched/internal lanes, snap-error statistics, recorded route prefix,
heading-error statistics, continuation tail, initial SUMO pose, mean/peak speed,
and continuation status.

`prepare_sumo.py` rewrites `hybrid_config.json` every time it runs. The
`--keep-existing-net` option preserves only `network.net.xml`; it does not merge
or preserve optional settings from the previous hybrid configuration. Therefore,
repeat options such as `--enable-prediction-risk`, `--critical-actor`, and any
intentional timing override whenever regenerating the corresponding experiment.

For the current scene-0103 conversion at the default boundaries, validation
finds 11 moving and 51 stationary recorded vehicles. Five one-point vehicle
records cannot be classified and 46 pedestrian tracks are outside SUMO route
generation. If the example critical actor in Section 5 is selected, the split
is 10 SUMO moving vehicles, 51 CARLA-static vehicles, and one CARLA-physics
critical vehicle.

Optional standalone validation:

```bat
"%SUMO_HOME%\bin\sumo.exe" ^
  -c carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\scene.sumocfg ^
  --no-step-log true --duration-log.statistics true
```

This standalone command validates the generated network and route XML. The
one-time recorded-pose correction and recorded speed profile are hybrid-runner
features, so use Section 4 when validating full data alignment.

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
`--ego-mode external`. SUMO controls only the classified moving background
vehicles. By default, recorded parked vehicles are prestaged together as
physics-disabled CARLA actors and persist at their recorded poses. Before the
main scenario loop, bridge-only warmup ticks register those actors but filter
their CARLA-to-SUMO spawn notifications. This keeps the parked backdrop in
CARLA without creating false in-lane SUMO obstacles; moving departures include
a small startup delay so they begin after that warmup. The bridge still mirrors
the CARLA-owned ego and critical actors so moving traffic can react to them;
SUMO vehicle poses are synchronized back into
CARLA for rendering and sensors.

Every `carla_reference` track is finite. At its recorded end time, a lagging
physics actor does not brake immediately wherever it happens to be. Instead,
the controller continues along the remaining recorded path toward the final
position and caps speed with a constant-deceleration stopping envelope. This
lets the actor reach the recorded endpoint without teleporting; it does not
invent an unrecorded roadside pull-over. Within the endpoint tolerance, or
after crossing the final path plane, the controller latches zero throttle, zero
steering, and full brake on every subsequent tick; it therefore cannot turn
back and circle the final waypoint. The first hold tick, approach duration,
actual/recorded hold positions, endpoint error, and speed are stored under
`reference_track_end_control` in `summary.json`.

The terminal defaults are a 0.75 m endpoint tolerance and 3.0 m/s² planned
deceleration. A critical-actor entry can override them with
`endpoint_stop_tolerance_m` and `endpoint_deceleration_mps2`. These values tune
the physical approach only; the actor is never teleported or transferred to
SUMO.

The console logs each expected mover's `inserted`, `mirrored`, and `completed`
events. The run `summary.json` stores the same information under
`sumo_mover_lifecycle`, plus one-time pose/profile results under
`sumo_mover_fidelity`. Terminal stationary-tail release times and reasons are
recorded there without implying a route change. Proactive tail-extension and
guarded recovery diagnostics are stored under
`sumo_runtime_route_continuation`. `not_inserted_before_stop` means SUMO never
reported the actor departing before shutdown; compare its scheduled `depart`
with the SUMO `sumo_time_at_stop_s` to distinguish a not-yet-due actor from a
blocked or failed insertion.
`inserted_but_never_mirrored` indicates that an inserted SUMO actor never
obtained a CARLA mirror, for example because blueprint selection or CARLA spawn
failed. `completed` identifies a SUMO route exit. For the current full
scene-0103 validation, all 10 SUMO movers were inserted and mirrored; the two
that completed had reached the true boundary edge `-37`, while the other eight
were still active at the scenario stop.

The `sumo_static_proxy_exclusion` summary section reports how many recorded
static CARLA actor IDs were filtered before SUMO proxy creation, any legacy
proxies detached by the fallback, and any failures. The normal result is one
filtered entry per `carla_static` actor, zero detached proxies, and zero
failures.

Do not run another CARLA client that calls `world.tick()` at the same time.
External agents may apply controls, but the hybrid runner remains the only tick
owner.

### Optional HGT prediction-risk dashboard

The hybrid runner can observe the CARLA ego and nearby synchronized vehicles,
use the bundled HGT model to predict multi-modal neighbor trajectories, and
show their prediction-based inverse TTC values in a live dashboard. This is an
observational safety display: its predictions and risk values do not alter the
reference controller, Traffic Manager, SUMO, or an external ego controller.
The simulation remains closed-loop because those existing controllers and the
SUMO-CARLA bridge react to one another; the dashboard is not additional control
feedback.

Prediction-risk neighbor filtering uses recorded authority, not instantaneous
speed. Actors classified as `carla_static` are excluded from HGT histories,
inverse-TTC calculations, CARLA prediction drawings, and dashboard rows, so
parked roadside vehicles do not consume the nearest-neighbor slots. SUMO movers
and other dynamic CARLA actors remain eligible while temporarily stopped at a
signal or in a queue. This filter affects only prediction-risk observation; the
static actors remain spawned and continue participating in the simulation and
legacy safety metrics. Its policy and excluded track IDs/count are recorded
under `prediction_risk.actor_filter` in `summary.json`.

Install the optional dependencies into the same environment that runs the
hybrid simulation:

```bat
conda activate carla_0915
python -m pip install -r carla_reconstruction\requirements-prediction.txt
python -c "import torch, yaml, pyqtgraph, PyQt5; print('prediction dashboard dependencies OK')"
```

The requirements are deliberately not pinned to an unverified Torch/CUDA
combination. If a CUDA build is required, install a Python-3.7-compatible
PyTorch build appropriate for that machine first, then install the requirements
file. CPU inference is the most portable initial check.

A `hybrid_config.json` whose `prediction_risk` section is enabled needs no
second dashboard command. Run the same command shown above exactly as written:

```bat
python carla_reconstruction\runtime\run_sumo_hybrid.py ^
  --config carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\hybrid_config.json ^
  --sumo-gui --ego-mode reference
```

Future configurations can opt in while being generated:

```bat
python carla_reconstruction\tools\prepare_sumo.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --enable-prediction-risk
```

The generated configuration also requests wall-clock real-time pacing, so the
dashboard-enabled command does not need a separate `--realtime` switch. The
runner still advances both simulators synchronously and never skips a simulation
step; if CPU inference takes longer than one step, wall-clock execution will be
slower than real time. Older hand-written configurations can continue to use
`--realtime` explicitly.

Command-line overrides take precedence over the JSON configuration:

- `--prediction-risk` enables prediction and the dashboard for a configuration
  that does not opt in.
- `--no-prediction-risk` disables the entire optional observer and avoids
  importing Torch or Qt.
- `--prediction-device cpu` provides a deterministic compatibility path;
  `--prediction-device cuda` or a specific CUDA device can be used only when
  the installed Torch build supports it.

Dashboard layout is controlled in `prediction_risk.dashboard`. The optional
`column_width_px` value sets the approximate width of each of the two plot
columns (450 pixels by default); the complete window is approximately twice
that width. `max_neighbors` controls the number of actor rows, while
`mode_count` controls the number of inverse-TTC bars in each left-hand plot.
The initial window gives both columns equal space and is clamped to the
available screen. Compact two-line plot titles prevent actor metadata from
pushing the right-hand history plots outside the visible window; hover over a
plot to see the complete actor ID and risk details.

At the default 0.05 s simulation step, the predictor first accumulates 30
consecutive samples for each neighbor, so a newly observed pair displays a
1.5 s warm-up state. It then predicts 50 future steps, a 2.5 s horizon. A new
neighbor warms up independently and does not suppress predictions for already
ready neighbors. The bundled checkpoint has no independent sampling-period
metadata, so the integration preserves and enforces the reference's 0.05 s
cadence instead of silently reinterpreting the model at another step length.
For the same reason, `prepare_sumo.py --enable-prediction-risk` rejects a
different `--step-length` until an explicit resampling layer is implemented.

The bundled checkpoint behaves as a fixed-axis highway model: controlled
east-, west-, north-, and south-moving inputs all produce primarily canonical
`+X` displacements. Interpreting those outputs directly as CARLA-world axes is
why trajectories can appear ahead of vehicles travelling one way and behind
vehicles travelling the opposite way. The default model setting therefore uses
an explicit output-frame compatibility transform:

```json
"model": {
  "output_frame": "actor_heading",
  "heading_history_samples": 5,
  "minimum_heading_displacement_m": 0.05
}
```

The bundled checkpoint selects `actor_heading` automatically. A custom
checkpoint defaults to `world` unless its configuration explicitly requests a
different output frame, so existing world-frame model adapters are not silently
rotated.

`actor_heading` rotates each decoded canonical longitudinal/lateral trajectory
to the synchronized vehicle's CARLA orientation before both inverse-TTC
evaluation and CARLA debug drawing. This remains stable while a vehicle is
stopped. If orientation is unavailable, direction is estimated from the most
recent five history samples; if that displacement is too small, the complete
30-sample history is used. Only a fully stationary actor with no supplied
orientation retains the canonical world-`+X` fallback. Set `output_frame` to
`world` for a checkpoint whose output coordinates are known to be CARLA-world
coordinates. A global X/Y sign flip is not used because it merely exchanges
which travel directions are wrong.

This compatibility transform aligns the bundled checkpoint's observed
canonical longitudinal output with each actor; it does not turn the supplied
checkpoint into a calibrated intersection model. A checkpoint
trained or validated on CARLA intersection motion, with documented coordinate
and target conventions, is still required before treating predicted paths or
risk values as ground truth. In particular, longitudinal forward alignment has
been verified, while the checkpoint's lateral handedness and lane-change
calibration remain undocumented.

The ego comparison path also preserves the supplied example: it is a
constant-velocity extrapolation from the ego's last two synchronized positions,
not the full future route of the reference controller or an external planner.
The bundled MARTS implementation applies that plan feature only to model agent
zero (the nearest ready neighbor); the remaining neighbors are still predicted
from their observed histories and social features but are not plan-conditioned.
These checkpoint/model semantics should be validated before treating the risk
values as calibrated safety probabilities.

The displayed quantity preserves the reference implementation's legacy
rectangular-overlap definition. For each predicted mode, the first future step
where the ego plan and neighbor prediction are within the configured CARLA
world-`x`/world-`y` thresholds (`overlap_half_x_m` and `overlap_half_y_m`) is
treated as the TTC; no predicted overlap yields zero inverse TTC. Inverse TTC
is reported in `s^-1`. This axis-aligned surrogate is
not an oriented-box collision test and is distinct from the line-of-sight
closing TTC already written to `metrics.csv`. The dashboard reports both the
worst mode (maximum inverse TTC) and the probability-weighted inverse TTC over
all 20 model modes. To keep each row readable, the bar chart shows only the
configured number of modes (five by default); this visual limit does not
truncate either aggregate.

The Qt dashboard runs as a child process rather than taking over the simulation
thread. The hybrid runner owns it and requests shutdown during normal exit,
Ctrl+C, and exception cleanup. Stop the runner itself to finish the complete
CARLA/SUMO cleanup; closing only the dashboard window is not a substitute for
stopping the simulation.

CARLA debug-line drawing of the ego plan and predicted trajectories is disabled
by default because drawing every segment can delay the synchronous bridge tick.
It can be enabled deliberately with `prediction_risk.drawing.enabled` in the
hybrid JSON; the separate live dashboard does not require that overlay. Each
purple path is anchored from the vehicle's current position to its first future
sample, then follows the direction-corrected prediction.

If the dashboard does not open:

1. First check whether `prepare_sumo.py` was rerun without
   `--enable-prediction-risk`. That removes the `prediction_risk` section from
   the regenerated `hybrid_config.json`, so the runner correctly starts without
   the dashboard. `generate_safety_variants.py` copies the base configuration;
   it cannot restore a missing prediction-risk section.
2. To enable the dashboard immediately without regenerating files, force the
   runtime override:

   ```bat
   python carla_reconstruction\runtime\run_sumo_hybrid.py ^
     --config carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\hybrid_config.json ^
     --sumo-gui --ego-mode reference ^
     --prediction-risk
   ```

3. To restore automatic dashboard startup in the saved configuration, rerun
   `prepare_sumo.py` with `--enable-prediction-risk`. If the experiment also
   uses a critical actor, repeat both options as shown in Section 5 below.
4. Confirm that `python` is the same Python 3.7 environment that imports the
   CARLA API, then run the dependency import check above.
5. Retry with `--prediction-device cpu` when Torch reports a CUDA or driver
   mismatch.
6. Check that the model YAML and checkpoint under
   `carla_reconstruction\HGT_model` are present and readable from the local
   repository. PyTorch releases compatible with Python 3.7 may use the legacy
   pickle loader, so configure only a trusted local checkpoint.
7. A Qt platform-plugin error usually indicates a broken or mixed Qt install;
   reinstall PyQt5 and PyQtGraph in this environment rather than copying Qt DLLs
   between environments.
8. Use `--no-prediction-risk` to run the unchanged hybrid simulation while the
   optional prediction environment is repaired.

Offline tests can cover history warm-up, HGT input/output plumbing, inverse TTC,
and dashboard-process messaging without CARLA or SUMO. A successful combined
CARLA-SUMO-dashboard smoke test is still required on the simulator machine; it
is not claimed by the offline test suite.

## 5. Designate a CARLA-physics critical actor

Actor IDs are listed in `route_report.json`. Rebuild the SUMO files while
excluding the selected actor from SUMO. Use an `included[].id` value, not the
`sumo_id` value prefixed with `nusc_`. `ACTOR_ID_FROM_ROUTE_REPORT` is descriptive
placeholder text and must never be passed literally.

The following is a complete dashboard-enabled example for scene-0103. The actor
shown is a real ID in the current route report; replace its value if a different
surrounding vehicle is the intended critical actor:

```bat
set "SUMO_HOME=C:\Traffic software\SUMO"
set "CRITICAL_ACTOR=c283b224a9984736bff67a2f347866fa"

python carla_reconstruction\tools\prepare_sumo.py ^
  --manifest carla_reconstruction\generated\boston-seaport_scene-0103\scene_manifest.json ^
  --keep-existing-net ^
  --critical-actor %CRITICAL_ACTOR% ^
  --enable-prediction-risk
```

In Windows Command Prompt, each `set` command must be on its own line (or joined
to the next command with `&&`). Use `^`, not `\`, for command continuation.

The generated `hybrid_config.json` now gives that actor
`carla_reference` authority. It follows the recorded route with CARLA physics,
reacts to a leading vehicle using a configurable time headway, and can execute
scenario events. Explicit ego/critical authority takes precedence if an actor
would otherwise be classified as static.

## 6. Generate safety-critical braking variants

Run this in the same Command Prompt as Section 5, or set `CRITICAL_ACTOR` to
the same real route-report ID again before invoking the generator:

```bat
set "CRITICAL_ACTOR=c283b224a9984736bff67a2f347866fa"

python carla_reconstruction\tools\generate_safety_variants.py ^
  --config carla_reconstruction\generated\closed_loop\boston-seaport_scene-0103\sumo\hybrid_config.json ^
  --critical-actor %CRITICAL_ACTOR% ^
  --count 20
```

The generator uses deterministic Latin-hypercube samples of brake start time,
duration, intensity, desired-speed scale, and time headway. Run a variant by
passing its JSON file to `run_sumo_hybrid.py --config`. Variants inherit the
base `hybrid_config.json`, including its static-actor partition and
startup/departure schedule, as well as its prediction-risk/dashboard setting,
so prepare the base configuration with all desired options before generating
them. Regenerate existing variants after rerunning `prepare_sumo.py`; otherwise
those older JSON files retain the old actor partition and schedule.

Braking events are generated only inside the selected critical actor's finite
recorded control window. If a requested start range extends beyond that window,
the generator reports and records an effective bounded range; it also shortens
an individual duration when necessary so the complete event ends no later than
the track end. `index.json` records the track window, requested/effective timing
ranges, each brake end time, and the number of adjusted events. This prevents a
variant from scheduling its intervention after the recorded control window has
ended and the terminal endpoint approach described in Section 4 has begun.

## Run outputs and current boundary

Each TM or hybrid run writes `metrics.csv` and `summary.json` under the ignored
`carla_reconstruction\generated\closed_loop_runs` folder. The first metrics
are collision count/impulse, minimum ego-to-actor distance, ego speed, and
line-of-sight closing TTC. Every summary records the manifest, map, timing,
seed/config, actor authorities, SUMO mover lifecycle, and initial-pose/speed
fidelity status.

When prediction risk is enabled, its mode-wise, worst-case, and
probability-weighted inverse TTC values are live dashboard outputs, and the run
summary records the observer configuration/status. They are not added to or
used to replace the legacy `metrics.csv` closing-TTC fields.

The moving SUMO traffic is **data-seeded and closed-loop, not trajectory
replay**. The recorded observations determine the initial pose, departure
batch, route prefix, and desired speed profile. SUMO still decides the safe
realized speed, following response, intersection behavior, and lane changes;
after recorded observations end it also owns the desired speed. Continuation
edges beyond the data are seeded valid choices rather than ground truth. The
route types use IDM car following and SL2015 lane changing because CARLA's
bridge enables SUMO sublane simulation. Consequently, a mover can deviate from
its recorded position when it reacts to the ego, a critical actor, a leader, or
right-of-way constraints. Driver-model calibration against nuScenes headway,
acceleration, and lane-change observations remains a research stage.
