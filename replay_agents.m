%% replay_agents.m
% Replay the recorded nuScenes agents (cars, trucks, pedestrians,
% cyclists, motorcycles) plus the ego vehicle in a RoadRunner Scenario,
% on top of the city scene built by nuscenes_ground_buildings.m.
%
% Pipeline: <scene>_meta.json agent_frames (patch-local x/y/yaw at
% frame_dt) -> one CSV trajectory per agent (RoadRunner "CSV Trajectory"
% format: columns time,x,y,yaw; yaw in radians) -> importScenario with
% per-category actor assets, spawn/remove times from when each agent
% enters/leaves the recording -> saved .rrscenario.
%
% Run nuscenes_ground_buildings.m for the same scene first (the scenario
% opens that scene). Requires a RoadRunner Scenario license.

%% Config
% base / sceneIdOverride / installFolder can be pre-set by the caller
% (e.g. main.py via matlab -batch "base='...'; run('replay_agents.m')")
% Otherwise locate the repo by trying, in order, this script's folder, its
% folder on the path, and the current folder - the first one that actually
% holds output_roadrunner wins. mfilename is empty when only a section or
% a selection of this file is run instead of the whole file, which used to
% leave base = "" and make every scene look missing.
baseTry = strings(0, 1);
if exist("base", "var"), baseTry(end+1) = string(base); end
baseTry(end+1) = string(fileparts(mfilename("fullpath")));
baseTry(end+1) = string(fileparts(which("replay_agents.m")));
try     % the file open in the editor - this is the section-run case
    baseTry(end+1) = string(fileparts(matlab.desktop.editor.getActiveFilename));
catch   % no editor (matlab -batch): nothing to add
end
baseTry(end+1) = string(pwd);
baseTry = baseTry(strlength(baseTry) > 0);
base = "";
for b = baseTry(:)'
    if isfolder(fullfile(b, "output_roadrunner")), base = b; break; end
end
assert(strlength(base) > 0, "Cannot find the repo: no output_roadrunner " + ...
    "folder under any of" + newline + join("  " + baseTry, newline) + ...
    newline + "Set it explicitly, e.g. base = ""C:\path\to\nuscenes_1"";");
if exist("sceneIdOverride", "var")
    sceneId = sceneIdOverride;
else
sceneId = "scene-01";       % same convention as nuscenes_ground_buildings
end
outDir  = base + "\output_roadrunner";

projectFolder = base + "\nuscenes_test1";
if ~exist("installFolder", "var")
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";
end

% actor asset per nuScenes category (paths relative to the project's
% Assets folder, per the CSV-trajectory import convention)
carAssets = ["Vehicles/Sedan.fbx", "Vehicles/Suv.fbx", ...
             "Vehicles/CompactCar.fbx"];      % cycled for variety
CAT = struct( ...
    "vehicle_car",              "",  ...      % "" -> pick from carAssets
    "vehicle_truck",            "Vehicles/PickupTruck.fbx", ...
    "vehicle_bus",              "Vehicles/SchoolBus.fbx", ...
    "vehicle_construction",     "Vehicles/UtilityTruck.fbx", ...
    "vehicle_emergency",        "Vehicles/Ambulance.fbx", ...
    "vehicle_bicycle",          "NCAP Assets/NCAP_Bicycle_NoPlatform.fbx", ...
    "vehicle_motorcycle",       "NCAP Assets/NCAP_Motorcycle_NoPlatform.fbx", ...
    "human_pedestrian_adult",   "Characters/Citizen Male Casual01.rrchar", ...
    "human_pedestrian_child",   "Characters/Citizen Male Child01.rrchar");
egoAsset = "Vehicles/Sedan.fbx";
egoColor = "#00B050";
minTrackFrames = 2;         % skip agents seen in fewer frames
includePedestrians = true;  % pedestrians need .rrchar character assets
                            % (R2024b+); prop/mesh assets abort the sim

% Pose clean-up (see section 1b). An agent counts as parked on frames
% where nothing within +/-stillWin/2 seconds of it sits more than
% stillPosSpread away - a window rather than a single step, so the odd
% 0.3 m annotation spike cannot masquerade as movement. Parked stretches
% are pinned to one position, and to one heading unless they turn by more
% than stillYawSpread, which is more rotation than noise can explain.
% stillDrift caps how wide a stretch may get before it is pinned in more
% than one piece; it is deliberately looser than stillPosSpread, because
% a parked car's box can wander a metre over a whole scene and splitting
% that stretch would put all of the wander into one visible jump.
stillWin       = 3.0;               % s
stillPosSpread = 1.0;               % m
stillDrift     = 3.0;               % m
stillYawSpread = deg2rad(45);

% Collision-avoidance footprints [length width] (m) used to pre-check the
% replay against RoadRunner's default "fail on collision" condition.
% Vehicle footprints are the substitute-asset sizes plus a small margin;
% pedestrians use tight boxes so groups walking together survive.
FOOT = struct( ...
    "vehicle_car",            [5.1 2.2], ...
    "vehicle_truck",          [5.9 2.4], ...
    "vehicle_bus",            [11.5 2.9], ...
    "vehicle_construction",   [5.8 2.5], ...
    "vehicle_emergency",      [6.8 2.6], ...
    "vehicle_bicycle",        [2.0 0.8], ...
    "vehicle_motorcycle",     [2.4 1.0], ...
    "human_pedestrian_adult", [0.6 0.6], ...
    "human_pedestrian_child", [0.5 0.5]);
egoFoot = [5.0 2.1];

% Safety inflation so pruning is strictly more conservative than
% RoadRunner's own collision test (asset boxes include mirrors etc.):
vehScale = 1.25;    % multiply vehicle half-extents
vehPad   = 0.30;    % then add this margin (m)
pedPad   = 0.15;    % pedestrians: additive margin only (tight boxes)
% Model every actor as parked at its first waypoint from t=0 (in case
% actors waiting on their spawn delay are physically present):
assumePresentBeforeSpawn = true;

%% Resolve scene (same logic as nuscenes_ground_buildings)
dAll = dir(outDir + "\*_scene-*.xodr");
names = string({dAll.name});
names = names(~contains(names, "_geo"));
roster = sort(extractBefore(names, ".xodr"));
dd = dir(outDir + "\*_" + sceneId + ".xodr");
if isscalar(dd)
    [~, sceneBase] = fileparts(dd(1).name);
else
    n = str2double(extractAfter(sceneId, "scene-"));
    assert(~isempty(roster), "No converted scenes in " + outDir + ...
        " - run the nuScenes -> OpenDRIVE conversion first");
    assert(isfinite(n) && n >= 1 && n <= numel(roster), ...
        "Unknown scene '" + sceneId + "' - use scene-01..scene-" + ...
        sprintf("%02d", numel(roster)) + " or a real id like scene-0103." + ...
        newline + "Available in " + outDir + ":" + newline + ...
        join(compose("  scene-%02d = %s", (1:numel(roster))', roster(:)), newline));
    sceneBase = roster(n);
end
fprintf("Replaying agents for %s\n", sceneBase);

metaFile     = outDir + "\" + sceneBase + "_meta.json";
sceneOutDir  = outDir + "\" + sceneBase;
trajDir      = sceneOutDir + "\trajectories";
if ~isfolder(trajDir), mkdir(trajDir); end
citySceneName = replace(sceneBase, "-", "_") + "_city";
scenarioName  = replace(sceneBase, "-", "_") + "_replay";

assert(isfile(metaFile), "Missing: " + metaFile);
meta = jsondecode(fileread(metaFile));
dt = meta.frame_dt;

%% 1. Collect per-agent tracks from the frame lists
tracks = dictionary(strings(0,1), cell(0,1));   % id -> {struct(cat,t,x,y,yaw)}
frames = meta.agent_frames;         % cell array (ragged) or struct array
nF = numel(frames);
for fi = 1:nF
    if iscell(frames), fl = frames{fi}; else, fl = frames(fi); end
    t = (fi - 1) * dt;
    for a = 1:numel(fl)
        if iscell(fl), ag = fl{a}; else, ag = fl(a); end
        id = string(ag.id);
        if ~isKey(tracks, id)
            tr = struct("cat", string(ag.category), ...
                "t", [], "x", [], "y", [], "yaw", []);
        else
            tr = tracks{id};
        end
        tr.t(end+1)   = t;
        tr.x(end+1)   = ag.x;
        tr.y(end+1)   = ag.y;
        tr.yaw(end+1) = ag.yaw;
        tracks{id} = tr;
    end
end
ids = keys(tracks);
fprintf("%d agents, %d frames (%.1f s)\n", numel(ids), nF, (nF-1)*dt);

%% 1b. Make the recorded pose replayable
% Three artefacts of the raw annotations make actors rotate on the spot:
%   * yaw is an atan2 angle, so it jumps by ~2*pi whenever an agent faces
%     the +/-pi direction. RoadRunner interpolates yaw linearly between
%     waypoints, so one such jump is played back as a full spin.
%   * box orientation is noisy, so agents standing still wobble by tens of
%     degrees without going anywhere.
%   * box position is noisy too, and now and then jumps a few tens of cm
%     in a random direction. A parked agent therefore steps off on a
%     random bearing between two waypoints, and a follower that takes its
%     heading from the direction of travel swings the actor round to face
%     it - the occasional spin of a car that never goes anywhere.
% Unwrapping fixes the first; pinning heading AND position over each
% parked stretch fixes the other two, leaving the deliberate creep added
% by writeTrajCsv as the only motion there - so the direction of travel
% agrees with the heading instead of fighting it. Done here so the
% collision pruning below sees the same boxes that end up in the CSVs.
nSpin = 0;
maxShift = 0;
for k = 1:numel(ids)
    tr = tracks{ids(k)};
    was = [tr.x; tr.y; tr.yaw];
    [tr.x, tr.y, tr.yaw] = stabilizeTrack(tr.t, tr.x, tr.y, tr.yaw, ...
        stillWin, stillPosSpread, stillDrift, stillYawSpread);
    if any(abs([tr.x; tr.y; tr.yaw] - was) > 1e-6, "all"), nSpin = nSpin + 1; end
    maxShift = max(maxShift, max(hypot(tr.x - was(1,:), tr.y - was(2,:))));
    tracks{ids(k)} = tr;
end
fprintf("Pose clean-up: %d of %d tracks corrected (max shift %.2f m)\n", ...
    nSpin, numel(ids), maxShift);

% ego gets the same treatment (its yaw wraps in some scenes too)
egoP   = meta.ego_trajectory_local;
egoT   = (0:numel(egoP)-1)' * dt;
[egoX, egoY, egoYaw] = stabilizeTrack(egoT, [egoP.x]', [egoP.y]', ...
    [egoP.yaw]', stillWin, stillPosSpread, stillDrift, stillYawSpread);

%% 2. Prune trajectories that would trip the collision fail condition
% Ego is untouchable; agents are processed in order of appearance. An
% agent whose box would touch an already-accepted actor is truncated
% right before the first contact frame (RemoveTime takes it out of the
% simulation there); if the contact exists from its first frame, or the
% remainder is too short, the agent is dropped.
% The check runs on a fine interpolated timeline (RoadRunner moves actors
% by linear interpolation between waypoints, so contacts can happen
% BETWEEN the 0.5 s recorded frames).
fineDt = 0.1;
tMax = max((nF - 1), numel(egoT) - 1) * dt;
tq   = 0:fineDt:tMax;
accList = struct( ...
    "D", interpTrack(egoT, egoX, egoY, egoYaw, tq, false), ...
    "half", egoFoot/2 * vehScale + vehPad);

starts = zeros(numel(ids), 1);
for k = 1:numel(ids), tr = tracks{ids(k)}; starts(k) = tr.t(1); end
[~, ord] = sort(starts);

dropIds = strings(0, 1);
nTrunc = 0;
for k = ord'
    id = ids(k);
    tr = tracks{id};
    catKey = replace(tr.cat, ".", "_");
    if ~isfield(FOOT, catKey), continue; end    % unmapped: skipped later
    if ~includePedestrians && startsWith(catKey, "human"), continue; end
    half = FOOT.(catKey) / 2;
    if startsWith(catKey, "human")
        half = half + pedPad;
    else
        half = half * vehScale + vehPad;
    end
    D = interpTrack(tr.t', tr.x', tr.y', tr.yaw', tq, assumePresentBeforeSpawn);

    tHit = inf;
    for qi = find(~isnan(D(:,1)))'
        p = D(qi, :);
        for a = 1:numel(accList)
            q = accList(a).D(qi, :);
            if any(isnan(q)), continue; end
            if obbOverlap(p(1:2), p(3), half, q(1:2), q(3), accList(a).half)
                tHit = tq(qi);
                break
            end
        end
        if ~isinf(tHit), break; end
    end

    if ~isinf(tHit)
        if tHit < tr.t(1)                   % pre-spawn ghost was hit:
            dropIds(end+1) = id; %#ok<SAGROW> truncation cannot fix this
            continue
        end
        keep = tr.t < tHit - fineDt/2;      % end strictly before contact
        if nnz(keep) < minTrackFrames
            dropIds(end+1) = id; %#ok<SAGROW>
            continue
        end
        nTrunc = nTrunc + 1;
        tr.t = tr.t(keep); tr.x = tr.x(keep);
        tr.y = tr.y(keep); tr.yaw = tr.yaw(keep);
        tracks{id} = tr;
        D = interpTrack(tr.t', tr.x', tr.y', tr.yaw', tq, ...
            assumePresentBeforeSpawn);
    end
    accList(end+1) = struct("D", D, "half", half); %#ok<SAGROW>
end
fprintf("Collision pruning: %d dropped, %d truncated\n", ...
    numel(dropIds), nTrunc);

%% 3. Write CSV trajectories (time restarted at 0; spawn handles offset)
% Clear the folder first. Only the files listed below are imported, but a
% run with fewer agents used to leave the previous run's CSVs lying
% around, and anyone inspecting the folder afterwards reads stale
% trajectories as if they were current.
old = dir(trajDir + "\*.csv");
for i = 1:numel(old), delete(fullfile(old(i).folder, old(i).name)); end

% ego
egoCsv = trajDir + "\ego.csv";
writeTrajCsv(egoCsv, egoT, egoX, egoY, egoYaw, true);

% agents
agentCsv  = strings(0, 1);
agentOpts = {};
nCar = 0;
for k = 1:numel(ids)
    if ismember(ids(k), dropIds), continue; end
    tr = tracks{ids(k)};
    if numel(tr.t) < minTrackFrames, continue; end
    catKey = replace(tr.cat, ".", "_");
    if ~isfield(CAT, catKey), continue; end     % unmapped category: skip
    if ~includePedestrians && startsWith(catKey, "human"), continue; end
    asset = string(CAT.(catKey));
    if asset == ""                              % cars: cycle the pool
        nCar = nCar + 1;
        asset = carAssets(mod(nCar - 1, numel(carAssets)) + 1);
    end
    f = trajDir + "\" + catKey + "_" + k + ".csv";
    writeTrajCsv(f, (tr.t - tr.t(1))', tr.x', tr.y', tr.yaw', ...
        ~startsWith(catKey, "human"));
    agentCsv(end+1) = f; %#ok<SAGROW>
    agentOpts{end+1} = csvTrajectoryImportOptions( ...
        SpawnTime=tr.t(1), ...
        RemoveTime=tr.t(end), ...
        ActorImportOptions=actorImportOptions( ...
            Name=catKey + "_" + k, AssetPath=asset)); %#ok<SAGROW>
end
fprintf("Wrote %d agent trajectories + ego to %s\n", numel(agentCsv), trajDir);

%% 4. RoadRunner: open the city scene, build the replay scenario
needLaunch = true;
if exist("rrApp", "var")
    try
        status(rrApp);
        needLaunch = false;
    catch
    end
end
if needLaunch
    rrApp = roadrunner(projectFolder, InstallationFolder=installFolder);
end

% stop any active simulation - scene/scenario changes fail while
% RoadRunner is simulating ("Unable to change datasets")
try
    rrSim = createSimulation(rrApp);
    set(rrSim, "SimulationCommand", "Stop");
    pause(1);
catch
end

try
    openScene(rrApp, citySceneName);
catch
    fprintf("Scene '%s' not found - building it now ...\n", citySceneName);
    sceneIdOverride = sceneId; %#ok<NASGU> consumed inside the run() script
    run(fullfile(base, "nuscenes_ground_buildings.m"));
    clear sceneIdOverride
    openScene(rrApp, citySceneName);
end

newScenario(rrApp);

importScenario(rrApp, egoCsv, "CSV Trajectory", csvTrajectoryImportOptions( ...
    ActorImportOptions=actorImportOptions(Name="ego", ...
        AssetPath=egoAsset, Color=egoColor)));
for k = 1:numel(agentCsv)
    importScenario(rrApp, agentCsv(k), "CSV Trajectory", agentOpts{k});
end

saveScenario(rrApp, scenarioName);
fprintf("Saved scenario: %s (press Play in the Simulation tool)\n", ...
    scenarioName);

% surface any import problems RoadRunner reported (bad assets etc.)
try
    msgs = getOutputMessages(rrApp, Types=["Error", "Warning"]);
    for i = 1:numel(msgs)
        disp(msgs(i));
    end
catch
end

%% ---------------------------------------------------------------------
function [xOut, yOut, yawOut] = stabilizeTrack(t, x, y, yaw, win, ...
        posSpread, driftMax, yawSpread)
% Turn one track's recorded pose into something a linear-interpolating
% trajectory follower can replay:
%   1. unwrap yaw, so the +/-pi discontinuity of atan2 no longer reads as
%      a near-full rotation between two consecutive waypoints;
%   2. find the frames where the agent is not really going anywhere - no
%      sample within +/-win/2 seconds of it lies further than posSpread -
%      and freeze its pose across each such stretch. Freezing the position
%      is what stops a motion-derived heading from chasing the few-tens-of-
%      cm spikes in the annotations; freezing the heading stops the yaw
%      column itself from wobbling.
% Testing a window rather than one step is what makes this robust: a lone
% spike leaves the window's spread small, so it is recognised as noise
% instead of movement. A stretch is pinned in one piece up to driftMax
% across, which bounds how far any sample can be displaced and, being
% looser than posSpread, keeps a slowly wandering box in a single piece -
% splitting it would concentrate the whole wander into one visible jump.
% The heading is left alone when a stretch turns by more than yawSpread
% (a real turn on the spot rather than jitter). Shapes are preserved.
xOut = x(:)';  yOut = y(:)';  yawOut = unwrap(yaw(:))';
n = numel(yawOut);
if n >= 2
    t = t(:)';
    parked = false(1, n);
    for i = 1:n
        w = abs(t - t(i)) <= win/2;
        parked(i) = max(hypot(xOut(w) - xOut(i), yOut(w) - yOut(i))) <= posSpread;
    end
    i = 1;
    while i <= n
        if ~parked(i), i = i + 1; continue; end
        j = i;
        while j < n && parked(j+1) && ...
                boxSpread(xOut(i:j+1), yOut(i:j+1)) <= driftMax
            j = j + 1;
        end
        % Hold the pose the agent had on arrival, not the median of the
        % stretch. The median sits among samples the agent only reaches
        % later, so an arriving vehicle would overshoot it and have to
        % step backwards to settle - one frame of the actor facing the
        % wrong way, which is the very thing this is here to prevent.
        xOut(i:j) = xOut(i);
        yOut(i:j) = yOut(i);
        seg = yawOut(i:j);
        if max(seg) - min(seg) <= yawSpread
            yawOut(i:j) = median(seg);      % median: ignores stray boxes
        end
        i = j + 1;
    end
end
xOut   = reshape(xOut, size(x));
yOut   = reshape(yOut, size(y));
yawOut = reshape(yawOut, size(yaw));
end

function s = boxSpread(x, y)
% Diagonal of the bounding box - an upper bound on how far apart any two
% of the samples are, and on how far the median can be from any of them.
s = hypot(max(x) - min(x), max(y) - min(y));
end

function D = interpTrack(t, x, y, yaw, tq, padStart)
% Sample a track on the query timeline tq (linear interpolation, matching
% how RoadRunner moves actors between waypoints). NaN outside the track's
% lifetime, except padStart=true holds the first pose from t=0 until the
% track begins (actor waiting on its spawn delay). Yaw is unwrapped again
% here so the helper is safe on a raw track as well.
D = nan(numel(tq), 3);
if isempty(t), return; end
if numel(t) < 2
    in = abs(tq - t(1)) < 1e-9;
    D(in, :) = repmat([x(1), y(1), yaw(1)], nnz(in), 1);
else
    in = tq >= t(1) - 1e-9 & tq <= t(end) + 1e-9;
    D(in, 1) = interp1(t, x, tq(in), "linear", "extrap");
    D(in, 2) = interp1(t, y, tq(in), "linear", "extrap");
    D(in, 3) = interp1(t, unwrap(yaw), tq(in), "linear", "extrap");
end
if padStart
    pre = tq < t(1) - 1e-9;
    D(pre, :) = repmat([x(1), y(1), yaw(1)], nnz(pre), 1);
end
end

function tf = obbOverlap(c1, yaw1, h1, c2, yaw2, h2)
% Separating-axis test for two oriented 2-D boxes (centers, yaws,
% half-extents [hx hy]). True if the boxes intersect.
u1 = [cos(yaw1), sin(yaw1)];  v1 = [-sin(yaw1), cos(yaw1)];
u2 = [cos(yaw2), sin(yaw2)];  v2 = [-sin(yaw2), cos(yaw2)];
d = c2 - c1;
axesAll = [u1; v1; u2; v2];
tf = true;
for a = 1:4
    ax = axesAll(a, :);
    r1 = h1(1)*abs(ax*u1') + h1(2)*abs(ax*v1');
    r2 = h2(1)*abs(ax*u2') + h2(2)*abs(ax*v2');
    if abs(ax*d') > r1 + r2
        tf = false;
        return
    end
end
end

function writeTrajCsv(file, t, x, y, yaw, isVehicle)
% RoadRunner "CSV Trajectory" format: header names are case-sensitive;
% x/y in metres, time in seconds, yaw in radians - continuous (unwrapped,
% so possibly outside +/-pi), never the raw atan2 angle, or the follower
% spins the actor the long way round at every wrap.
% A step that does not carry the agent at least minStep along its own
% heading is rebuilt so that it points exactly that way. This covers, in
% one rule, every step whose direction says nothing reliable about where
% the agent is facing: steps of a few cm (pure annotation noise), steps
% that go sideways, and steps that go backwards - a car settling into a
% parking spot overshoots and steps back, which a follower that takes its
% heading from the direction of travel plays as a 180-degree flip. It
% also keeps consecutive waypoints distinct, which the follower requires.
% Section 1b has already pinned whole parked stretches; this catches the
% ragged edges, mostly the last metre as a vehicle comes to rest.
% The distance actually travelled along the heading is preserved (floored
% at creep), so a vehicle easing forward keeps its pace and a stationary
% one barely moves. This does assume nothing reverses for long: a genuine
% reversing manoeuvre would be flattened to a crawl. That holds here - of
% 9162 vehicle steps in the mini set only 8 run more than 0.30 m against
% the heading, and no two of them are consecutive.
% Vehicles get one more test: a step may not run more than maxSkew off
% the heading, because a car cannot travel sideways. That catches the
% sideways teleports in the raw boxes, and the step out of a pinned
% stretch, which carries however far the box drifted while parked. The
% actor is left off its recorded line by the lateral part that was
% dropped, and rejoins within a frame or two as soon as a well-aligned
% step arrives. Pedestrians are exempt: theirs is a body orientation and
% people really do step sideways. The limit is loose enough for any
% genuine turn: a step tilts by about half the turn taken across it, and
% vehicle yaw never moves more than 33 degrees per frame here, so real
% motion stays inside ~17 degrees and the limit has room to spare.
minStep = 0.15;             % m - below this a step direction is not trustworthy
creep   = 0.002;            % m - advance for a waypoint that would not move
maxSkew = deg2rad(30);
for i = 2:numel(x)
    a   = yaw(i-1);
    dx  = x(i) - x(i-1);
    dy  = y(i) - y(i-1);
    adv = dx*cos(a) + dy*sin(a);        % distance made good along the heading
    lat = -dx*sin(a) + dy*cos(a);       % and the part that goes sideways
    if adv >= minStep && ~(isVehicle && abs(atan2(lat, adv)) > maxSkew)
        continue                        % real motion: leave the step alone
    end
    adv  = max(adv, creep);
    x(i) = x(i-1) + adv*cos(a);
    y(i) = y(i-1) + adv*sin(a);
end
T = table(t, x, y, yaw, VariableNames=["time", "x", "y", "yaw"]);
writetable(T, file);
end
