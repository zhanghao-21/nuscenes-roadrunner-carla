%% replay_on_full.m
% Replay any of the four Boston nuScenes-mini scenarios (scene-0103,
% scene-0553, scene-0655, scene-0757) on the FULL boston-seaport scene
% built by build_full_scene.m. Same replay pipeline as replay_agents.m;
% the only difference is a constant offset that maps each scenario's
% patch-local coordinates into the full-region frame.

%% Config
% base / sceneIdOverride / installFolder can be pre-set by the caller
if ~exist("base", "var")
base    = string(fileparts(fileparts(mfilename("fullpath"))));   % repo root
end
fullDir = base + "\fullmap";
outDir  = base + "\output_roadrunner";      % per-scenario sidecars live here
if exist("sceneIdOverride", "var")
    sceneId = sceneIdOverride;
else
sceneId = "scene-0103";     % scene-0103 | scene-0553 | scene-0655 | scene-0757
end
                            % (or an index "scene-01".."scene-04" into the
                            % sorted Boston list)

projectFolder = base + "\nuscenes_test1";
if ~exist("installFolder", "var")
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";
end

carAssets = ["Vehicles/Sedan.fbx", "Vehicles/Suv.fbx", ...
             "Vehicles/CompactCar.fbx"];
CAT = struct( ...
    "vehicle_car",              "",  ...
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
minTrackFrames = 2;
includePedestrians = true;

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

vehScale = 1.25;
vehPad   = 0.30;
pedPad   = 0.15;
assumePresentBeforeSpawn = true;

%% Resolve the Boston scenario and its offset into the full region
dAll = dir(outDir + "\boston-seaport_scene-*.xodr");
names = string({dAll.name});
names = names(~contains(names, "_geo"));
roster = sort(extractBefore(names, ".xodr"));
dd = dir(outDir + "\boston-seaport_" + sceneId + ".xodr");
if isscalar(dd)
    [~, sceneBase] = fileparts(dd(1).name);
else
    n = str2double(extractAfter(sceneId, "scene-"));
    assert(isfinite(n) && n >= 1 && n <= numel(roster), ...
        "Unknown Boston scene '" + sceneId + "'. Available: " + ...
        join(roster, ", "));
    sceneBase = roster(n);
end
fprintf("Replaying %s on the full boston-seaport scene\n", sceneBase);

metaFile     = outDir + "\" + sceneBase + "_meta.json";
fullMetaFile = fullDir + "\boston-seaport_full_meta.json";
assert(isfile(metaFile),     "Missing: " + metaFile);
assert(isfile(fullMetaFile), "Missing: " + fullMetaFile);
meta     = jsondecode(fileread(metaFile));
fullMeta = jsondecode(fileread(fullMetaFile));
dt = meta.frame_dt;

% scenario patch-local -> full-region frame
off = [meta.origin.x - fullMeta.origin.x, ...
       meta.origin.y - fullMeta.origin.y];
fprintf("Scenario offset in full frame: [%.1f, %.1f] m\n", off(1), off(2));

trajDir = fullDir + "\trajectories\" + sceneBase;
if ~isfolder(trajDir), mkdir(trajDir); end
citySceneName = "boston_seaport_full_city";
scenarioName  = replace(sceneBase, "-", "_") + "_replay_full";

%% 1. Collect per-agent tracks (already shifted into the full frame)
tracks = dictionary(strings(0,1), cell(0,1));
frames = meta.agent_frames;
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
        tr.x(end+1)   = ag.x + off(1);
        tr.y(end+1)   = ag.y + off(2);
        tr.yaw(end+1) = ag.yaw;
        tracks{id} = tr;
    end
end
ids = keys(tracks);
fprintf("%d agents, %d frames (%.1f s)\n", numel(ids), nF, (nF-1)*dt);

%% 2. Prune trajectories that would trip the collision fail condition
fineDt = 0.1;
ego  = meta.ego_trajectory_local;
egoX = [ego.x]' + off(1);
egoY = [ego.y]' + off(2);
egoW = [ego.yaw]';
tMax = max((nF - 1), numel(ego) - 1) * dt;
tq   = 0:fineDt:tMax;
egoT = (0:numel(ego)-1)' * dt;
accList = struct( ...
    "D", interpTrack(egoT, egoX, egoY, egoW, tq, false), ...
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
    if ~isfield(FOOT, catKey), continue; end
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
        if tHit < tr.t(1)
            dropIds(end+1) = id; %#ok<SAGROW>
            continue
        end
        keep = tr.t < tHit - fineDt/2;
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
egoCsv = trajDir + "\ego.csv";
writeTrajCsv(egoCsv, egoT, egoX, egoY, egoW);

agentCsv  = strings(0, 1);
agentOpts = {};
nCar = 0;
for k = 1:numel(ids)
    if ismember(ids(k), dropIds), continue; end
    tr = tracks{ids(k)};
    if numel(tr.t) < minTrackFrames, continue; end
    catKey = replace(tr.cat, ".", "_");
    if ~isfield(CAT, catKey), continue; end
    if ~includePedestrians && startsWith(catKey, "human"), continue; end
    asset = string(CAT.(catKey));
    if asset == ""
        nCar = nCar + 1;
        asset = carAssets(mod(nCar - 1, numel(carAssets)) + 1);
    end
    f = trajDir + "\" + catKey + "_" + k + ".csv";
    writeTrajCsv(f, (tr.t - tr.t(1))', tr.x', tr.y', tr.yaw');
    agentCsv(end+1) = f; %#ok<SAGROW>
    agentOpts{end+1} = csvTrajectoryImportOptions( ...
        SpawnTime=tr.t(1), ...
        RemoveTime=tr.t(end), ...
        ActorImportOptions=actorImportOptions( ...
            Name=catKey + "_" + k, AssetPath=asset)); %#ok<SAGROW>
end
fprintf("Wrote %d agent trajectories + ego to %s\n", numel(agentCsv), trajDir);

%% 4. RoadRunner: open the full scene, build the replay scenario
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
    run(fullfile(fullDir, "build_full_scene.m"));
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

try
    msgs = getOutputMessages(rrApp, Types=["Error", "Warning"]);
    for i = 1:numel(msgs)
        disp(msgs(i));
    end
catch
end

%% ---------------------------------------------------------------------
function D = interpTrack(t, x, y, yaw, tq, padStart)
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

function writeTrajCsv(file, t, x, y, yaw)
% RoadRunner "CSV Trajectory" format; 2 mm-per-frame creep keeps
% consecutive waypoints distinct for parked/stopped actors.
s = 0.002 * (0:numel(x)-1)';
x = x + s .* cos(yaw);
y = y + s .* sin(yaw);
T = table(t, x, y, yaw, VariableNames=["time", "x", "y", "yaw"]);
writetable(T, file);
end
