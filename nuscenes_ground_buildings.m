%% nuscenes_ground_buildings.m
% Build a RoadRunner scene with:
%   1. the nuScenes scene-0103 OpenDRIVE road network,
%   2. REAL lane markings from the nuScenes map expansion layers
%      (lane_divider -> dashed white, road_divider -> double yellow),
%      drawn as HD-map curve markings,
%   3. traffic signals from the nuScenes traffic_light layer (surveyed
%      position + facing direction), assembled like FourWaySignal.rrscene:
%      vertical post + mast arm + 3-light heads,
%   4. a large level ground slab,
%   5. buildings from the aligned OSM data (no OSM roads).
%
% Markings and signals come from the SAME nuScenes map frame as the lanes,
% so they are aligned by construction (no manualShift needed for them).
% Only the OSM buildings use manualShift.
%
% Ground + markings + signals + buildings travel in one .rrhd file.
% The .rrhd import requires a RoadRunner Scene Builder license.

%% Config
% every path below can be pre-set by the caller (e.g. main.py via
% matlab -batch "base='...'; run('nuscenes_ground_buildings.m')")
if ~exist("base", "var")
base    = string(fileparts(mfilename("fullpath")));   % this script's folder
end
if exist("sceneIdOverride", "var")
    sceneId = sceneIdOverride;  % set by replay_agents when it auto-builds
else
sceneId = "scene-01";       % "scene-01".."scene-10": index into the sorted
                            % scene list (printed below); real nuScenes ids
                            % ("scene-0103" etc.) also work
end
outDir      = base + "\output_roadrunner";      % converter outputs (inputs)
if ~exist("nuscMapsDir", "var")
nuscMapsDir = base + "\v1.0-mini\maps\expansion";
end

% roster of available scenes, sorted by name (skip generated _geo copies)
dAll = dir(outDir + "\*_scene-*.xodr");
names = string({dAll.name});
names = names(~contains(names, "_geo"));
roster = sort(extractBefore(names, ".xodr"));
fprintf("Available scenes:\n");
for i = 1:numel(roster), fprintf("  scene-%02d = %s\n", i, roster(i)); end

% resolve sceneId: exact nuScenes id first, then simple index
dd = dir(outDir + "\*_" + sceneId + ".xodr");
if isscalar(dd)
    [~, sceneBase] = fileparts(dd(1).name);     % e.g. boston-seaport_scene-0103
else
    n = str2double(extractAfter(sceneId, "scene-"));
    assert(isfinite(n) && n >= 1 && n <= numel(roster), ...
        "Unknown scene '" + sceneId + "' - use scene-01..scene-" + ...
        sprintf("%02d", numel(roster)) + " or a real id like scene-0103");
    sceneBase = roster(n);
end
mapName = extractBefore(sceneBase, "_scene-");
fprintf("Selected: %s (map: %s)\n", sceneBase, mapName);

% inputs are read from outDir; everything generated goes into a
% per-scene subfolder
sceneXodr   = outDir + "\" + sceneBase + ".xodr";
osmFile     = outDir + "\" + sceneBase + ".osm";
metaFile    = outDir + "\" + sceneBase + "_meta.json";
mapJson     = nuscMapsDir + "\" + mapName + ".json";
sceneOutDir = outDir + "\" + sceneBase;
if ~isfolder(sceneOutDir), mkdir(sceneOutDir); end
rrhdFile  = sceneOutDir + "\" + sceneBase + "_city.rrhd";
markCache = sceneOutDir + "\" + sceneBase + "_marks.mat";

projectFolder = base + "\nuscenes_test1";
if ~exist("installFolder", "var")
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";
end
sceneName     = replace(sceneBase, "-", "_") + "_city";

% Per-scene OSM->nuScenes calibration ([east north] m). Only scene-0103 is
% calibrated so far; scenes without an entry use [0 0] (buildings may be a
% few metres off). To calibrate a scene: run it, measure the offset of a
% building vs the roads in the preview figure, add an entry, re-run.
SHIFT = struct( ...
    "scene_0103", [0 0]);
shiftKey = replace(extractAfter(sceneBase, mapName + "_"), "-", "_");
if isfield(SHIFT, shiftKey)
    manualShift = SHIFT.(shiftKey);
else
    manualShift = [0 0];
    fprintf("Note: %s has no OSM calibration yet (manualShift = [0 0]).\n", ...
        sceneBase);
end

groundMargin = 120;              % slab margin beyond the patch (m)
groundZ      = -0.15;            % slab sits just below the roads
slabLaneW    = 12;               % width of each flat slab lane (m)
markZ        = 0.02;             % markings drawn just above the road

defaultHeight = 10;              % m, when OSM has no height info
levelHeight   = 3.2;             % m per building:levels
buildingAssets = [ ...
    "Assets/Buildings/Downtown_15mX15m_01_5storey.fbx"
    "Assets/Buildings/Downtown_15mX15m_02_5storey.fbx"
    "Assets/Buildings/Downtown_30mX15m_01_5storey.fbx"];
buildingMinFill    = 0.55;   % split footprints whose box fill-ratio is lower
buildingSplitDepth = 3;      % max recursive splits for concave footprints
roadClearance      = 2.0;    % m: keep building boxes this far off lane centers
minBldgScale       = 0.8;   % floor for shrinking road-overlapping buildings
warning("off", "MATLAB:polyshape:repairedBySimplify");

laneDividerAsset = "Assets/Markings/DashedSingleWhite.rrlms";
roadDividerAsset = "Assets/Markings/SolidDoubleYellow.rrlms";

% Traffic-light assembly copied from FourWaySignal.rrscene: a vertical
% post + a mast arm over the road + 3-light heads hanging from the arm.
% half-extents below match the measured FBX geometry so the assets keep
% their natural proportions when scaled into the bounding boxes
signalPostAsset = "Assets/Props/Signals/Signal_Post_30ft.fbx";
signalPostHalf  = [0.28 0.28 4.6];  % 9.2 m pole
signalArmAsset  = "Assets/Props/Signals/Signal_MastArm_25ft.fbx";
signalArmHalf   = [3.88 0.15 0.65]; % 7.8 m arm; spans local +X from pole
signalArmZ      = 5.4;              % arm mounting height (center, m)
signalHeadAsset = "Assets/Props/Signals/Signal_3Light_Post01.fbx";
signalHeadHalf  = [0.26 0.32 0.58];
signalHeadZ     = 4.9;              % head center height (below the arm)
signalHeadFrac  = [0.5 0.85];     % head positions as fraction of arm span
signalYawOffset = 0;              % rad: global facing tweak (try +-pi/2, pi)
signalSource    = "xodr";         % "xodr" (<signal> entries in the xodr),
                                  % "osm" (traffic_signals nodes), "nuscenes"

assert(isfile(metaFile), "Missing: " + metaFile);
assert(isfile(mapJson),  "Missing: " + mapJson);

% published nuScenes map origins (lat/lon of map coordinate (0,0)),
% same values as nuscenes2xodr
REF = struct( ...
    "boston_seaport",           [42.336849169438615, -71.05785369873047], ...
    "singapore_onenorth",       [1.2882100868743724, 103.78475189208984], ...
    "singapore_hollandvillage", [1.2993652317780957, 103.78217697143555], ...
    "singapore_queenstown",     [1.2782562240223188, 103.76741409301758]);
refLL = REF.(replace(mapName, "-", "_"));
lat0 = refLL(1);  lon0 = refLL(2);

% patch corner / extent from the scene sidecar
meta = jsondecode(fileread(metaFile));
xmin = meta.patch.x_min;  ymin = meta.patch.y_min;
patchExtent = [meta.patch.x_max - xmin, meta.patch.y_max - ymin];

EARTH_R = 6378137.0;
latO = lat0 + (ymin / EARTH_R) * (180/pi);
lonO = lon0 + (xmin / (EARTH_R * cosd(lat0))) * (180/pi);

% download the OSM extract for this scene's area if not there yet;
% failure is non-fatal (the scene just builds without buildings)
if ~isfile(osmFile)
    pad  = 0.0018;   % deg, ~200 m
    latS = lat0 + (meta.patch.y_min / EARTH_R) * (180/pi) - pad;
    latN = lat0 + (meta.patch.y_max / EARTH_R) * (180/pi) + pad;
    lonW = lon0 + (meta.patch.x_min / (EARTH_R*cosd(lat0))) * (180/pi) - pad;
    lonE = lon0 + (meta.patch.x_max / (EARTH_R*cosd(lat0))) * (180/pi) + pad;
    url = sprintf("https://api.openstreetmap.org/api/0.6/map?bbox=%.7f,%.7f,%.7f,%.7f", ...
        lonW, latS, lonE, latN);
    fprintf("Downloading OSM extract for %s ...\n", sceneBase);
    try
        % force XML: MATLAB's default Accept header makes the API send JSON
        websave(osmFile, url, weboptions(Timeout=180, ...
            HeaderFields={'Accept', 'application/xml'}));
    catch ex
        warning("OSM download failed (%s) - continuing without buildings.", ...
            ex.message);
        if isfile(osmFile), delete(osmFile); end   % no partial downloads
    end
end

%% 1. nuScenes map layers: dividers, stop lines, traffic lights
M = extractNuscMarks(mapJson, markCache, [xmin ymin], ...
    [-10, patchExtent(1)+10, -10, patchExtent(2)+10]);
fprintf("nuScenes layers in patch: %d lane dividers, %d road dividers, " + ...
    "%d stop lines, %d traffic lights\n", ...
    numel(M.LD), numel(M.RD), numel(M.SL), size(M.TL, 1));


%% 2. OSM building footprints (patch frame) - optional
roadPts = xodrRoadSamples(sceneXodr, false);    % ~1 m road samples [x y hdg]
mainPts = xodrRoadSamples(sceneXodr, true);     % non-junction roads only
assert(~isempty(mainPts), "No road samples parsed from " + sceneXodr);

lim = [-groundMargin, patchExtent(1)+groundMargin, ...
       -groundMargin, patchExtent(2)+groundMargin];
fp = {};
fh = [];
osmSignals = zeros(0, 2);
if ~isfile(osmFile)
    disp("No OSM data - scene will have no buildings.");
else
S = readstruct(osmFile, FileType="xml");
nodeId  = [S.node.idAttribute];
nodeLat = [S.node.latAttribute];
nodeLon = [S.node.lonAttribute];
id2idx  = containers.Map(nodeId, 1:numel(nodeId));

% traffic_signals nodes (used when signalSource == "osm"); OSM nodes have
% no orientation, so facing is derived from the nearest lane later
for i = 1:numel(S.node)
    n = S.node(i);
    if ~isfield(n, "tag") || ~isstruct(n.tag) || isempty(n.tag), continue; end
    ks = string({n.tag.kAttribute});
    vs = string({n.tag.vAttribute});
    if any(ks == "highway" & vs == "traffic_signals")
        x = deg2rad(n.lonAttribute - lonO) * EARTH_R * cosd(lat0) + manualShift(1);
        y = deg2rad(n.latAttribute - latO) * EARTH_R            + manualShift(2);
        if x > lim(1) && x < lim(2) && y > lim(3) && y < lim(4)
            osmSignals(end+1, :) = [x y]; %#ok<SAGROW>
        end
    end
end

for i = 1:numel(S.way)
    w = S.way(i);
    if ~isfield(w, "tag") || ~isstruct(w.tag) || isempty(w.tag), continue; end
    ks = string({w.tag.kAttribute});
    vs = string({w.tag.vAttribute});
    if ~any(ks == "building"), continue; end

    refs = [w.nd.refAttribute];
    ok = arrayfun(@(r) isKey(id2idx, r), refs);
    if nnz(ok) < 3, continue; end
    idx = cell2mat(values(id2idx, num2cell(refs(ok))));

    x = deg2rad(nodeLon(idx) - lonO) * EARTH_R * cosd(lat0) + manualShift(1);
    y = deg2rad(nodeLat(idx) - latO) * EARTH_R            + manualShift(2);
    if all(x < lim(1)) || all(x > lim(2)) || all(y < lim(3)) || all(y > lim(4))
        continue
    end
    h = defaultHeight;
    k = find(ks == "height", 1);
    if ~isempty(k)
        v = str2double(regexp(vs(k), "[\d.]+", "match", "once"));
        if isfinite(v) && v > 0, h = v; end
    else
        k = find(ks == "building:levels", 1);
        if ~isempty(k)
            v = str2double(regexp(vs(k), "[\d.]+", "match", "once"));
            if isfinite(v) && v > 0, h = v * levelHeight; end
        end
    end
    fp{end+1} = [x(:) y(:)]; %#ok<SAGROW>
    fh(end+1) = h;           %#ok<SAGROW>
end
end   % if isfile(osmFile)
fprintf("OSM buildings in area: %d\n", numel(fp));

% pick the signal source: [x y facing], facing = NaN when unknown
xodrSig = xodrSignalPositions(sceneXodr);
if signalSource == "xodr" && ~isempty(xodrSig)
    signals = xodrSig;
    fprintf("Signals: %d from xodr <signal> entries\n", size(signals, 1));
elseif signalSource == "osm" && ~isempty(osmSignals)
    signals = [osmSignals, nan(size(osmSignals, 1), 1)];
    fprintf("Signals: %d from OSM traffic_signals nodes\n", size(signals, 1));
elseif ~isempty(M.TL)
    signals = M.TL;
    fprintf("Signals: %d from nuScenes traffic_light layer\n", size(signals, 1));
else
    signals = xodrSig;
    fprintf("Signals: %d from xodr <signal> entries (fallback)\n", ...
        size(signals, 1));
end

%% 3. Build the HD map
map = roadrunnerHDMap;
map.GeoReference = [latO lonO];

% --- ground slab: parallel flat Sidewalk lanes drawn west -> east
xs = lim(1);  xe = lim(2);
y0 = lim(3);  ye = lim(4);
nRows = ceil((ye - y0) / slabLaneW);
for b = 0:nRows
    yb = y0 + b * slabLaneW;
    map.LaneBoundaries(b+1) = roadrunner.hdmap.LaneBoundary( ...
        ID="GB" + b, Geometry=[xs yb groundZ; xe yb groundZ]);
end
for j = 1:nRows
    yc = y0 + (j - 0.5) * slabLaneW;
    map.Lanes(j) = roadrunner.hdmap.Lane(ID="GL" + j, ...
        Geometry=[xs yc groundZ; xe yc groundZ], ...
        TravelDirection="Forward", LaneType="Sidewalk");
    leftBoundary (map.Lanes(j), "GB" + j,     Alignment="Forward");
    rightBoundary(map.Lanes(j), "GB" + (j-1), Alignment="Forward");
end

% --- curve markings from the nuScenes layers
% CurveMarking/CurveMarkingType were added in R2024b.  Keep stage 1 usable
% with older university-managed RoadRunner installations by omitting only
% these custom divider curves when that API is unavailable.
hasCurveMarkings = isprop(map, "CurveMarkingTypes") && ...
    ~isempty(which("roadrunner.hdmap.CurveMarkingType")) && ...
    ~isempty(which("roadrunner.hdmap.CurveMarking"));
if hasCurveMarkings
    map.CurveMarkingTypes(1) = roadrunner.hdmap.CurveMarkingType(ID="MT_LD", ...
        AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=laneDividerAsset));
    map.CurveMarkingTypes(2) = roadrunner.hdmap.CurveMarkingType(ID="MT_RD", ...
        AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=roadDividerAsset));
    nMark = 0;
    for k = 1:numel(M.LD)
        nMark = nMark + 1;
        map.CurveMarkings(nMark) = roadrunner.hdmap.CurveMarking(ID="LD" + k, ...
            Geometry=[M.LD{k}, repmat(markZ, size(M.LD{k},1), 1)], ...
            MarkingTypeReference=roadrunner.hdmap.Reference(ID="MT_LD"));
    end
    for k = 1:numel(M.RD)
        nMark = nMark + 1;
        map.CurveMarkings(nMark) = roadrunner.hdmap.CurveMarking(ID="RD" + k, ...
            Geometry=[M.RD{k}, repmat(markZ, size(M.RD{k},1), 1)], ...
            MarkingTypeReference=roadrunner.hdmap.Reference(ID="MT_RD"));
    end
else
    warning("nuScenes:CurveMarkingsUnavailable", ...
        "CurveMarking API is unavailable in this MATLAB/RoadRunner installation. Continuing without %d lane-divider and %d road-divider curves.", ...
        numel(M.LD), numel(M.RD));
end
% --- asset types for props
for a = 1:numel(buildingAssets)
    map.StaticObjectTypes(a) = roadrunner.hdmap.StaticObjectType( ...
        ID="BldgType" + a, ...
        AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=buildingAssets(a)));
end
map.StaticObjectTypes(end+1) = roadrunner.hdmap.StaticObjectType( ...
    ID="SigPostType", ...
    AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=signalPostAsset));
map.StaticObjectTypes(end+1) = roadrunner.hdmap.StaticObjectType( ...
    ID="SigArmType", ...
    AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=signalArmAsset));
map.StaticObjectTypes(end+1) = roadrunner.hdmap.StaticObjectType( ...
    ID="SigHeadType", ...
    AssetPath=roadrunner.hdmap.RelativeAssetPath(AssetPath=signalHeadAsset));

% --- buildings: concave/L-shaped footprints are decomposed into several
% boxes (a single min-area rect would spill into neighbors and roads)
nObj = 0;
nShrunk = 0;
allBoxes = zeros(0, 5);                 % [cx cy hx hy yaw] for the preview
for k = 1:numel(fp)
    boxes = footprintBoxes(fp{k}, buildingMinFill, buildingSplitDepth);
    for j = 1:size(boxes, 1)
        % if the box covers road, scale it down (about its center) until
        % it clears the lanes - never drop the building entirely
        [newHalf, adj] = shrinkBoxOffRoad(boxes(j,1:2), boxes(j,3:4), ...
            boxes(j,5), roadPts, roadClearance, minBldgScale);
        if adj
            boxes(j,3:4) = newHalf;
            nShrunk = nShrunk + 1;
        end
        nObj = nObj + 1;
        map.StaticObjects(nObj) = roadrunner.hdmap.StaticObject( ...
            ID="Bldg" + k + "_" + j, ...
            Geometry=roadrunner.hdmap.GeoOrientedBoundingBox( ...
                Center=[boxes(j,1:2), fh(k)/2], ...
                Dimension=[boxes(j,3:4), fh(k)/2], ...
                GeoOrientation=[0 0 boxes(j,5)]), ...   % radians
            ObjectTypeReference=roadrunner.hdmap.Reference( ...
                ID="BldgType" + (mod(k-1, numel(buildingAssets)) + 1)));
    end
    allBoxes = [allBoxes; boxes]; %#ok<AGROW>
end
if nShrunk > 0
    fprintf("%d building boxes shrunk to clear the road\n", nShrunk);
end

% --- traffic signals, assembled like FourWaySignal.rrscene.
% The nuScenes traffic_light record marks the LIGHT position (at or over
% the road), not a pole position. So: pole is walked outward from the
% recorded point to the roadside (first spot clear of the drivable area),
% the mast arm spans from the pole back over the road, and the heads hang
% under the arm near the recorded point, facing the recorded direction.
sigPole = zeros(size(signals, 1), 2);
sigU    = zeros(size(signals, 1), 2);
sigF    = zeros(size(signals, 1), 1);
wrapAng = @(x) mod(x + pi, 2*pi) - pi;
for k = 1:size(signals, 1)
    pos = signals(k, 1:2);
    f   = signals(k, 3);                            % recorded facing

    % Snap the facing to the controlled lane. Each converter road is a
    % single forward lane, so sample heading = travel direction. The
    % record picks WHICH lane (parallel axis, preferring its own sign);
    % the lane then dictates the exact facing = against travel.
    d2h  = hypot(mainPts(:,1) - pos(1), mainPts(:,2) - pos(2));
    cand = find(d2h < 8);
    if isnan(f)
        % no recorded orientation (OSM node): face against the nearest lane
        if ~isempty(cand)
            [~, ci] = min(d2h(cand));
            f = mainPts(cand(ci), 3) + pi;
        else
            [~, ni] = min(d2h);         % nothing within 8 m: nearest anyway
            f = mainPts(ni, 3) + pi;
        end
    elseif ~isempty(cand)
        a = abs(wrapAng(mainPts(cand,3) - (f + pi)));
        same = cand(a < pi/3);          % lanes travelling toward the light
        opp  = cand(a > 2*pi/3);        % parallel but record sign flipped
        if ~isempty(same)
            [~, ci] = min(d2h(same));
            f = mainPts(same(ci), 3) + pi;
        elseif ~isempty(opp)
            [~, ci] = min(d2h(opp));
            f = mainPts(opp(ci), 3) + pi;
        end
    end
    sigF(k) = f;
    u = [cos(f + pi/2), sin(f + pi/2)];             % across the road
    % orient u so that the -u side is the off-road side (pole side)
    dPlus  = min(hypot(roadPts(:,1) - (pos(1) + 6*u(1)), ...
                       roadPts(:,2) - (pos(2) + 6*u(2))));
    dMinus = min(hypot(roadPts(:,1) - (pos(1) - 6*u(1)), ...
                       roadPts(:,2) - (pos(2) - 6*u(2))));
    if dPlus > dMinus, u = -u; end
    % walk the pole out of the drivable area (road samples are lane
    % centers of ~4 m lanes, so >= 3.2 m clearance is past the edge)
    s = 0;
    poleP = pos;
    while s < 12
        poleP = pos - u * s;
        if min(hypot(roadPts(:,1) - poleP(1), roadPts(:,2) - poleP(2))) >= 3.2
            break
        end
        s = s + 0.5;
    end
    sigPole(k, :) = poleP;
    sigU(k, :)    = u;
    armYaw = atan2(u(2), u(1)) + signalYawOffset;

    nObj = nObj + 1;                                % vertical post (roadside)
    map.StaticObjects(nObj) = roadrunner.hdmap.StaticObject( ...
        ID="SigPost" + k, ...
        Geometry=roadrunner.hdmap.GeoOrientedBoundingBox( ...
            Center=[poleP, signalPostHalf(3)], ...
            Dimension=signalPostHalf, ...
            GeoOrientation=[0 0 armYaw]), ...
        ObjectTypeReference=roadrunner.hdmap.Reference(ID="SigPostType"));

    nObj = nObj + 1;                                % mast arm over the road
    map.StaticObjects(nObj) = roadrunner.hdmap.StaticObject( ...
        ID="SigArm" + k, ...
        Geometry=roadrunner.hdmap.GeoOrientedBoundingBox( ...
            Center=[poleP + u * signalArmHalf(1), signalArmZ], ...
            Dimension=signalArmHalf, ...
            GeoOrientation=[0 0 armYaw]), ...
        ObjectTypeReference=roadrunner.hdmap.Reference(ID="SigArmType"));

    % heads: outermost one over the recorded light position (clamped to
    % the arm span), second one at 60% of that distance
    dTip  = 2 * signalArmHalf(1) - 0.6;
    dHead = min(max(s, 4), dTip);
    for hIdx = 1:numel(signalHeadFrac)
        headCenter = poleP + u * (dHead * signalHeadFrac(hIdx) / max(signalHeadFrac));
        nObj = nObj + 1;
        map.StaticObjects(nObj) = roadrunner.hdmap.StaticObject( ...
            ID="SigHead" + k + "_" + hIdx, ...
            Geometry=roadrunner.hdmap.GeoOrientedBoundingBox( ...
                Center=[headCenter, signalHeadZ], ...
                Dimension=signalHeadHalf, ...
                GeoOrientation=[0 0 f - pi/2 + signalYawOffset]), ...  % head
                ...                     % shines along local +Y (FBX probe)
            ObjectTypeReference=roadrunner.hdmap.Reference(ID="SigHeadType"));
    end
end

write(map, rrhdFile);
disp("Wrote " + rrhdFile);

% quick preview
figure; hold on; axis equal; grid on
rectangle(Position=[xs y0 xe-xs ye-y0], EdgeColor=[0.5 0.5 0.5]);
plot(roadPts(:,1), roadPts(:,2), ".", MarkerSize=3, Color=[0.7 0.7 0.9]);
for k = 1:numel(fp)
    p = fp{k};
    plot(p([1:end 1],1), p([1:end 1],2), "-", Color=[0.85 0.4 0.1]);
end
for k = 1:size(allBoxes, 1)             % boxes actually placed (cyan)
    c = allBoxes(k,1:2); hf = allBoxes(k,3:4); yw = allBoxes(k,5);
    R = [cos(yw) -sin(yw); sin(yw) cos(yw)];
    cn = (R * ([1 1; 1 -1; -1 -1; -1 1; 1 1] .* hf)')' + c;
    plot(cn(:,1), cn(:,2), "-", Color=[0.3 0.8 0.9]);
end
for k = 1:numel(M.LD), plot(M.LD{k}(:,1), M.LD{k}(:,2), "w--"); end
for k = 1:numel(M.RD), plot(M.RD{k}(:,1), M.RD{k}(:,2), "y-", LineWidth=1.5); end
quiver(signals(:,1), signals(:,2), 6*cos(sigF), 6*sin(sigF), ...
    0, "y", LineWidth=1.5);
plot(signals(:,1), signals(:,2), "^", MarkerSize=8, ...
    MarkerFaceColor="y", MarkerEdgeColor="k");
for k = 1:size(signals, 1)                          % pole + arm span
    tip = sigPole(k,:) + sigU(k,:) * 2 * signalArmHalf(1);
    plot([sigPole(k,1) tip(1)], [sigPole(k,2) tip(2)], "m-", LineWidth=1.5);
    plot(sigPole(k,1), sigPole(k,2), "ms", MarkerSize=8, MarkerFaceColor="m");
end
set(gca, Color=[0.2 0.2 0.2]);
title(sprintf("%d buildings, %d markings, %d signals (patch frame)", ...
    numel(fp), nMark, size(signals, 1)));

%% 4. RoadRunner: nuScenes roads first, then the HD map content
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

% stop any active simulation - scene changes fail while simulating
try
    rrSim = createSimulation(rrApp);
    set(rrSim, "SimulationCommand", "Stop");
    pause(1);
catch
end

newScene(rrApp);
changeWorldSettings(rrApp, WorldOrigin=[latO lonO]);

% 4a. nuScenes OpenDRIVE (ImportSignals=false: signals are placed above)
sceneXodrGeo = ensureGeoRef(sceneXodr, latO, lonO, sceneOutDir);
importScene(rrApp, sceneXodrGeo, "OpenDRIVE", openDriveImportOptions( ...
    ImportSignals=false, ImportObjects=true, ProjectionMode="FullProjection"));

% 4b. ground + markings + buildings + signals; keep the existing roads
buildOpts = roadrunnerHDMapBuildOptions( ...
    ClearSceneOfExistingData=false, ...
    EnableOverlapGroupsOptions=enableOverlapGroupsOptions(IsEnabled=false));
importScene(rrApp, rrhdFile, "RoadRunner HD Map", ...
    roadrunnerHDMapImportOptions(BuildOptions=buildOpts));

saveScene(rrApp, sceneName);
disp("Saved scene: " + sceneName);

%% ---------------------------------------------------------------------
function M = extractNuscMarks(jsonFile, cacheFile, origin, lim)
% Extract lane_divider / road_divider / stop_line / traffic_light layers
% from a nuScenes map expansion JSON, converted to patch-local metres and
% clipped to lim = [xmin xmax ymin ymax]. Cached to a .mat file.
if isfile(cacheFile)
    load(cacheFile, "M");
    return
end
disp("Parsing " + jsonFile + " (first run, cached afterwards) ...");
S = jsondecode(fileread(jsonFile));

% NOTE: some maps have null tokens and non-uniform records (jsondecode
% then yields [] tokens and cell arrays), so everything below is guarded.
nodeX = [S.node.x];
nodeY = [S.node.y];
dNode = tokenDict({S.node.token});
dLine = tokenDict({S.line.token});
dPoly = tokenDict({S.polygon.token});

    function e = elem(L, r)             % layer record: cell or struct array
        if iscell(L), e = L{r}; else, e = L(r); end
    end

    function t = tok(rec, fld)          % token field as string, "" if bad
        t = "";
        if isstruct(rec) && isfield(rec, fld)
            v = rec.(fld);
            if (ischar(v) || isstring(v)) && strlength(string(v)) > 0
                t = string(v);
            end
        end
    end

    function pts = linePts(t)
        pts = zeros(0, 2);
        if t == "" || ~isKey(dLine, t), return; end
        nt = elem(S.line, dLine(t)).node_tokens;
        if ~iscell(nt), nt = cellstr(nt); end
        nt = string(nt);
        nt = nt(isKey(dNode, nt));
        idx = dNode(nt);
        pts = [nodeX(idx)', nodeY(idx)'] - origin;
    end

    function pts = polyPts(t)
        pts = zeros(0, 2);
        if t == "" || ~isKey(dPoly, t), return; end
        nt = elem(S.polygon, dPoly(t)).exterior_node_tokens;
        if ~iscell(nt), nt = cellstr(nt); end
        nt = string(nt);
        nt = nt(isKey(dNode, nt));
        idx = dNode(nt);
        pts = [nodeX(idx)', nodeY(idx)'] - origin;
    end

    function tf = inLim(pts)
        tf = ~isempty(pts) && any( ...
            pts(:,1) > lim(1) & pts(:,1) < lim(2) & ...
            pts(:,2) > lim(3) & pts(:,2) < lim(4));
    end

M = struct("LD", {{}}, "RD", {{}}, "SL", {{}}, "TL", zeros(0, 3));
if isfield(S, "lane_divider")
    for r = 1:numel(S.lane_divider)
        p = linePts(tok(elem(S.lane_divider, r), "line_token"));
        if inLim(p), M.LD{end+1} = p; end
    end
end
if isfield(S, "road_divider")
    for r = 1:numel(S.road_divider)
        p = linePts(tok(elem(S.road_divider, r), "line_token"));
        if inLim(p), M.RD{end+1} = p; end
    end
end
if isfield(S, "stop_line")
    for r = 1:numel(S.stop_line)
        p = polyPts(tok(elem(S.stop_line, r), "polygon_token"));
        if inLim(p), M.SL{end+1} = p; end
    end
end
if isfield(S, "traffic_light")
    for r = 1:numel(S.traffic_light)
        p = linePts(tok(elem(S.traffic_light, r), "line_token"));
        if size(p, 1) >= 2 && inLim(p)
            yaw = atan2(p(end,2) - p(1,2), p(end,1) - p(1,1));
            M.TL(end+1, :) = [p(1,1), p(1,2), yaw];
        end
    end
end
save(cacheFile, "M");
end

function d = tokenDict(c)
% Dictionary token -> original index, skipping null/empty tokens (some
% map JSONs contain them and they would break the string conversion).
v = find(cellfun(@(t) (ischar(t) || isstring(t)) && strlength(string(t)) > 0, c));
d = dictionary(string(c(v)), v);
end

function sig = xodrSignalPositions(file)
% Fallback: [x y yaw] of every <signal> in an OpenDRIVE file whose
% planView is a dense polyline of <line/> geometries.
txt = fileread(file);
roads = regexp(txt, '<road\s.*?</road>', 'match');
sig = zeros(0, 3);
for r = 1:numel(roads)
    g = regexp(roads{r}, ...
        '<geometry s="([^"]+)" x="([^"]+)" y="([^"]+)" hdg="([^"]+)"', 'tokens');
    if isempty(g), continue; end
    G = cellfun(@str2double, vertcat(g{:}));            % [s x y hdg]
    sg = regexp(roads{r}, '<signal\s[^>]*\ss="([^"]+)"[^>]*\st="([^"]+)"', 'tokens');
    for k = 1:numel(sg)
        ss = str2double(sg{k}{1});
        tt = str2double(sg{k}{2});
        idx = find(G(:,1) <= ss, 1, "last");
        if isempty(idx), idx = 1; end
        ds = ss - G(idx,1);
        h  = G(idx,4);
        x = G(idx,2) + cos(h)*ds - sin(h)*tt;           % +t is left of road
        y = G(idx,3) + sin(h)*ds + cos(h)*tt;
        sig(end+1, :) = [x, y, h + pi]; %#ok<AGROW>     % face oncoming traffic
    end
end
end

function [half, adjusted] = shrinkBoxOffRoad(c, half, yaw, road, marg, minScale)
% Scale an oriented box down about its center until no road sample lies
% inside it (with marg metres of clearance off the lane centerlines).
% The scale never goes below minScale, so buildings shrink but survive.
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];    % columns: local x/y axes
q = abs((road(:,1:2) - c) * R);                 % box-local coordinates
in = q(:,1) < half(1) + marg & q(:,2) < half(2) + marg;
adjusted = false;
if ~any(in)
    return
end
qi = q(in, :);
si = max((qi(:,1) - marg) / half(1), (qi(:,2) - marg) / half(2));
s = max(min(min(si), 1), minScale);
if s < 1
    half = half * s;
    adjusted = true;
end
end

function boxes = footprintBoxes(pts, minFill, depth)
% Approximate a building footprint with one or more oriented boxes,
% rows [cx cy hx hy yaw]. If the min-area rect covers much more ground
% than the polygon (fill ratio < minFill), split the polygon in half
% across the rect's long axis and recurse - so L/U-shaped buildings
% become several tight boxes instead of one oversized one.
[c, half, yaw] = minAreaRect(pts);
boxes = [c, half, yaw];
if depth <= 0 || max(half) < 6
    return
end
try
    ps = polyshape(pts(:,1), pts(:,2));
catch
    return
end
rectA = 4 * half(1) * half(2);
if area(ps) <= 1 || rectA <= 0 || area(ps) / rectA >= minFill
    return
end
u = [cos(yaw) sin(yaw)];
v = [-sin(yaw) cos(yaw)];
if half(1) >= half(2)
    d = u; e = v; hl = half(1); he = half(2);
else
    d = v; e = u; hl = half(2); he = half(1);
end
boxes = zeros(0, 5);
for sgn = [-1 1]                        % clip polygon with each half-rect
    cc = c + sgn * d * hl/2;
    P = cc + [ d*hl/2 + e*he*2;  d*hl/2 - e*he*2; ...
              -d*hl/2 - e*he*2; -d*hl/2 + e*he*2];
    part = intersect(ps, polyshape(P(:,1), P(:,2)));
    rg = regions(part);
    for q = 1:numel(rg)
        vv = rg(q).Vertices;
        vv = vv(~any(isnan(vv), 2), :);
        if size(vv, 1) >= 3 && area(rg(q)) > 4
            boxes = [boxes; footprintBoxes(vv, minFill, depth - 1)]; %#ok<AGROW>
        end
    end
end
if isempty(boxes)                       % degenerate clip: keep the rect
    boxes = [c, half, yaw];
end
end

function [c, half, yaw] = minAreaRect(pts)
% Minimum-area oriented rectangle of 2-D points: center, half-extents, yaw.
pts = unique(pts, "rows", "stable");
try
    hullIdx = convhull(pts(:,1), pts(:,2));
    hull = pts(hullIdx, :);
catch
    hull = pts;
end
best = inf;
edges = diff(hull);
for e = 1:size(edges,1)
    a = atan2(edges(e,2), edges(e,1));
    Rm = [cos(-a) -sin(-a); sin(-a) cos(-a)];
    q = (Rm * pts')';
    lo = min(q); hi = max(q);
    area = prod(hi - lo);
    if area < best
        best = area;
        yaw = a;
        half = (hi - lo) / 2;
        cLocal = (hi + lo) / 2;
        Rb = [cos(a) -sin(a); sin(a) cos(a)];
        c = (Rb * cLocal')';
    end
end
if ~isfinite(best)   % degenerate footprint
    c = mean(pts, 1); half = [2 2]; yaw = 0;
end
half = max(half, 0.5);
end

function outFile = ensureGeoRef(inFile, lat, lon, outFolder)
% Write a copy of an OpenDRIVE file whose header carries a transverse-
% mercator geoReference centered at (lat, lon), into outFolder.
geo = sprintf(['<geoReference><![CDATA[+proj=tmerc +lat_0=%.12f ' ...
    '+lon_0=%.12f +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m ' ...
    '+no_defs]]></geoReference>'], lat, lon);
txt = fileread(inFile);
if contains(txt, "<geoReference>")
    txt = regexprep(txt, "<geoReference>.*?</geoReference>", geo, "once");
elseif ~isempty(regexp(txt, "<header[^>]*/>", "once"))
    txt = regexprep(txt, "(<header[^>]*)/>", "$1>" + geo + "</header>", "once");
else
    txt = regexprep(txt, "(<header[^>]*>)", "$1" + geo, "once");
end
[~, n, e] = fileparts(inFile);
outFile = fullfile(outFolder, n + "_geo" + e);
fid = fopen(outFile, "w");
fwrite(fid, txt);
fclose(fid);
end

function pts = xodrRoadSamples(file, mainOnly)
% [x y hdg] samples along every planView line geometry, interpolated to
% ~1 m spacing (the DP-simplified xodr has single long <line/> segments,
% so start points alone would be far too sparse for proximity queries).
% mainOnly=true keeps only roads outside junctions (junction="-1").
txt = fileread(file);
roads = regexp(txt, '<road\s.*?</road>', 'match');   % NOTE: \b is backspace
pts = zeros(0, 3);                                   % in MATLAB regexp!
for r = 1:numel(roads)
    hdr = regexp(roads{r}, '<road\s[^>]*>', 'match', 'once');
    if mainOnly && ~contains(hdr, 'junction="-1"'), continue; end
    g = regexp(roads{r}, ['<geometry s="[^"]+" x="([^"]+)" y="([^"]+)" ' ...
        'hdg="([^"]+)" length="([^"]+)"'], 'tokens');
    if isempty(g), continue; end
    G = cellfun(@str2double, vertcat(g{:}));         % [x y hdg len]
    for i = 1:size(G, 1)
        t = (0:1:max(G(i,4), 0))';
        pts = [pts; G(i,1) + t*cos(G(i,3)), ...
                    G(i,2) + t*sin(G(i,3)), ...
                    repmat(G(i,3), numel(t), 1)]; %#ok<AGROW>
    end
end
end
