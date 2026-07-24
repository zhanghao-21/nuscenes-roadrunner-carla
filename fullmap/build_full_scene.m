%% build_full_scene.m
% Build ONE large RoadRunner scene covering the boston-seaport region that
% contains all four nuScenes-mini Boston scenarios (0103, 0553, 0655,
% 0757): region x [159,2085], y [509,1797] in the nuScenes map frame
% (~1.9 x 1.3 km), generated with nuscenes2xodr --region.
%
% Same pipeline as nuscenes_ground_buildings.m, full-region inputs:
%   roads    <- fullmap\boston-seaport_full.xodr  (2086 roads, 436 junctions)
%   markings <- nuScenes map layers clipped to the region
%   signals  <- 131 xodr <signal> entries (posts + mast arms + heads)
%   ground   <- flat slab over the region
%   buildings<- fullmap\boston-seaport_full.osm (Overpass extract)
%
% Saves scene "boston_seaport_full_city". Needs Scene Builder license.
% Replay any Boston scenario on it with replay_on_full.m.

%% Config
% base / nuscMapsDir / installFolder can be pre-set by the caller
% (e.g. main.py via matlab -batch "base='...'; run('build_full_scene.m')")
if ~exist("base", "var")
base    = string(fileparts(fileparts(mfilename("fullpath"))));   % repo root
end
fullDir = base + "\fullmap";
mapName = "boston-seaport";
if ~exist("nuscMapsDir", "var")
nuscMapsDir = base + "\v1.0-mini\maps\expansion";
end

sceneXodr = fullDir + "\boston-seaport_full.xodr";
osmFile   = fullDir + "\boston-seaport_full.osm";
metaFile  = fullDir + "\boston-seaport_full_meta.json";
mapJson   = nuscMapsDir + "\" + mapName + ".json";
rrhdFile  = fullDir + "\boston-seaport_full_city.rrhd";
markCache = fullDir + "\boston-seaport_full_marks.mat";

projectFolder = base + "\nuscenes_test1";
if ~exist("installFolder", "var")
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";
end
sceneName     = "boston_seaport_full_city";

manualShift = [0 0];             % OSM->nuScenes calibration for buildings

groundMargin = 60;               % slab margin beyond the region (m)
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
minBldgScale       = 0.8;    % floor for shrinking road-overlapping buildings
warning("off", "MATLAB:polyshape:repairedBySimplify");

laneDividerAsset = "Assets/Markings/DashedSingleWhite.rrlms";
roadDividerAsset = "Assets/Markings/SolidDoubleYellow.rrlms";

% Traffic-light assembly copied from FourWaySignal.rrscene: a vertical
% post + a mast arm over the road + 3-light heads hanging from the arm.
signalPostAsset = "Assets/Props/Signals/Signal_Post_30ft.fbx";
signalPostHalf  = [0.28 0.28 4.6];  % 9.2 m pole
signalArmAsset  = "Assets/Props/Signals/Signal_MastArm_25ft.fbx";
signalArmHalf   = [3.88 0.15 0.65]; % 7.8 m arm; spans local +X from pole
signalArmZ      = 5.4;              % arm mounting height (center, m)
signalHeadAsset = "Assets/Props/Signals/Signal_3Light_Post01.fbx";
signalHeadHalf  = [0.26 0.32 0.58];
signalHeadZ     = 4.9;              % head center height (below the arm)
signalHeadFrac  = [0.5 0.85];       % head positions as fraction of arm span
signalYawOffset = 0;                % rad: global facing tweak

assert(isfile(sceneXodr), "Missing: " + sceneXodr);
assert(isfile(metaFile),  "Missing: " + metaFile);
assert(isfile(mapJson),   "Missing: " + mapJson);

% published nuScenes map origin (lat/lon of map coordinate (0,0))
lat0 = 42.336849169438615;
lon0 = -71.05785369873047;
EARTH_R = 6378137.0;

% region corner / extent from the sidecar
meta = jsondecode(fileread(metaFile));
xmin = meta.patch.x_min;  ymin = meta.patch.y_min;
patchExtent = [meta.patch.x_max - xmin, meta.patch.y_max - ymin];
fprintf("Region: %.0f x %.0f m at map (%.0f, %.0f)\n", ...
    patchExtent(1), patchExtent(2), xmin, ymin);

latO = lat0 + (ymin / EARTH_R) * (180/pi);
lonO = lon0 + (xmin / (EARTH_R * cosd(lat0))) * (180/pi);

% region is too large for the plain OSM API (node cap) - Overpass fallback
if ~isfile(osmFile)
    pad  = 0.0018;
    latS = lat0 + (meta.patch.y_min / EARTH_R) * (180/pi) - pad;
    latN = lat0 + (meta.patch.y_max / EARTH_R) * (180/pi) + pad;
    lonW = lon0 + (meta.patch.x_min / (EARTH_R*cosd(lat0))) * (180/pi) - pad;
    lonE = lon0 + (meta.patch.x_max / (EARTH_R*cosd(lat0))) * (180/pi) + pad;
    bb = sprintf("%.7f,%.7f,%.7f,%.7f", latS, lonW, latN, lonE);
    q = "[out:xml][timeout:180];(way[""building""](" + bb + ");" + ...
        "way[""highway""](" + bb + ");" + ...
        "node[""highway""=""traffic_signals""](" + bb + "););(._;>;);out;";
    fprintf("Downloading region OSM via Overpass ...\n");
    ok = false;
    for url = ["https://overpass-api.de/api/interpreter", ...
               "https://overpass.kumi.systems/api/interpreter"]
        try
            txt = webwrite(url, "data", q, weboptions(Timeout=240));
            fid = fopen(osmFile, "w"); fwrite(fid, txt); fclose(fid);
            ok = true;
            break
        catch
        end
    end
    if ~ok
        warning("Overpass download failed - continuing without buildings.");
    end
end

%% 1. nuScenes map layers: dividers, traffic lights (clipped to region)
M = extractNuscMarks(mapJson, markCache, [xmin ymin], ...
    [-10, patchExtent(1)+10, -10, patchExtent(2)+10]);
fprintf("nuScenes layers in region: %d lane dividers, %d road dividers, " + ...
    "%d stop lines, %d traffic lights\n", ...
    numel(M.LD), numel(M.RD), numel(M.SL), size(M.TL, 1));

%% 2. OSM building footprints (region frame)
roadPts = xodrRoadSamples(sceneXodr, false);    % ~1 m road samples [x y hdg]
mainPts = xodrRoadSamples(sceneXodr, true);     % non-junction roads only
assert(~isempty(mainPts), "No road samples parsed from " + sceneXodr);
fprintf("Road samples: %d (%d on main roads)\n", ...
    size(roadPts,1), size(mainPts,1));

lim = [-groundMargin, patchExtent(1)+groundMargin, ...
       -groundMargin, patchExtent(2)+groundMargin];
fp = {};
fh = [];
if ~isfile(osmFile)
    disp("No OSM data - scene will have no buildings.");
else
S = readstruct(osmFile, FileType="xml");
nodeId  = [S.node.idAttribute];
nodeLat = [S.node.latAttribute];
nodeLon = [S.node.lonAttribute];
id2idx  = containers.Map(nodeId, 1:numel(nodeId));

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
fprintf("OSM buildings in region: %d\n", numel(fp));

% signals from the xodr (converter placed 131 across the region)
signals = xodrSignalPositions(sceneXodr);
fprintf("Signals: %d from xodr <signal> entries\n", size(signals, 1));

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
% CurveMarking/CurveMarkingType were added in R2024b.  Keep the full-map
% build usable with older managed installations by omitting only these
% custom divider curves when that API is unavailable.
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

% --- buildings: concave footprints decomposed, road overlaps shrunk
nObj = 0;
nShrunk = 0;
allBoxes = zeros(0, 5);
for k = 1:numel(fp)
    boxes = footprintBoxes(fp{k}, buildingMinFill, buildingSplitDepth);
    for j = 1:size(boxes, 1)
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

% --- traffic signals (post + mast arm + heads, facing snapped to lanes)
sigPole = zeros(size(signals, 1), 2);
sigU    = zeros(size(signals, 1), 2);
sigF    = zeros(size(signals, 1), 1);
wrapAng = @(x) mod(x + pi, 2*pi) - pi;
for k = 1:size(signals, 1)
    pos = signals(k, 1:2);
    f   = signals(k, 3);                            % recorded facing
    d2h  = hypot(mainPts(:,1) - pos(1), mainPts(:,2) - pos(2));
    cand = find(d2h < 8);
    if isnan(f)
        if ~isempty(cand)
            [~, ci] = min(d2h(cand));
            f = mainPts(cand(ci), 3) + pi;
        else
            [~, ni] = min(d2h);
            f = mainPts(ni, 3) + pi;
        end
    elseif ~isempty(cand)
        a = abs(wrapAng(mainPts(cand,3) - (f + pi)));
        same = cand(a < pi/3);
        opp  = cand(a > 2*pi/3);
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
    dPlus  = min(hypot(roadPts(:,1) - (pos(1) + 6*u(1)), ...
                       roadPts(:,2) - (pos(2) + 6*u(2))));
    dMinus = min(hypot(roadPts(:,1) - (pos(1) - 6*u(1)), ...
                       roadPts(:,2) - (pos(2) - 6*u(2))));
    if dPlus > dMinus, u = -u; end
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
                GeoOrientation=[0 0 f - pi/2 + signalYawOffset]), ...
            ObjectTypeReference=roadrunner.hdmap.Reference(ID="SigHeadType"));
    end
end

write(map, rrhdFile);
disp("Wrote " + rrhdFile);

% quick preview
figure; hold on; axis equal; grid on
rectangle(Position=[xs y0 xe-xs ye-y0], EdgeColor=[0.5 0.5 0.5]);
plot(roadPts(1:5:end,1), roadPts(1:5:end,2), ".", MarkerSize=2, ...
    Color=[0.7 0.7 0.9]);
for k = 1:size(allBoxes, 1)
    c = allBoxes(k,1:2); hf = allBoxes(k,3:4); yw = allBoxes(k,5);
    R = [cos(yw) -sin(yw); sin(yw) cos(yw)];
    cn = (R * ([1 1; 1 -1; -1 -1; -1 1; 1 1] .* hf)')' + c;
    plot(cn(:,1), cn(:,2), "-", Color=[0.3 0.8 0.9]);
end
for k = 1:numel(M.LD), plot(M.LD{k}(:,1), M.LD{k}(:,2), "w--"); end
for k = 1:numel(M.RD), plot(M.RD{k}(:,1), M.RD{k}(:,2), "y-"); end
plot(signals(:,1), signals(:,2), "^", MarkerSize=6, ...
    MarkerFaceColor="y", MarkerEdgeColor="k");
set(gca, Color=[0.2 0.2 0.2]);
title(sprintf("Full region: %d buildings, %d markings, %d signals", ...
    numel(fp), nMark, size(signals, 1)));
drawnow

%% 4. RoadRunner: roads first, then the HD map content
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
changeWorldSettings(rrApp, WorldOrigin=[latO lonO], ...
    SceneExtents=[patchExtent(1) + 2*groundMargin, ...
                  patchExtent(2) + 2*groundMargin], ...
    SceneCenter=[patchExtent(1)/2, patchExtent(2)/2]);

% 4a. full-region OpenDRIVE (this import takes a while: 2086 roads)
sceneXodrGeo = ensureGeoRef(sceneXodr, latO, lonO, fullDir);
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
% from a nuScenes map expansion JSON, converted to region-local metres and
% clipped to lim = [xmin xmax ymin ymax]. Cached to a .mat file.
if isfile(cacheFile)
    load(cacheFile, "M");
    return
end
disp("Parsing " + jsonFile + " (first run, cached afterwards) ...");
S = jsondecode(fileread(jsonFile));

nodeX = [S.node.x];
nodeY = [S.node.y];
dNode = tokenDict({S.node.token});
dLine = tokenDict({S.line.token});
dPoly = tokenDict({S.polygon.token});

    function e = elem(L, r)
        if iscell(L), e = L{r}; else, e = L(r); end
    end

    function t = tok(rec, fld)
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
v = find(cellfun(@(t) (ischar(t) || isstring(t)) && strlength(string(t)) > 0, c));
d = dictionary(string(c(v)), v);
end

function sig = xodrSignalPositions(file)
% [x y yaw] of every <signal> in an OpenDRIVE file.
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
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
q = abs((road(:,1:2) - c) * R);
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
for sgn = [-1 1]
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
if isempty(boxes)
    boxes = [c, half, yaw];
end
end

function [c, half, yaw] = minAreaRect(pts)
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
if ~isfinite(best)
    c = mean(pts, 1); half = [2 2]; yaw = 0;
end
half = max(half, 0.5);
end

function outFile = ensureGeoRef(inFile, lat, lon, outFolder)
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
% [x y hdg] ~1 m samples along every planView line geometry.
txt = fileread(file);
roads = regexp(txt, '<road\s.*?</road>', 'match');   % NOTE: \b is backspace
parts = cell(numel(roads), 1);                       % in MATLAB regexp!
for r = 1:numel(roads)
    hdr = regexp(roads{r}, '<road\s[^>]*>', 'match', 'once');
    if mainOnly && ~contains(hdr, 'junction="-1"'), continue; end
    g = regexp(roads{r}, ['<geometry s="[^"]+" x="([^"]+)" y="([^"]+)" ' ...
        'hdg="([^"]+)" length="([^"]+)"'], 'tokens');
    if isempty(g), continue; end
    G = cellfun(@str2double, vertcat(g{:}));         % [x y hdg len]
    seg = cell(size(G, 1), 1);
    for i = 1:size(G, 1)
        t = (0:1:max(G(i,4), 0))';
        seg{i} = [G(i,1) + t*cos(G(i,3)), ...
                  G(i,2) + t*sin(G(i,3)), ...
                  repmat(G(i,3), numel(t), 1)];
    end
    parts{r} = vertcat(seg{:});
end
pts = vertcat(parts{:});
if isempty(pts), pts = zeros(0, 3); end
end
