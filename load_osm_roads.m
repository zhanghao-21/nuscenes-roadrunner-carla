%% load_osm_roads.m
% Build a RoadRunner scene combining the nuScenes scene-0103 OpenDRIVE with
% the OpenStreetMap road network of the same Boston-Seaport area.
%
% Alignment strategy (deterministic):
%   - both xodr files get a REAL geoReference header (transverse mercator
%     at their respective local origins),
%   - the RoadRunner scene world origin is set to the nuScenes patch corner,
%   - both files are imported with ProjectionMode="FullProjection", so
%     RoadRunner places them by projection instead of by data center,
%   - manualShift (below) absorbs the residual OSM-vs-nuScenes registration
%     error (OSM absolute accuracy in cities is typically a few metres).
%
% Calibrating manualShift: run the script once and look at the
% "Scene-frame overlay" figure. Zoom to a feature visible in both layers
% (e.g. an intersection), data-cursor the same feature in both, then set
%   manualShift = [nusc_x - osm_x,  nusc_y - osm_y]
% and re-run. The figure and the RoadRunner import both honor it.

%% Config
base      = string(fileparts(mfilename("fullpath")));   % this script's folder
outDir    = base + "\nuscenes2xodr\output";
sceneXodr = outDir + "\boston-seaport_scene-0103.xodr";
osmFile   = outDir + "\boston-seaport_scene-0103.osm";
osmXodr   = outDir + "\boston-seaport_scene-0103_osm.xodr";

projectFolder = base + "\nuscenes_test1";
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";
sceneName     = "boston_seaport_scene_0103_osm";

manualShift = [0 0];   % [east north] metres applied to the OSM layer

assert(isfile(sceneXodr), "Missing: " + sceneXodr);
assert(isfile(osmFile),   "Missing: " + osmFile);

% nuScenes boston-seaport georeference (same constants as nuscenes2xodr)
EARTH_R = 6378137.0;
lat0 = 42.336849169438615;      % map origin (map coord 0,0)
lon0 = -71.05785369873047;
xmin = 550.1202137947669;       % patch corner = scene local (0,0), from meta json
ymin = 1523.2390413260864;
latO = lat0 + (ymin / EARTH_R) * (180/pi);                  % patch corner lat
lonO = lon0 + (xmin / (EARTH_R * cosd(lat0))) * (180/pi);   % patch corner lon
fprintf("Patch corner (scene origin): %.10f, %.10f\n", latO, lonO);

%% 1. OSM -> drivingScenario -> OpenDRIVE
ds = drivingScenario;
roadNetwork(ds, "OpenStreetMap", osmFile);
ref = ds.GeographicReference;   % [lat lon alt] of the OSM local origin
fprintf("OSM importer origin:         %.10f, %.10f\n", ref(1), ref(2));

export(ds, "OpenDRIVE", osmXodr);

%% 2. Diagnostic overlay in the scene frame (predicts RoadRunner placement)
% Offset of the OSM origin from the patch corner, as the projection sees it
tx = deg2rad(ref(2) - lonO) * EARTH_R * cosd((ref(1) + latO)/2);
ty = deg2rad(ref(1) - latO) * EARTH_R;

nusc = xodrPlanViewPoints(sceneXodr);
osm  = xodrPlanViewPoints(osmXodr);
figure; hold on; axis equal; grid on
plot(nusc(:,1), nusc(:,2), ".", MarkerSize=4, DisplayName="nuScenes lanes");
plot(osm(:,1) + tx + manualShift(1), osm(:,2) + ty + manualShift(2), ...
    ".", MarkerSize=4, DisplayName="OSM roads (shifted)");
legend; xlabel("x (m)"); ylabel("y (m)");
title(sprintf("Scene-frame overlay   (manualShift = [%.1f %.1f])", ...
    manualShift(1), manualShift(2)));

%% 3. Write georeferenced copies of both files (originals untouched)
sceneXodrGeo = ensureGeoRef(sceneXodr, latO, lonO);

% manualShift is baked into the declared OSM origin, so the projection
% itself carries the correction (no reliance on import Offset semantics)
latRefShifted = ref(1) + manualShift(2) / EARTH_R * (180/pi);
lonRefShifted = ref(2) + manualShift(1) / (EARTH_R * cosd(ref(1))) * (180/pi);
osmXodrGeo = ensureGeoRef(osmXodr, latRefShifted, lonRefShifted);

%% 4. Connect to RoadRunner (reuse a live session if we have one)
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

%% 5. Fresh scene: set world origin FIRST, then import both by projection
newScene(rrApp);
changeWorldSettings(rrApp, WorldOrigin=[latO lonO]);

nuscOpts = openDriveImportOptions( ...
    ImportSignals=true, ImportObjects=true, ...
    ProjectionMode="FullProjection");
importScene(rrApp, sceneXodrGeo, "OpenDRIVE", nuscOpts);

osmOpts = openDriveImportOptions( ...
    ImportSignals=false, ImportObjects=false, ...
    ProjectionMode="FullProjection");
importScene(rrApp, osmXodrGeo, "OpenDRIVE", osmOpts);

%% 6. Save into the project
saveScene(rrApp, sceneName);
disp("Saved scene: " + sceneName);

%% ---------------------------------------------------------------------
function outFile = ensureGeoRef(inFile, lat, lon)
% Write a copy of an OpenDRIVE file whose header carries a transverse-
% mercator geoReference centered at (lat, lon).
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
[p, n, e] = fileparts(inFile);
outFile = fullfile(p, n + "_geo" + e);
fid = fopen(outFile, "w");
fwrite(fid, txt);
fclose(fid);
end

function pts = xodrPlanViewPoints(file)
% Start points of every <geometry> element in an OpenDRIVE file.
txt = fileread(file);
els = regexp(txt, '<geometry\s[^>]*', 'match');   % \b is backspace in MATLAB
pts = nan(numel(els), 2);
for i = 1:numel(els)
    xt = regexp(els{i}, '\sx="([^"]+)"', 'tokens', 'once');
    yt = regexp(els{i}, '\sy="([^"]+)"', 'tokens', 'once');
    if ~isempty(xt) && ~isempty(yt)
        pts(i,:) = [str2double(xt{1}), str2double(yt{1})];
    end
end
pts = pts(~any(isnan(pts), 2), :);
end
