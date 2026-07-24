%% insert_intersection.m
% Launch RoadRunner from MATLAB, programmatically build a 4-way
% intersection as a RoadRunner HD Map, and import it into a new scene.
%
% Requirements:
%   - MATLAB + Automated Driving Toolbox (for roadrunner / roadrunnerHDMap)
%   - RoadRunner and RoadRunner Scene Builder licenses
%     (Scene Builder is what auto-builds the junction on HD Map import)
%
% How it works: the MATLAB API cannot edit scene geometry directly, so we
% describe two crossing 2-lane roads as an HD map. Where the lanes overlap,
% Scene Builder creates an "overlap group" and builds the intersection
% (junction surface, corner radii, markings) automatically.

%% 1. Launch RoadRunner with the project
projectFolder = string(fileparts(mfilename("fullpath"))) + "\nuscenes_test1";
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";  % <-- adjust to your version

rrApp = roadrunner(projectFolder, InstallationFolder=installFolder);
newScene(rrApp);

%% 2. Define the intersection geometry
laneWidth = 3.5;   % m
halfLen   = 60;    % half-length of each road arm (m)

map = roadrunnerHDMap;

% Geometry helpers: EW road drawn west->east, NS road drawn south->north
ew = @(y) [-halfLen y; halfLen y];
ns = @(x) [x -halfLen; x halfLen];

% Lane boundaries: 3 per road (right edge, centerline, left edge)
bGeom = {ew(-laneWidth); ew(0); ew(laneWidth); ...   % B1..B3 east-west road
         ns(-laneWidth); ns(0); ns(laneWidth)};      % B4..B6 north-south road
for i = 1:numel(bGeom)
    map.LaneBoundaries(i) = roadrunner.hdmap.LaneBoundary( ...
        ID="B" + i, Geometry=bGeom{i});
end

% One travel lane on each side of each centerline (right-hand traffic)
laneGeom  = {ew(-laneWidth/2); ew(laneWidth/2); ...
             ns(-laneWidth/2); ns(laneWidth/2)};
travelDir = ["Forward" "Backward" "Forward" "Backward"];
leftB     = ["B2" "B3" "B5" "B6"];
rightB    = ["B1" "B2" "B4" "B5"];
for i = 1:4
    map.Lanes(i) = roadrunner.hdmap.Lane(ID="L" + i, ...
        Geometry=laneGeom{i}, TravelDirection=travelDir(i), ...
        LaneType="Driving");
    leftBoundary (map.Lanes(i), leftB(i),  Alignment="Forward");
    rightBoundary(map.Lanes(i), rightB(i), Alignment="Forward");
end

% Optional sanity check before import
plot(map);
axis equal

%% 3. Write the HD map and import it (junction gets built here)
hdFile = fullfile(tempdir, "fourWayIntersection.rrhd");
write(map, hdFile);

overlapOpts = enableOverlapGroupsOptions(IsEnabled=true);
buildOpts   = roadrunnerHDMapBuildOptions(EnableOverlapGroupsOptions=overlapOpts);
importOpts  = roadrunnerHDMapImportOptions(BuildOptions=buildOpts);

importScene(rrApp, hdFile, "RoadRunner HD Map", importOpts);

%% 4. Save the scene into the project
saveScene(rrApp, "ProgrammaticIntersection");
% Scene lands in nuscenes_test1\Scenes\ProgrammaticIntersection.rrscene

% When done:
% close(rrApp);
