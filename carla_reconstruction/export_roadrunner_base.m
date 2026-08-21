function fbxPath = export_roadrunner_base(manifestPath, projectFolder, outputFolder, installFolder)
%EXPORT_ROADRUNNER_BASE Programmatically export CARLA road geometry.
%   The manifest OpenDRIVE is imported in its existing patch-local frame, so
%   the FBX, Unreal decorations, and recorded trajectories share one origin.
%   CARLA Filmbox is intentional: unlike generic Filmbox, it exports meshes
%   split by semantic class and .rrdata.xml material metadata for CARLA's
%   RoadRunner importer.  If a project has no renderable marking asset, the
%   Unreal decoration stage generates the visual markings from OpenDRIVE.

arguments
    manifestPath (1,1) string
    projectFolder (1,1) string
    outputFolder (1,1) string
    installFolder (1,1) string = "C:\Traffic software\RoadRunner R2024a\bin\win64"
end

assert(isfile(manifestPath), "Manifest not found: " + manifestPath);
assert(isfolder(projectFolder), "RoadRunner project not found: " + projectFolder);
projectFile = fullfile(projectFolder, "Project", "Project.rrproj");
assert(isfile(projectFile), "RoadRunner project metadata not found: " + projectFile);
% Git does not retain the ignored empty working folders in a RoadRunner
% project. Recreate them locally without touching the read-only OneDrive copy.
workingFolders = ["Assets", "Exports", "Scenes", "Scenarios"];
for folder = workingFolders
    path = fullfile(projectFolder, folder);
    if ~isfolder(path)
        mkdir(path);
    end
end
manifest = jsondecode(fileread(manifestPath));
xodrPath = string(manifest.source.xodr);
assetName = string(manifest.map.asset_name);
assert(isfile(xodrPath), "OpenDRIVE not found: " + xodrPath);
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end

rrApp = roadrunner(ProjectFolder=projectFolder, InstallationFolder=installFolder);
try
    rrSim = createSimulation(rrApp);
    set(rrSim, "SimulationCommand", "Stop");
catch
end
newScene(rrApp);
opts = openDriveImportOptions( ...
    ImportSignals=false, ...
    ImportObjects=true);
importScene(rrApp, xodrPath, "OpenDRIVE", opts);
saveScene(rrApp, assetName + "_CARLA_Base");

fbxPath = fullfile(outputFolder, assetName + ".fbx");
fbxOptions = filmboxExportOptions( ...
    SplitMeshes=true, ...
    ResizeTextureDimensions=true, ...
    EmbedTextures=true);
xodrOptions = openDriveExportOptions( ...
    OpenDriveVersion=1.5, ...
    ExportMarkingsAsLine=true, ...
    ExportSignals=false, ...
    ExportObjects=true);
carlaOptions = carlaFilmboxExportOptions( ...
    FilmboxOptions=fbxOptions, ...
    OpenDriveOptions=xodrOptions);
exportScene(rrApp, fbxPath, "CARLA Filmbox", carlaOptions);

rrdataPath = fullfile(outputFolder, assetName + ".rrdata.xml");
assert(isfile(fbxPath), "RoadRunner did not create FBX: " + fbxPath);
assert(isfile(rrdataPath), ...
    "CARLA Filmbox did not create material metadata: " + rrdataPath);
fprintf("Exported CARLA road geometry: %s\n", fbxPath);
fprintf("Exported CARLA material metadata: %s\n", rrdataPath);
end
