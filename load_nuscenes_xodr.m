%% load_nuscenes_xodr.m
% Load a nuScenes-derived OpenDRIVE map into RoadRunner and save it
% as a scene in the nuscenes_test1 project.

base          = string(fileparts(mfilename("fullpath")));   % this script's folder
xodrFile      = base + "\nuscenes2xodr\output\boston-seaport_scene-0103.xodr";
projectFolder = base + "\nuscenes_test1";
installFolder = "C:\Program Files\RoadRunner R2026a\bin\win64";  % <-- adjust to your version
sceneName     = "boston_seaport_scene_0103";

assert(isfile(xodrFile), "OpenDRIVE file not found: " + xodrFile);

%% Connect to RoadRunner (reuse the running instance if we have one)
needLaunch = true;
if exist("rrApp", "var")
    try
        status(rrApp);          % errors if the app was closed
        needLaunch = false;
    catch
    end
end
if needLaunch
    rrApp = roadrunner(projectFolder, InstallationFolder=installFolder);
end

%% Import the OpenDRIVE file into a fresh scene
newScene(rrApp);

opts = openDriveImportOptions( ...
    ImportSignals=true, ...
    ImportObjects=true);

importScene(rrApp, xodrFile, "OpenDRIVE", opts);

%% Save into the project (lands in nuscenes_test1\Scenes\)
saveScene(rrApp, sceneName);
disp("Saved scene: " + sceneName);
