param(
    [Parameter(Mandatory=$true)][string]$Manifest,
    [string]$SourceLevel,
    [string]$TargetLevel,
    [string]$CarlaRoot = "C:\carla",
    [string]$UnrealRoot = "C:\UnrealEngine",
    [string]$Python = "python",
    [ValidateSet("nuscenes", "opendrive")][string]$MarkingSource = "nuscenes",
    [string]$MarkingMapJson,
    [switch]$MarkingsOnly,
    [switch]$PlaceTrafficLights,
    [switch]$AllowCarlaWrite
)

$ErrorActionPreference = "Stop"

if (-not $AllowCarlaWrite) {
    throw "This command writes a new .umap under the CARLA source tree. Re-run with -AllowCarlaWrite after reviewing the manifest and target level."
}
if ($PlaceTrafficLights) {
    throw "-PlaceTrafficLights is disabled: ungrouped CARLA TrafficLight Blueprints crash WorldObserver at runtime. Keep OpenDRIVE signals authoritative until functional groups/controllers are generated."
}
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$manifestPath = (Resolve-Path -LiteralPath $Manifest).Path
$manifestData = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
if (-not $SourceLevel) { $SourceLevel = $manifestData.map.unreal_source_level }
if (-not $TargetLevel) { $TargetLevel = $manifestData.map.unreal_target_level }
if (-not $SourceLevel -or -not $TargetLevel) {
    throw "SourceLevel and TargetLevel were not supplied and are missing from the manifest."
}
if ($SourceLevel -eq $TargetLevel) {
    throw "SourceLevel and TargetLevel must differ; the pipeline does not overwrite the imported base map."
}
$catalogPath = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "asset_catalog.json")).Path
$pythonScript = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "unreal\build_environment.py")).Path
$markingGenerator = (Resolve-Path -LiteralPath (Join-Path $scriptRoot "tools\generate_lane_markings.py")).Path
$markingDirectory = Join-Path $scriptRoot "generated\markings"
$markingObj = Join-Path $markingDirectory ($manifestData.map.asset_name + "_LaneMarkings.obj")
$resultPath = Join-Path $markingDirectory ($manifestData.map.asset_name + "_UnrealBuildResult.json")
$uproject = Join-Path $CarlaRoot "Unreal\CarlaUE4\CarlaUE4.uproject"
$editor = Join-Path $UnrealRoot "Engine\Binaries\Win64\UE4Editor-Cmd.exe"

if (-not (Test-Path -LiteralPath $uproject)) { throw "Missing CARLA project: $uproject" }
if (-not (Test-Path -LiteralPath $editor)) { throw "Missing Unreal editor: $editor" }

$targetRelative = $TargetLevel -replace '^/Game/', ''
$targetFile = Join-Path (Join-Path $CarlaRoot "Unreal\CarlaUE4\Content") ($targetRelative.Replace('/', '\') + ".umap")
if ($MarkingsOnly -and -not (Test-Path -LiteralPath $targetFile)) {
    throw "-MarkingsOnly requires an existing decorated level. Run a full build first."
}
if (Test-Path -LiteralPath $targetFile) {
    try {
        $lockProbe = [System.IO.File]::Open(
            $targetFile,
            [System.IO.FileMode]::Open,
            [System.IO.FileAccess]::ReadWrite,
            [System.IO.FileShare]::None)
        $lockProbe.Close()
    }
    catch {
        throw "The target level is open in CARLA/Unreal and cannot be replaced: $targetFile. Close the running CARLA server/editor, then rerun this command."
    }
}

New-Item -ItemType Directory -Path $markingDirectory -Force | Out-Null
if ($MarkingsOnly) {
    $backup = Join-Path $scriptRoot ("generated\checkpoints\markings_" + $manifestData.map.asset_name + "_" + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
    New-Item -ItemType Directory -Path $backup | Out-Null
    Copy-Item -LiteralPath $targetFile -Destination $backup
    $assetDir = Join-Path (Join-Path $CarlaRoot "Unreal\CarlaUE4\Content") ($manifestData.map.package_name + '\Static\Marking\' + $manifestData.map.asset_name)
    if (Test-Path -LiteralPath $assetDir) {
        Copy-Item -LiteralPath $assetDir -Destination (Join-Path $backup 'marking_assets') -Recurse
    }
    foreach ($extension in @('.obj', '.mtl', '.json')) {
        $oldArtifact = [System.IO.Path]::ChangeExtension($markingObj, $extension)
        if (Test-Path -LiteralPath $oldArtifact) { Copy-Item -LiteralPath $oldArtifact -Destination $backup }
    }
    Write-Output "Markings-only rollback checkpoint: $backup"
}
$generatorArgs = @('--manifest', $manifestPath, '--output', $markingObj, '--source', $MarkingSource)
if ($MarkingMapJson) { $generatorArgs += @('--map-json', $MarkingMapJson) }
& $Python $markingGenerator @generatorArgs
if ($LASTEXITCODE -ne 0) { throw "Lane-marking mesh generation failed with exit code $LASTEXITCODE" }
$markingObj = (Resolve-Path -LiteralPath $markingObj).Path
if (Test-Path -LiteralPath $resultPath) { Remove-Item -LiteralPath $resultPath -Force }

$env:NUSC_CARLA_MANIFEST = $manifestPath
$env:NUSC_CARLA_ASSET_CATALOG = $catalogPath
$env:NUSC_CARLA_MARKINGS_OBJ = $markingObj
$env:NUSC_CARLA_RESULT = $resultPath
$env:NUSC_CARLA_SOURCE_LEVEL = $SourceLevel
$env:NUSC_CARLA_TARGET_LEVEL = $TargetLevel
$env:NUSC_CARLA_PLACE_TRAFFIC_LIGHTS = if ($PlaceTrafficLights) { "1" } else { "0" }
$env:NUSC_CARLA_MARKINGS_ONLY = if ($MarkingsOnly) { "1" } else { "0" }

& $editor $uproject "-ExecutePythonScript=$pythonScript" -unattended -nop4
$editorExitCode = $LASTEXITCODE
if ($editorExitCode -ne 0) { throw "Unreal editor exited with code $editorExitCode" }
if (-not (Test-Path -LiteralPath $resultPath)) {
    throw "Unreal did not report a successful level save. Inspect the latest CarlaUE4 log under C:\carla\Unreal\CarlaUE4\Saved\Logs."
}
Get-Content -LiteralPath $resultPath
