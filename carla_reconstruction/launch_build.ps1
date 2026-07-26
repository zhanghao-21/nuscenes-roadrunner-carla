param(
    [Parameter(Mandatory=$true)][string]$Manifest,
    [string]$SourceLevel,
    [string]$TargetLevel,
    [string]$CarlaRoot = "C:\carla",
    [string]$UnrealRoot = "C:\UnrealEngine",
    [switch]$PlaceTrafficLights,
    [switch]$AllowCarlaWrite
)

$ErrorActionPreference = "Stop"

if (-not $AllowCarlaWrite) {
    throw "This command writes a new .umap under the CARLA source tree. Re-run with -AllowCarlaWrite after reviewing the manifest and target level."
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
$uproject = Join-Path $CarlaRoot "Unreal\CarlaUE4\CarlaUE4.uproject"
$editor = Join-Path $UnrealRoot "Engine\Binaries\Win64\UE4Editor-Cmd.exe"

if (-not (Test-Path -LiteralPath $uproject)) { throw "Missing CARLA project: $uproject" }
if (-not (Test-Path -LiteralPath $editor)) { throw "Missing Unreal editor: $editor" }

$env:NUSC_CARLA_MANIFEST = $manifestPath
$env:NUSC_CARLA_ASSET_CATALOG = $catalogPath
$env:NUSC_CARLA_SOURCE_LEVEL = $SourceLevel
$env:NUSC_CARLA_TARGET_LEVEL = $TargetLevel
$env:NUSC_CARLA_PLACE_TRAFFIC_LIGHTS = if ($PlaceTrafficLights) { "1" } else { "0" }

& $editor $uproject "-ExecutePythonScript=$pythonScript" -unattended -nop4
exit $LASTEXITCODE
