param(
    [Parameter(Mandatory=$true)][string]$Manifest,
    [string]$CarlaRoot = "C:\carla",
    [switch]$AllowCarlaWrite
)

$ErrorActionPreference = "Stop"

if (-not $AllowCarlaWrite) {
    throw "This command updates the imported CARLA package. Re-run with -AllowCarlaWrite after reviewing the decorated map."
}

$manifestPath = (Resolve-Path -LiteralPath $Manifest).Path
$manifestData = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
$packageName = $manifestData.map.package_name
$sourceName = $manifestData.map.asset_name
$targetName = $manifestData.map.target_asset_name
if (-not $packageName -or -not $sourceName -or -not $targetName) {
    throw "Manifest does not contain package_name, asset_name, and target_asset_name."
}

$contentRoot = Join-Path $CarlaRoot "Unreal\CarlaUE4\Content"
$mapFolder = Join-Path $contentRoot ("{0}\Maps\{1}" -f $packageName, $sourceName)
$targetLevel = Join-Path $mapFolder ($targetName + ".umap")
$openDriveFolder = Join-Path $mapFolder "OpenDrive"
$sourceXodr = Join-Path $openDriveFolder ($sourceName + ".xodr")
$targetXodr = Join-Path $openDriveFolder ($targetName + ".xodr")
$config = Join-Path $contentRoot ("{0}\Config\{0}.Package.json" -f $packageName)

foreach ($required in @($targetLevel, $sourceXodr, $config)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required imported/decorated asset is missing: $required"
    }
}

Copy-Item -LiteralPath $sourceXodr -Destination $targetXodr -Force
$packageData = Get-Content -LiteralPath $config -Raw | ConvertFrom-Json
$entry = [pscustomobject]@{
    name = $targetName
    path = $manifestData.map.package_map_path
    use_carla_materials = $true
}
$packageData.maps = @($packageData.maps | Where-Object { $_.name -ne $targetName }) + @($entry)
$backup = $config + ".nuscenes-reconstruction.bak"
if (-not (Test-Path -LiteralPath $backup)) {
    Copy-Item -LiteralPath $config -Destination $backup
}
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText(
    $config, ($packageData | ConvertTo-Json -Depth 20) + [Environment]::NewLine,
    $utf8NoBom)

Write-Output "Registered decorated map: $($manifestData.map.runtime_name)"
Write-Output "Copied OpenDRIVE: $targetXodr"
Write-Output "Updated package config: $config"
Write-Output "Original package config backup: $backup"
