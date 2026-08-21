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
$trafficManagerFolder = Join-Path $mapFolder "TM"
$sourceTrafficManager = Join-Path $trafficManagerFolder ($sourceName + ".bin")
$targetTrafficManager = Join-Path $trafficManagerFolder ($targetName + ".bin")
$repairScript = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "tools\repair_carla_xodr.py"
$config = Join-Path $contentRoot ("{0}\Config\{0}.Package.json" -f $packageName)

foreach ($required in @($targetLevel, $sourceXodr, $config, $repairScript)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required imported/decorated asset is missing: $required"
    }
}

$xodrBackup = $sourceXodr + ".pre-junction-id-fix.bak"
if (-not (Test-Path -LiteralPath $xodrBackup)) {
    Copy-Item -LiteralPath $sourceXodr -Destination $xodrBackup
}
$repairJson = & python $repairScript --input $sourceXodr --in-place
if ($LASTEXITCODE -ne 0) {
    throw "OpenDRIVE topology repair failed with exit code $LASTEXITCODE"
}
$repair = $repairJson | ConvertFrom-Json
if ($repair.remapped_count -gt 0) {
    foreach ($cache in @($sourceTrafficManager, $targetTrafficManager)) {
        if (Test-Path -LiteralPath $cache -PathType Leaf) {
            $cacheBackup = $cache + ".pre-junction-id-fix.bak"
            if (Test-Path -LiteralPath $cacheBackup) {
                throw "Refusing to replace existing Traffic Manager backup: $cacheBackup"
            }
            Move-Item -LiteralPath $cache -Destination $cacheBackup
            Write-Output "Backed up stale Traffic Manager cache: $cacheBackup"
        }
    }
}

Copy-Item -LiteralPath $sourceXodr -Destination $targetXodr -Force
if (Test-Path -LiteralPath $sourceTrafficManager -PathType Leaf) {
    Copy-Item -LiteralPath $sourceTrafficManager -Destination $targetTrafficManager -Force
} else {
    Write-Warning "Traffic Manager cache was not found. TM will build its local graph from the corrected OpenDRIVE map on first use."
}
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
Write-Output "Junction IDs remapped for CARLA: $($repair.remapped_count)"
if (Test-Path -LiteralPath $targetTrafficManager -PathType Leaf) {
    Write-Output "Copied Traffic Manager data: $targetTrafficManager"
}
Write-Output "Updated package config: $config"
Write-Output "Original package config backup: $backup"
