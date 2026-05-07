# Deploy script for FusionRobotExporter
# Installs to Fusion 360 Scripts folder

$SourcePath = $PSScriptRoot
$ProjectName = "FusionRobotExporter"
$FusionAPIPath = "$env:APPDATA\Autodesk\Autodesk Fusion 360\API\Scripts"
$DestinationPath = Join-Path $FusionAPIPath $ProjectName

Write-Host "=== Fusion 360 Script Deploy ===" -ForegroundColor Cyan
Write-Host "Project: $ProjectName" -ForegroundColor White
Write-Host "Source:  $SourcePath" -ForegroundColor White
Write-Host "Target:  $DestinationPath" -ForegroundColor White
Write-Host ""

# Check if Fusion API folder exists
if (-not (Test-Path $FusionAPIPath)) {
    Write-Host "Error: Fusion 360 API folder not found at $FusionAPIPath" -ForegroundColor Red
    Write-Host "Is Fusion 360 installed?" -ForegroundColor Red
    exit 1
}

# Remove existing installation if present
if (Test-Path $DestinationPath) {
    Write-Host "Removing existing installation..." -ForegroundColor Yellow
    Remove-Item -Path $DestinationPath -Recurse -Force
}

# Copy project files
Write-Host "Copying files..." -ForegroundColor Cyan
Copy-Item -Path $SourcePath -Destination $DestinationPath -Recurse -Force

# Remove excluded items from destination
$excludeItems = @(".git", "deploy.ps1", "deploy.sh", ".gitignore", "__pycache__")
foreach ($item in $excludeItems) {
    $itemPath = Join-Path $DestinationPath $item
    if (Test-Path $itemPath) {
        Remove-Item -Path $itemPath -Recurse -Force
    }
}

Write-Host ""
Write-Host "Deployed successfully!" -ForegroundColor Green
Write-Host "Open Fusion 360 -> Shift+S -> $ProjectName -> Run" -ForegroundColor Green
