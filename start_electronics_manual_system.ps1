param(
    [ValidateRange(1, 4)]
    [int]$VehicleCount = 3,
    [switch]$NoFrontend,
    [switch]$NoBridge
)

$ErrorActionPreference = "Stop"
$QcarPath = Join-Path $PSScriptRoot "Development/multi_vehicle_self_driving_RealQcar/qcar"
$SimulationPath = Join-Path $QcarPath "simulation"
$GuiPath = Join-Path $QcarPath "GUI"
$FrontendPath = Join-Path $PSScriptRoot "GroundStation-Qcar-App"
$NativeLibrary = Join-Path $QcarPath "electronics/native/build/cran_electronics_core.dll"

if (-not (Test-Path -LiteralPath $NativeLibrary -PathType Leaf)) {
    throw "Native core not found. Run .\run_electronics_manual_test.ps1 first."
}

for ($CarId = 0; $CarId -lt $VehicleCount; $CarId++) {
    $VehicleCommand = "conda run -n Qcar python fake_vehicle_real_logic.py $CarId"
    Start-Process powershell `
        -ArgumentList @("-NoExit", "-Command", $VehicleCommand) `
        -WorkingDirectory $SimulationPath `
        -WindowStyle Normal
}

if (-not $NoBridge) {
    Start-Process powershell `
        -ArgumentList @("-NoExit", "-Command", "conda run -n Qcar python app_main.py") `
        -WorkingDirectory $GuiPath `
        -WindowStyle Normal
}

if (-not $NoFrontend) {
    Start-Process powershell `
        -ArgumentList @("-NoExit", "-Command", "npm run dev") `
        -WorkingDirectory $FrontendPath `
        -WindowStyle Normal
}

Write-Host "Started $VehicleCount fake vehicles with the native electronics core."
if (-not $NoBridge) { Write-Host "Ground Station bridge was started." }
if (-not $NoFrontend) { Write-Host "Web UI was started; open the Vite URL shown in its terminal." }
