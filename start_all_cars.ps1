[CmdletBinding()]
param(
    [string]$Python = 'python',
    [ValidateRange(1, 32)][int]$VehicleCount = 4
)
$ErrorActionPreference = 'Stop'
$vehicleScript = Join-Path $PSScriptRoot 'Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_main.py'
$pythonCommand = (Get-Command $Python -ErrorAction Stop).Source
$launchLogs = Join-Path $PSScriptRoot 'logs/launch'
New-Item -ItemType Directory -Path $launchLogs -Force | Out-Null
for ($carId = 0; $carId -lt $VehicleCount; $carId++) {
    $vehicleProcess = Start-Process -FilePath $pythonCommand -WindowStyle Hidden -PassThru `
        -WorkingDirectory $PSScriptRoot `
        -ArgumentList @('-u', ('"{0}"' -f $vehicleScript), '--car-id', $carId) `
        -RedirectStandardOutput (Join-Path $launchLogs "car_$carId.out.log") `
        -RedirectStandardError (Join-Path $launchLogs "car_$carId.err.log")
    Write-Host "Car $carId started with PID $($vehicleProcess.Id). Logs: $launchLogs"
}
