[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$vehicleScript = Join-Path $PSScriptRoot 'Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_main.py'
$vehicles = @(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and
    $_.CommandLine -and $_.CommandLine.Contains($vehicleScript)
})
foreach ($vehicleProcess in $vehicles) {
    Stop-Process -Id $vehicleProcess.ProcessId -ErrorAction SilentlyContinue
    Write-Host "Stopped vehicle PID $($vehicleProcess.ProcessId)."
}
Write-Host "Stopped $($vehicles.Count) vehicle process(es) from this checkout."
