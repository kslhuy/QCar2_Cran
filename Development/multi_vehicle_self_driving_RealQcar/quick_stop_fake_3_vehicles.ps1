[CmdletBinding()]
param(
    [string]$GroundStationHost = "127.0.0.1",
    [int]$Port = 5000,
    [string]$CarIds = "0,1,2",
    [switch]$KeepTerminals,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

$resolvedCarIds = foreach ($carIdValue in ($CarIds -split ",")) {
    $trimmedCarId = $carIdValue.Trim()
    if (-not [string]::IsNullOrWhiteSpace($trimmedCarId)) {
        [int]$trimmedCarId
    }
}

function Test-QuickFakeVehicleProcess {
    param(
        [Microsoft.Management.Infrastructure.CimInstance]$Process,
        [int]$CarId
    )

    if (-not $Process.CommandLine) {
        return $false
    }

    $commandLine = $Process.CommandLine
    $carIdArgumentMatches = (
        $commandLine -like "* '$CarId' *" -or
        $commandLine -like "* $CarId *" -or
        $commandLine -like "*`"$CarId`" *"
    )

    return (
        $commandLine -like "*fake_vehicle_real_logic.py*" -and
        $commandLine -like "*$GroundStationHost*" -and
        $commandLine -like "*$Port*" -and
        $carIdArgumentMatches
    )
}

$allProcesses = Get-CimInstance Win32_Process
$targetProcesses = foreach ($carId in $resolvedCarIds) {
    $allProcesses | Where-Object { Test-QuickFakeVehicleProcess -Process $_ -CarId $carId }
}

$targetProcesses = $targetProcesses |
    Where-Object { $_ } |
    Sort-Object -Property ProcessId -Unique

if (-not $targetProcesses) {
    Write-Host "No matching fake_vehicle_real_logic.py quick-test processes found."
} else {
    foreach ($process in $targetProcesses) {
        $description = "$($process.Name) PID $($process.ProcessId)"

        if ($DryRun) {
            Write-Host "Would stop $description"
            Write-Host "  $($process.CommandLine)"
            continue
        }

        Write-Host "Stopping $description"
        Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
    }
}

if ($KeepTerminals) {
    exit 0
}

$terminalTitles = foreach ($carId in $resolvedCarIds) {
    "QCar fake vehicle $carId"
}

$terminalProcesses = Get-Process powershell -ErrorAction SilentlyContinue |
    Where-Object { $terminalTitles -contains $_.MainWindowTitle }

foreach ($terminal in $terminalProcesses) {
    $description = "PowerShell terminal '$($terminal.MainWindowTitle)' PID $($terminal.Id)"

    if ($DryRun) {
        Write-Host "Would close $description"
        continue
    }

    Write-Host "Closing $description"
    Stop-Process -Id $terminal.Id -Force -ErrorAction SilentlyContinue
}
