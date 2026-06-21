[CmdletBinding()]
param(
    [string]$GroundStationHost = "127.0.0.1",
    [int]$Port = 5000,
    [int[]]$CarIds = @(0, 1, 2),
    [switch]$KeepTerminals,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Test-QuickVehicleProcess {
    param(
        [Microsoft.Management.Infrastructure.CimInstance]$Process,
        [int]$CarId
    )

    if (-not $Process.CommandLine) {
        return $false
    }

    $commandLine = $Process.CommandLine
    return (
        $commandLine -like "*vehicle_main.py*" -and
        $commandLine -like "*--host*" -and
        $commandLine -like "*$GroundStationHost*" -and
        $commandLine -like "*--port $Port*" -and
        $commandLine -like "*--car-id $CarId*"
    )
}

$allProcesses = Get-CimInstance Win32_Process
$targetProcesses = foreach ($carId in $CarIds) {
    $allProcesses | Where-Object { Test-QuickVehicleProcess -Process $_ -CarId $carId }
}

$targetProcesses = $targetProcesses |
    Where-Object { $_ } |
    Sort-Object -Property ProcessId -Unique

if (-not $targetProcesses) {
    Write-Host "No matching vehicle_main.py quick-test processes found."
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

$terminalTitles = foreach ($carId in $CarIds) {
    "QCar vehicle $carId"
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
