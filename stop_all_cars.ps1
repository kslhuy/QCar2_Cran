# Stop all vehicle processes
Write-Host "Stopping all vehicle processes..." -ForegroundColor Yellow

# Find and kill all python processes running vehicle_main.py
$vehicleProcesses = Get-WmiObject Win32_Process -Filter "name = 'python.exe'" | Where-Object {
    $_.CommandLine -like "*vehicle_main.py*"
}

if ($vehicleProcesses) {
    foreach ($process in $vehicleProcesses) {
        Write-Host "Stopping vehicle process: PID $($process.ProcessId) - $($process.CommandLine)" -ForegroundColor Cyan
        Stop-Process -Id $process.ProcessId -Force
    }
    Write-Host "All vehicle processes stopped." -ForegroundColor Green
} else {
    Write-Host "No vehicle processes found." -ForegroundColor Yellow
}

# Optional: Also close the extra PowerShell windows that were opened
$currentPID = $PID
$allPowerShellWindows = Get-Process powershell -ErrorAction SilentlyContinue | Where-Object { $_.Id -ne $currentPID }

if ($allPowerShellWindows) {
    Write-Host "`nFound $($allPowerShellWindows.Count) other PowerShell windows." -ForegroundColor Yellow
    $closeWindows = Read-Host "Do you want to close them? (y/n)"
    if ($closeWindows -eq 'y' -or $closeWindows -eq 'Y') {
        $allPowerShellWindows | Stop-Process -Force
        Write-Host "PowerShell windows closed." -ForegroundColor Green
    }
}

Write-Host "`nDone!" -ForegroundColor Green
