# Stop all vehicle processes (Car_0 to Car_3)
Write-Host "Stopping all vehicle processes (Car_0 to Car_3)..." -ForegroundColor Yellow

$totalStopped = 0

# Stop vehicles Car_0 to Car_3
for ($i = 0; $i -le 3; $i++) {
    Write-Host "Stopping Car_$i..." -ForegroundColor Cyan
    
    # Use WMI to find python processes with vehicle_main.py and matching car-id
    # Note: Use wildcard matching for python path (works with venv)
    $vehicleProcesses = Get-WmiObject Win32_Process | Where-Object {
        ($_.CommandLine -like "*python.exe*vehicle_main.py*--car-id $i*")
    }
    
    if ($vehicleProcesses) {
        foreach ($process in $vehicleProcesses) {
            Write-Host "  Stopping PID $($process.ProcessId)" -ForegroundColor Gray
            Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
            $totalStopped++
        }
        Write-Host "  Car_$i stopped." -ForegroundColor Green
    } else {
        Write-Host "  Car_$i process not found." -ForegroundColor Yellow
    }
}

if ($totalStopped -gt 0) {
    Write-Host "`n$totalStopped vehicle process(es) stopped." -ForegroundColor Green
} else {
    Write-Host "`nNo vehicle processes found by car-id. Trying alternative method..." -ForegroundColor Yellow
    
    # Fallback: Stop all vehicle_main.py processes
    $allVehicleProcesses = Get-WmiObject Win32_Process | Where-Object {
        $_.CommandLine -like "*vehicle_main.py*"
    }
    
    if ($allVehicleProcesses) {
        Write-Host "Found $($allVehicleProcesses.Count) vehicle_main.py process(es):" -ForegroundColor Cyan
        foreach ($process in $allVehicleProcesses) {
            Write-Host "  Stopping PID $($process.ProcessId): car-id $(if ($process.CommandLine -match '--car-id (\d+)') { $matches[1] } else { 'unknown' })" -ForegroundColor Gray
            Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
            $totalStopped++
        }
        Write-Host "$totalStopped vehicle process(es) stopped." -ForegroundColor Green
    } else {
        Write-Host "No vehicle processes running." -ForegroundColor Yellow
    }
}

Write-Host "`nDone!" -ForegroundColor Green
