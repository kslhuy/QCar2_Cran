[CmdletBinding()]
param(
    [string]$CondaEnv = "Qcar",
    [string]$GroundStationHost = "127.0.0.1",
    [int]$Port = 5000,
    [int[]]$CarIds = @(0, 1, 2),
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function ConvertTo-SingleQuotedPowerShellLiteral {
    param([string]$Value)
    return $Value.Replace("'", "''")
}

$qcarDir = Join-Path $PSScriptRoot "qcar"
$vehicleMain = Join-Path $qcarDir "vehicle_main.py"

if (-not (Test-Path -LiteralPath $vehicleMain)) {
    throw "Could not find vehicle_main.py at: $vehicleMain"
}

$condaCommand = Get-Command conda -ErrorAction SilentlyContinue
if (-not $condaCommand -and -not $env:CONDA_EXE) {
    throw "Could not find conda. Open Anaconda Prompt or add conda to PATH, then run this script again."
}

if ($env:CONDA_EXE -and (Test-Path -LiteralPath $env:CONDA_EXE)) {
    $condaExecutable = $env:CONDA_EXE
} elseif ($condaCommand.CommandType -in @("Application", "ExternalScript")) {
    $condaExecutable = $condaCommand.Source
} else {
    $condaExecutable = "conda"
}

foreach ($carId in $CarIds) {
    $title = "QCar vehicle $carId"
    $titleLiteral = ConvertTo-SingleQuotedPowerShellLiteral $title
    $qcarDirLiteral = ConvertTo-SingleQuotedPowerShellLiteral $qcarDir
    $condaLiteral = ConvertTo-SingleQuotedPowerShellLiteral $condaExecutable
    $envLiteral = ConvertTo-SingleQuotedPowerShellLiteral $CondaEnv
    $hostLiteral = ConvertTo-SingleQuotedPowerShellLiteral $GroundStationHost

    $command = @"
`$Host.UI.RawUI.WindowTitle = '$titleLiteral'
Set-Location -LiteralPath '$qcarDirLiteral'
& '$condaLiteral' run --no-capture-output -n '$envLiteral' python .\vehicle_main.py --host '$hostLiteral' --port $Port --car-id $carId
`$exitCode = `$LASTEXITCODE
Write-Host ""
Write-Host "vehicle_main.py for car $carId exited with code `$exitCode"
"@

    if ($DryRun) {
        Write-Host "Would start terminal for car-id $carId"
        Write-Host $command
        Write-Host ""
        continue
    }

    Start-Process powershell.exe -ArgumentList @(
        "-NoExit",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-Command",
        $command
    )
}
