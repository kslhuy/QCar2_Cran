[CmdletBinding()]
param(
    [string]$CondaEnv = "Qcar",
    [string]$GroundStationHost = "127.0.0.1",
    [int]$Port = 5000,
    [string]$CarIds = "0,1,2",
    [string]$Model = "kinematic",
    [string]$LongitudinalModel = "",
    [string]$SteeringModel = "",
    [string]$VehicleParams = "",
    [string]$TireModel = "",
    [string[]]$ExtraArgs = @(),
    [switch]$UseFleetPose,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function ConvertTo-SingleQuotedPowerShellLiteral {
    param([string]$Value)
    return $Value.Replace("'", "''")
}

function Add-OptionalArgument {
    param(
        [System.Collections.Generic.List[string]]$Arguments,
        [string]$Value
    )

    if (-not [string]::IsNullOrWhiteSpace($Value)) {
        [void]$Arguments.Add($Value)
    }
}

$resolvedCarIds = foreach ($carIdValue in ($CarIds -split ",")) {
    $trimmedCarId = $carIdValue.Trim()
    if (-not [string]::IsNullOrWhiteSpace($trimmedCarId)) {
        [int]$trimmedCarId
    }
}

$simulationDir = Join-Path $PSScriptRoot "qcar\simulation"
$fakeVehicleMain = Join-Path $simulationDir "fake_vehicle_real_logic.py"

if (-not (Test-Path -LiteralPath $fakeVehicleMain)) {
    throw "Could not find fake_vehicle_real_logic.py at: $fakeVehicleMain"
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

foreach ($carId in $resolvedCarIds) {
    $title = "QCar fake vehicle $carId"
    $titleLiteral = ConvertTo-SingleQuotedPowerShellLiteral $title
    $simulationDirLiteral = ConvertTo-SingleQuotedPowerShellLiteral $simulationDir
    $condaLiteral = ConvertTo-SingleQuotedPowerShellLiteral $condaExecutable
    $envLiteral = ConvertTo-SingleQuotedPowerShellLiteral $CondaEnv

    $fakeArgs = [System.Collections.Generic.List[string]]::new()
    [void]$fakeArgs.Add([string]$carId)
    [void]$fakeArgs.Add($GroundStationHost)
    [void]$fakeArgs.Add([string]$Port)
    Add-OptionalArgument -Arguments $fakeArgs -Value $Model
    Add-OptionalArgument -Arguments $fakeArgs -Value $LongitudinalModel
    Add-OptionalArgument -Arguments $fakeArgs -Value $SteeringModel
    Add-OptionalArgument -Arguments $fakeArgs -Value $VehicleParams
    Add-OptionalArgument -Arguments $fakeArgs -Value $TireModel
    if (-not $UseFleetPose) {
        [void]$fakeArgs.Add("--use-direct-poses")
    }
    foreach ($arg in $ExtraArgs) {
        Add-OptionalArgument -Arguments $fakeArgs -Value $arg
    }

    $argumentLiterals = foreach ($arg in $fakeArgs) {
        "'" + (ConvertTo-SingleQuotedPowerShellLiteral $arg) + "'"
    }
    $fakeArgumentText = $argumentLiterals -join " "

    $command = @"
`$Host.UI.RawUI.WindowTitle = '$titleLiteral'
Set-Location -LiteralPath '$simulationDirLiteral'
& '$condaLiteral' run --no-capture-output -n '$envLiteral' python .\fake_vehicle_real_logic.py $fakeArgumentText
`$exitCode = `$LASTEXITCODE
Write-Host ""
Write-Host "fake_vehicle_real_logic.py for car $carId exited with code `$exitCode"
"@

    if ($DryRun) {
        Write-Host "Would start terminal for fake car-id $carId"
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
