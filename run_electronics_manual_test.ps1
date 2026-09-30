param(
    [ValidateRange(1, 1000000)]
    [int]$AuthorityComparisons = 1000,
    [string]$OutputPath = "simulation/results/electronics_manual_validation.json"
)

$ErrorActionPreference = "Stop"
$QcarPath = Join-Path $PSScriptRoot "Development/multi_vehicle_self_driving_RealQcar/qcar"

Push-Location $QcarPath
try {
    Write-Host "[1/4] Configuring native electronics core..."
    & conda run -n Qcar cmake -S electronics/native -B electronics/native/build -G Ninja
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed." }

    Write-Host "[2/4] Building shared SIL and portable static core..."
    & conda run -n Qcar cmake --build electronics/native/build
    if ($LASTEXITCODE -ne 0) { throw "Native build failed." }

    Write-Host "[3/4] Running protocol, V2V, parity and hardware-target tests..."
    & conda run -n Qcar python -m unittest `
        electronics.test_electronics_digital_twin `
        electronics.test_v2v_transport_integration `
        electronics.test_trust_observer_native `
        electronics.test_hardware_abstraction `
        -v
    if ($LASTEXITCODE -ne 0) { throw "Electronics unit/integration tests failed." }

    Write-Host "[4/4] Running end-to-end electronics validation..."
    & conda run -n Qcar python -m electronics.manual_validation `
        --scenario all `
        --authority-comparisons $AuthorityComparisons `
        --output $OutputPath
    if ($LASTEXITCODE -ne 0) { throw "Electronics validation failed." }
}
finally {
    Pop-Location
}
