[CmdletBinding()]
param(
    [ValidateSet('menu', 'check', 'train', 'quick', 'full', 'web', 'closed-loop', 'closed-loop-web', 'replay')]
    [string]$Action = 'menu',
    [string]$Python = '',
    [string]$CondaEnv = 'Qcar',
    [string]$Checkpoint = '',
    [string]$Results = '',
    [ValidateRange(1024, 65535)][int]$Port = 3000,
    [switch]$NoBrowser,
    [switch]$Smoke
)
$ErrorActionPreference = 'Stop'
$workflow = Join-Path $PSScriptRoot 'Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/workflow.py'

try {
    if (-not $Python) {
        # Locate the named environment even when conda activate has not been run.
        $candidate = Join-Path $env:USERPROFILE ".conda/envs/$CondaEnv/python.exe"
        if (Test-Path -LiteralPath $candidate) {
            $Python = $candidate
        } elseif (Get-Command conda -ErrorAction SilentlyContinue) {
            $envInfo = (& conda env list --json | Out-String | ConvertFrom-Json)
            $match = $envInfo.envs | Where-Object { (Split-Path $_ -Leaf) -eq $CondaEnv } | Select-Object -First 1
            if ($match) { $Python = Join-Path $match 'python.exe' }
        }
        if (-not $Python) {
            throw "Cannot locate conda environment '$CondaEnv'. Use -Python 'C:/path/to/python.exe' or follow ROBUST_KALMANNET_WORKFLOW.md."
        }
    }
    $runArgs = @('-u', $workflow, $Action, '--port', [string]$Port)
    if ($Checkpoint) { $runArgs += @('--checkpoint', $Checkpoint) }
    if ($Results) { $runArgs += @('--results', $Results) }
    if ($NoBrowser) { $runArgs += '--no-browser' }
    if ($Smoke) { $runArgs += '--smoke' }
    & $Python @runArgs
    exit $LASTEXITCODE
} catch {
    Write-Host "ERROR: $_" -ForegroundColor Red
    exit 1
}
