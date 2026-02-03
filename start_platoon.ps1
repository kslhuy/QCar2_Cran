# 启动编队：先运行 initPlatoon.py，在确认配置完成后同时启动四辆车

# 获取当前脚本所在目录（项目根目录）
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Definition

# 路径配置
$projectRoot     = $scriptRoot
$pythonExe       = Join-Path $projectRoot "env\Scripts\python.exe"
$platoonFolder   = Join-Path $projectRoot "Development\QCar2_multi-vehicle_control"
$initScriptPath  = Join-Path $platoonFolder "initPlatoon.py"
$qcarFolder      = Join-Path $projectRoot "Development\multi_vehicle_self_driving_RealQcar\qcar"
$activateScript  = Join-Path $projectRoot "env\Scripts\Activate.ps1"

Write-Host "[start_platoon] 使用 Python: $pythonExe" -ForegroundColor Cyan
Write-Host "[start_platoon] 初始化脚本: $initScriptPath" -ForegroundColor Cyan

if (-not (Test-Path $pythonExe)) {
    Write-Error "找不到 Python 可执行文件: $pythonExe，请确认虚拟环境已创建。"
    exit 1
}

if (-not (Test-Path $initScriptPath)) {
    Write-Error "找不到 initPlatoon.py: $initScriptPath，请检查路径。"
    exit 1
}

if (-not (Test-Path $qcarFolder)) {
    Write-Error "找不到 QCar 工程目录: $qcarFolder，请检查路径。"
    exit 1
}

if (-not (Test-Path $activateScript)) {
    Write-Error "找不到虚拟环境激活脚本: $activateScript，请确认虚拟环境已正确创建。"
    exit 1
}

# 先运行 initPlatoon.py
Write-Host "[start_platoon] 正在运行 initPlatoon.py 配置编队参数..." -ForegroundColor Yellow

Push-Location $platoonFolder
try {
    # 运行 initPlatoon.py，并保留完整输出
    $initOutput = & $pythonExe $initScriptPath 2>&1 | Tee-Object -Variable initLines
}
finally {
    Pop-Location
}

# 检查输出中是否包含关键提示
$successPattern = "All vehicles configured with default transforms."

if ($initLines -match $successPattern) {
    Write-Host "[start_platoon] 检测到成功信息：'$successPattern'" -ForegroundColor Green
    Write-Host "[start_platoon] 即将同时启动四辆车..." -ForegroundColor Green

    # 直接在正确路径下启动 4 辆车（car-id 0~3）
    for ($carId = 0; $carId -lt 4; $carId++) {
        $cmd = "cd `"$qcarFolder`"; & `"$activateScript`"; python vehicle_main.py --car-id $carId"
        Write-Host "[start_platoon] 启动车辆 car-id=$carId" -ForegroundColor Cyan
        Start-Process powershell -ArgumentList "-NoExit", "-Command", $cmd
    }
}
else {
    Write-Warning "[start_platoon] initPlatoon.py 输出中未找到成功信息：'$successPattern'。"
    Write-Warning "[start_platoon] 为安全起见，未自动启动车辆。下面是 initPlatoon.py 的完整输出，便于排查："

    # 打印 initPlatoon.py 的全部输出，帮助调试
    if ($initLines) {
        Write-Host "[start_platoon] ===== initPlatoon.py 输出开始 =====" -ForegroundColor DarkCyan
        $initLines | ForEach-Object { Write-Host "  $_" }
        Write-Host "[start_platoon] ===== initPlatoon.py 输出结束 =====" -ForegroundColor DarkCyan
    }
}
