[CmdletBinding()]
param(
    [string]$GodotExe = $env:GODOT_EXE,
    [switch]$Rviz,
    [int]$SceneSeed = 42,
    [ValidateRange(1, 120)][int]$BridgeTimeoutSeconds = 30,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot

function Find-Godot {
    if ($GodotExe) {
        if (-not (Test-Path -LiteralPath $GodotExe -PathType Leaf)) {
            throw "Godot executable not found: $GodotExe"
        }
        return (Resolve-Path -LiteralPath $GodotExe).Path
    }
    foreach ($name in @('godot', 'godot4', 'godot.exe', 'godot4.exe')) {
        $command = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue
        if ($command) { return $command.Source }
    }
    throw 'Godot is not on PATH. Pass -GodotExe ''C:\path\to\Godot.exe'' or set $env:GODOT_EXE to your executable path.'
}

function Test-Rosbridge {
    $socket = [System.Net.WebSockets.ClientWebSocket]::new()
    $timeout = [System.Threading.CancellationTokenSource]::new()
    $timeout.CancelAfter(2000)
    try {
        $socket.ConnectAsync([uri]'ws://127.0.0.1:9090', $timeout.Token).GetAwaiter().GetResult()
        return $socket.State -eq [System.Net.WebSockets.WebSocketState]::Open
    }
    catch { return $false }
    finally {
        $socket.Dispose()
        $timeout.Dispose()
    }
}

$godotPath = Find-Godot
$versionOutput = & $godotPath --headless --version | Out-String
if ($LASTEXITCODE -ne 0) { throw 'Could not run Godot to read its version.' }
$versionText = ($versionOutput -join "`n").Trim()
if ($versionText -notmatch '^4\.(\d+)\.' -or [int]$Matches[1] -lt 6) {
    throw "This project requires Godot 4.6 or newer in the 4.x series. Found: $versionText"
}
Write-Host "Godot: $godotPath ($versionText)"

if (-not (Get-Command docker -CommandType Application -ErrorAction SilentlyContinue)) {
    throw 'Docker Desktop is required. Install it and start its Linux engine first.'
}
$dockerOS = & docker info --format '{{.OSType}}' | Out-String
if ($LASTEXITCODE -ne 0) {
    throw 'Docker engine is unavailable. Start Docker Desktop, select Linux containers, and wait until docker version shows a Server section.'
}
if ($dockerOS.Trim() -ne 'linux') {
    throw 'Docker is using Windows containers. Switch Docker Desktop to Linux containers before starting ROS.'
}

if ($env:URC_DETECTOR -eq 'yolo') {
    foreach ($modelFile in @('weights/hammer_bottle_yolov8n.onnx', 'weights/hammer_bottle_yolov8n.classes.json')) {
        if (-not (Test-Path -LiteralPath (Join-Path $projectRoot $modelFile) -PathType Leaf)) {
            throw "YOLO is not ready: $modelFile is missing. Train and export the bottle/hammer model first, or set `$env:URC_DETECTOR = 'oracle'."
        }
    }
}

Push-Location $projectRoot
try {
    if (-not $CheckOnly) {
        # Stop the old browser simulator to avoid port conflicts and two controllers.
        & docker compose stop desktop
        if ($LASTEXITCODE -ne 0) { throw 'Could not stop the previous desktop service.' }
        if (-not $Rviz) {
            & docker compose stop rviz
            if ($LASTEXITCODE -ne 0) { throw 'Could not stop the optional RViz viewer.' }
        }
        $services = @('ros')
        if ($Rviz) { $services += 'rviz' }
        # A new Godot process restarts simulation time, epochs, and sensor sequences.
        & docker compose up --build -d --force-recreate @services
        if ($LASTEXITCODE -ne 0) { throw 'Docker startup failed. Check the build output above.' }
    }

    $deadline = [DateTime]::UtcNow.AddSeconds($BridgeTimeoutSeconds)
    $connected = Test-Rosbridge
    while (-not $connected -and [DateTime]::UtcNow -lt $deadline) {
        Start-Sleep -Milliseconds 500
        $connected = Test-Rosbridge
    }
    if (-not $connected) {
        throw 'No rosbridge WebSocket at ws://127.0.0.1:9090. Run docker compose ps and docker compose logs ros. Use the ros service; the legacy desktop service does not publish port 9090.'
    }
    Write-Host 'ROS bridge: connected at ws://127.0.0.1:9090'
    if ($CheckOnly) { return }

    & $godotPath --headless --path (Join-Path $projectRoot 'godot') --import | Out-Host
    if ($LASTEXITCODE -ne 0) { throw 'Godot asset import failed. Check the errors above.' }
    if ($Rviz) { Write-Host 'Optional RViz view: http://localhost:6080/vnc.html?autoconnect=true&resize=scale' }
    Write-Host 'Starting native Godot. Click Start after ROS CONNECTED appears. ROS keeps running after Godot closes; stop it with docker compose --profile rviz down.'
    & $godotPath --path (Join-Path $projectRoot 'godot') -- "--seed=$SceneSeed" '--bridge=ws://127.0.0.1:9090' | Out-Host
    if ($LASTEXITCODE -ne 0) { throw "Godot exited with code $LASTEXITCODE." }
}
finally { Pop-Location }
