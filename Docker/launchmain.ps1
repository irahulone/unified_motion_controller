# Set script to stop on error
$ErrorActionPreference = "Stop"

# Path to VcXsrv executable
$vcxsrvPath = "C:\Program Files\VcXsrv\vcxsrv.exe"
$configFile = "$env:USERPROFILE\.vcxsrv_config.xlaunch"

# Check if VcXsrv is already running
$vcxsrvRunning = Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue

if (-not $vcxsrvRunning) {
    Write-Host "Starting VcXsrv..."

    # Start VcXsrv with reasonable defaults if not already running
    if (Test-Path $vcxsrvPath) {
        Start-Process -FilePath $vcxsrvPath -ArgumentList ":0 -multiwindow -ac -nowgl" -NoNewWindow
        Start-Sleep -Seconds 3
    } else {
        Write-Error "VcXsrv not found at $vcxsrvPath. Please install VcXsrv or update the path."
        exit 1
    }
} else {
    Write-Host "VcXsrv is already running."
}

# Set DISPLAY for Docker
$env:DISPLAY = "host.docker.internal:0.0"

# Run docker compose
Write-Host "Running Docker Compose..."
docker compose -f "docker-compose.run.yml" up --build
