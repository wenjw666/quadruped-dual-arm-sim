$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
foreach ($backend in @('gazebo','mujoco')) {
    $status = docker exec $ContainerName python3 /ws/scripts/stop_sim.py --backend $backend --check
    if ($LASTEXITCODE -eq 0) { Write-Host "$backend is already running."; continue }
    & "$PSScriptRoot/run_sim.ps1" -Backend $backend -Background
}
Write-Host 'Gazebo uses domain 7; MuJoCo uses domain 8. Each has its own RViz and output files.'
