param([switch]$WithSimulation, [switch]$Headless)
$ErrorActionPreference = 'Stop'
docker exec qda-dev python3 /ws/scripts/prepare_v2_brain.py
if ($LASTEXITCODE -ne 0) { throw 'Brain assets are incomplete or failed verification. Run scripts/setup_v2_brain.sh inside qda-dev first.' }
$health = 'import urllib.request,json; print(json.load(urllib.request.urlopen("http://127.0.0.1:8088/health",timeout=2))["status"])'
docker exec qda-dev python3 -c $health 2>$null | Out-Null
if ($LASTEXITCODE -ne 0) {
    docker exec -d qda-dev bash -lc 'exec bash /ws/scripts/run_local_vlm.sh > /ws/output/v2_brain/llama_server.log 2>&1'
    $ready = $false
    for ($attempt=0; $attempt -lt 30; $attempt++) {
        Start-Sleep -Seconds 2
        docker exec qda-dev python3 -c $health 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { $ready = $true; break }
    }
    if (-not $ready) { throw 'Local model is not ready; inspect output/v2_brain/llama_server.log.' }
}
Write-Host 'SmolVLM Q8 + projector ready at container loopback 127.0.0.1:8088.'
if ($WithSimulation) {
    & (Join-Path $PSScriptRoot 'run_sim.ps1') -Brain -Manual -Background -Headless:$Headless
} else {
    Write-Host 'Start the simulation with: .\run_sim.ps1 -Brain -Manual -Background'
}
