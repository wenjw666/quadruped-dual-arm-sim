param([switch]$Refresh)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($Refresh) {
    docker exec $ContainerName bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/render_refined_robot.py'
    if ($LASTEXITCODE -ne 0) { throw 'Model generation failed.' }
}
docker exec $ContainerName python3 /ws/scripts/stop_model_viewer.py
docker exec -d -e DISPLAY=host.docker.internal:0.0 $ContainerName bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; exec python3 /ws/scripts/robot_model_viewer.py > /ws/output/model_refinement_v1/viewer.log 2>&1'
Write-Host 'Robot review: drag to orbit, scroll to zoom. O opens grippers, C closes them, R resets the camera.'
