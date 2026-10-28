param([switch]$FastValidation)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
# Start the project first with: .\scripts\windows\run_sim.ps1 -Brain -Manual -Background
$rosEnv = 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; '
$originalRender = $null
try {
    if ($FastValidation) {
        $originalRender = (docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc ($rosEnv + 'ros2 param get /qda_mujoco render_images --hide-type') | Out-String).Trim().ToLowerInvariant()
        if ($LASTEXITCODE -ne 0 -or $originalRender -notin @('true','false')) { throw 'Cannot read the current rendering setting.' }
        docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc ($rosEnv + 'ros2 param set /qda_mujoco render_images false')
        if ($LASTEXITCODE -ne 0) { throw 'Cannot configure the MuJoCo backend.' }
    }
    docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc ($rosEnv + 'python3 /ws/scripts/run_physical_pick_place.py')
    $taskExit = $LASTEXITCODE
} finally {
    if ($FastValidation -and $null -ne $originalRender) {
        docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc ($rosEnv + 'ros2 param set /qda_mujoco render_images ' + $originalRender)
    }
}
exit $taskExit
