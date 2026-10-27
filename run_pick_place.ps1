param([switch]$FastValidation)
$ErrorActionPreference = 'Stop'
# Start the project first with: .\run_sim.ps1 -Brain -Manual -Background
$rosEnv = 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; '
try {
    if ($FastValidation) {
        docker exec -e ROS_DOMAIN_ID=8 qda-dev bash -lc ($rosEnv + 'ros2 param set /qda_mujoco render_images false')
        if ($LASTEXITCODE -ne 0) { throw 'Cannot configure the MuJoCo backend.' }
    }
    docker exec -e ROS_DOMAIN_ID=8 qda-dev bash -lc ($rosEnv + 'python3 /ws/scripts/run_physical_pick_place.py')
    $taskExit = $LASTEXITCODE
} finally {
    if ($FastValidation) {
        docker exec -e ROS_DOMAIN_ID=8 qda-dev bash -lc ($rosEnv + 'ros2 param set /qda_mujoco render_images true')
    }
}
exit $taskExit
