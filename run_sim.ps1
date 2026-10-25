param(
    [ValidateSet('gazebo','mujoco')][string]$Backend = 'mujoco',
    [ValidateSet('auto','north','south','side','legacy')][string]$Scenario = 'auto',
    [ValidateSet('auto','rl','kinematic')][string]$MotionMode = 'auto',
    [switch]$VirtualGraspComparison,
    [switch]$Rviz,
    [switch]$NoRviz,
    [switch]$Manual,
    [switch]$Background,
    [switch]$Headless
)
$ErrorActionPreference = 'Stop'
if ($MotionMode -eq 'auto') { $MotionMode = if ($Backend -eq 'mujoco') { 'rl' } else { 'kinematic' } }
if ($Backend -eq 'gazebo' -and $MotionMode -eq 'rl') { throw 'RL dynamics currently requires MuJoCo.' }
$domainId = if ($Backend -eq 'mujoco') { '8' } else { '7' }
$running = docker exec qda-dev python3 /ws/scripts/stop_sim.py --backend $Backend --check
if ($LASTEXITCODE -eq 0) { throw "$Backend is already running. Use .\stop_sim.ps1 -Backend $Backend first." }
if ($MotionMode -eq 'rl') {
    docker exec -e PYTHONPATH=/opt/qda-rl-v2 qda-dev python3 /ws/scripts/check_v2_scene_assets.py
    if ($LASTEXITCODE -ne 0) { throw 'V2 assets/dependencies are not ready; launch canceled.' }
}
$guiValue = if ($Headless) { 'false' } else { 'true' }
$rvizValue = if ($NoRviz -or $Headless) { 'false' } else { 'true' }
$taskValue = if ($Manual -or ($MotionMode -eq 'rl' -and -not $VirtualGraspComparison)) { 'false' } else { 'true' }
$virtualValue = if ($VirtualGraspComparison) { 'true' } else { 'false' }
$outputDir = if ($MotionMode -eq 'rl') { '/ws/output/v2_scene' } elseif ($Backend -eq 'mujoco') { '/ws/output/mujoco' } else { '/ws/output' }
if ($MotionMode -eq 'rl') { Write-Host 'Experimental RL factory scene: Nav2 + physical arms. Navigation uses simulation truth. Pick/place requires explicit -VirtualGraspComparison until physical grasp validation.' }
$launchCommand = "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; cd /ws; exec ros2 launch qda_bringup pick_place.launch.py backend:=$Backend motion_mode:=$MotionMode allow_virtual_grasp:=$virtualValue scenario:=$Scenario gui:=$guiValue rviz:=$rvizValue run_task:=$taskValue output_dir:=$outputDir"
if ($Background) {
    $logPath = if ($Backend -eq 'mujoco') { '/ws/output/mujoco_session.log' } else { '/ws/output/gazebo_session.log' }
    docker exec -d -e PYTHONPATH=/opt/qda-rl-v2 -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 qda-dev bash -lc "$launchCommand > $logPath 2>&1"
    Write-Host "$Backend started on ROS_DOMAIN_ID=$domainId. Log: $logPath"
} else {
    docker exec -it -e PYTHONPATH=/opt/qda-rl-v2 -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 qda-dev bash -lc $launchCommand
}
exit $LASTEXITCODE
