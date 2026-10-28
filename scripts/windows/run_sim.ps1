param(
    [ValidateSet('gazebo','mujoco')][string]$Backend = 'mujoco',
    [ValidateSet('auto','north','south','side','legacy')][string]$Scenario = 'auto',
    [ValidateSet('auto','rl','kinematic')][string]$MotionMode = 'auto',
    [switch]$VirtualGraspComparison,
    [switch]$Brain,
    [switch]$ServoFixture,
    [switch]$Rviz,
    [switch]$NoRviz,
    [switch]$Manual,
    [switch]$Background,
    [switch]$NoPhysicsGui,
    [switch]$NoImages,
    [switch]$Mapping,
    [switch]$Headless
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($Mapping -and $NoImages) { throw 'RGB-D mapping requires images; omit -NoImages.' }
if ($Mapping -and $Backend -ne 'mujoco') { throw '-Mapping enables the MuJoCo RGB-D map. Gazebo retains its existing mapping pipeline.' }
if ($MotionMode -eq 'auto') { $MotionMode = if ($Backend -eq 'mujoco') { 'rl' } else { 'kinematic' } }
if ($Backend -eq 'gazebo' -and $MotionMode -eq 'rl') { throw 'RL dynamics currently requires MuJoCo.' }
$domainId = if ($Backend -eq 'mujoco') { '8' } else { '7' }
$running = docker exec $ContainerName python3 /ws/scripts/stop_sim.py --backend $Backend --check
if ($LASTEXITCODE -eq 0) { throw "$Backend is already running. Use .\scripts\windows\qda.ps1 stop -Backend $Backend first." }
if ($MotionMode -eq 'rl') {
    docker exec -e PYTHONPATH=/opt/qda-rl-v2 $ContainerName python3 /ws/scripts/check_v2_scene_assets.py
    if ($LASTEXITCODE -ne 0) { throw 'V2 assets/dependencies are not ready; launch canceled.' }
}
$guiValue = if ($Headless -or $NoPhysicsGui) { 'false' } else { 'true' }
$rvizValue = if ($NoRviz -or $Headless) { 'false' } else { 'true' }
$taskValue = if ($Manual -or ($MotionMode -eq 'rl' -and -not $VirtualGraspComparison)) { 'false' } else { 'true' }
$virtualValue = if ($VirtualGraspComparison) { 'true' } else { 'false' }
$outputDir = if ($MotionMode -eq 'rl') { '/ws/output/v2_scene' } elseif ($Backend -eq 'mujoco') { '/ws/output/mujoco' } else { '/ws/output' }
if ($MotionMode -eq 'rl') { Write-Host 'Experimental RL factory scene: Nav2 + contact-based pick/carry/place. Use -Brain for the physical task API. Localization/object poses use simulation truth.' }
$brainValue = if ($Brain) { 'true' } else { 'false' }
$fixtureValue = if ($ServoFixture) { 'true' } else { 'false' }
$imagesValue = if ($NoImages) { 'false' } else { 'true' }
$mappingValue = if ($Mapping) { 'true' } else { 'false' }
$launchCommand = "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; cd /ws; exec ros2 launch qda_bringup pick_place.launch.py backend:=$Backend motion_mode:=$MotionMode brain:=$brainValue servo_fixture:=$fixtureValue allow_virtual_grasp:=$virtualValue scenario:=$Scenario gui:=$guiValue rviz:=$rvizValue render_images:=$imagesValue mapping_rgbd:=$mappingValue run_task:=$taskValue output_dir:=$outputDir"
if ($Background) {
    $logPath = if ($Backend -eq 'mujoco') { '/ws/output/mujoco_session.log' } else { '/ws/output/gazebo_session.log' }
    docker exec -d -e PYTHONPATH=/opt/qda-rl-v2 -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 $ContainerName bash -lc "$launchCommand > $logPath 2>&1"
    if ($LASTEXITCODE -ne 0) { throw 'Unable to start the simulation in the selected container.' }
    Write-Host "$Backend started on ROS_DOMAIN_ID=$domainId. Log: $logPath"
} else {
    docker exec -it -e PYTHONPATH=/opt/qda-rl-v2 -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 $ContainerName bash -lc $launchCommand
}
exit $LASTEXITCODE
