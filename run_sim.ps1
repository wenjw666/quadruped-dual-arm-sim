param(
    [ValidateSet('gazebo','mujoco')][string]$Backend = 'mujoco',
    [ValidateSet('auto','north','south','side','legacy')][string]$Scenario = 'auto',
    [switch]$Rviz,
    [switch]$NoRviz,
    [switch]$Manual,
    [switch]$Background,
    [switch]$Headless
)
$ErrorActionPreference = 'Stop'
$domainId = if ($Backend -eq 'mujoco') { '8' } else { '7' }
$running = docker exec qda-dev python3 /ws/scripts/stop_sim.py --backend $Backend --check
if ($LASTEXITCODE -eq 0) { throw "$Backend is already running. Use .\stop_sim.ps1 -Backend $Backend first." }
$guiValue = if ($Headless) { 'false' } else { 'true' }
$rvizValue = if ($NoRviz -or $Headless) { 'false' } else { 'true' }
$taskValue = if ($Manual) { 'false' } else { 'true' }
$launchCommand = "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; cd /ws; exec ros2 launch qda_bringup pick_place.launch.py backend:=$Backend scenario:=$Scenario gui:=$guiValue rviz:=$rvizValue run_task:=$taskValue"
if ($Background) {
    $logPath = if ($Backend -eq 'mujoco') { '/ws/output/mujoco_session.log' } else { '/ws/output/gazebo_session.log' }
    docker exec -d -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 qda-dev bash -lc "$launchCommand > $logPath 2>&1"
    Write-Host "$Backend started on ROS_DOMAIN_ID=$domainId. Log: $logPath"
} else {
    docker exec -it -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 qda-dev bash -lc $launchCommand
}
exit $LASTEXITCODE
