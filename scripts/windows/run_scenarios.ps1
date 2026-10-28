param(
    [ValidateSet('auto','north','south','side','legacy')][string[]]$Scenarios = @('south','north','side','auto'),
    [ValidateRange(1,100)][int]$Repeat = 1
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$profiles = $Scenarios -join ' '
docker exec $ContainerName bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/run_scenario_suite.py --scenarios $profiles --repeat $Repeat"
exit $LASTEXITCODE
