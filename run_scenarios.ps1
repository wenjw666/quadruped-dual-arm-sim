param(
    [ValidateSet('auto','north','south','side','legacy')][string[]]$Scenarios = @('south','north','side','auto'),
    [ValidateRange(1,100)][int]$Repeat = 1
)
$ErrorActionPreference = 'Stop'
$profiles = $Scenarios -join ' '
docker exec qda-dev bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/run_scenario_suite.py --scenarios $profiles --repeat $Repeat"
exit $LASTEXITCODE
