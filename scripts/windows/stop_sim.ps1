param([ValidateSet('gazebo','mujoco','all')][string]$Backend = 'mujoco')
. (Join-Path $PSScriptRoot 'common.ps1')
docker exec $ContainerName python3 /ws/scripts/stop_sim.py --backend $Backend
exit $LASTEXITCODE
