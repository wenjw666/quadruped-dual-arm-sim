param([ValidateSet('gazebo','mujoco','all')][string]$Backend = 'mujoco')
docker exec qda-dev python3 /ws/scripts/stop_sim.py --backend $Backend
exit $LASTEXITCODE

