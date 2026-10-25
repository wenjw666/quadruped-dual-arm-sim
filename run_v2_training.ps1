param(
    [ValidateSet('stand','velocity')][string]$Task = 'stand',
    [ValidateRange(512,100000000)][int]$Steps = 8192,
    [ValidateRange(1,16)][int]$Envs = 2,
    [ValidateRange(0,2147483647)][int]$Seed = 7
)
$ErrorActionPreference = 'Stop'
$runName = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + $Task + '_' + [guid]::NewGuid().ToString('N').Substring(0,6)
# Training dependencies are isolated from the running ROS Python installation.
docker exec qda-dev python3 -m pip install --target /opt/qda-rl-v2 --no-deps -r /ws/docker/requirements-rl-v2.txt
if ($LASTEXITCODE -ne 0) { throw 'Training dependency installation failed' }
$command = "set -e; source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/build_v2_dynamics.py --training; python3 /ws/scripts/audit_v2_dynamics.py; python3 -m qda_locomotion.train_ppo --task $Task --steps $Steps --envs $Envs --seed $Seed --output /ws/output/v2/runs/$runName"
docker exec -e PYTHONPATH=/opt/qda-rl-v2 -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=2 qda-dev bash -lc $command
exit $LASTEXITCODE
