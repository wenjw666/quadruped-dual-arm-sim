param(
    [ValidateRange(1,600)][int]$Seconds = 30,
    [switch]$Uncalibrated
)
$ErrorActionPreference = 'Stop'
# Separate /v2 namespace; this does not launch Nav2 or the V1 demonstration.
$benchPolicy = '/ws/output/v2/runs/b2_transfer_v1/initial_policy.onnx'
$benchCalibration = if ($Uncalibrated) { '' } else { ' -p command_response:=/ws/output/v2_audit/b2_command_response.json' }
$benchCommand = "set -e; source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 -m qda_locomotion.rl_bench_node --ros-args -p policy_path:=$benchPolicy -p experimental:=true -p max_seconds:=$Seconds.0$benchCalibration"
docker exec -e PYTHONPATH=/opt/qda-rl-v2 -e OMP_NUM_THREADS=2 qda-dev bash -lc $benchCommand
if ($LASTEXITCODE -ne 0) { throw 'V2 RL bench failed; see the error above.' }
