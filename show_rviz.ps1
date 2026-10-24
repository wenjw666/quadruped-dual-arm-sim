param([ValidateSet('gazebo','mujoco')][string]$Backend = 'mujoco')
$domainId = if ($Backend -eq 'mujoco') { '8' } else { '7' }
$outDir = if ($Backend -eq 'mujoco') { '/ws/output/mujoco' } else { '/ws/output' }
docker exec qda-dev python3 /ws/scripts/stop_sim.py --backend $Backend --viewer-only
$command = "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; mkdir -p $outDir; if [ ! -f $outDir/factory.rviz ]; then cp /ws/src/qda_bringup/rviz/factory.rviz $outDir/factory.rviz; fi; exec rviz2 -d $outDir/factory.rviz --ros-args -p use_sim_time:=true > $outDir/rviz.log 2>&1"
docker exec -d -e ROS_DOMAIN_ID=$domainId -e DISPLAY=host.docker.internal:0.0 qda-dev bash -lc $command

