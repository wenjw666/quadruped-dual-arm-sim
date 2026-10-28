param(
    [ValidateSet('perimeter','destinations')][string]$Mode = 'perimeter',
    [string]$Take = (Get-Date -Format 'yyyyMMdd-HHmmss')
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
if ($Take -notmatch '^[a-zA-Z0-9_-]+$') { throw 'Take must contain only letters, digits, underscore or hyphen.' }
$outPath = "/ws/output/navigation/$Mode-$Take"
docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 param set /qda_mujoco capture_profile navigation; ros2 param set /qda_mujoco demo_camera_mode inside; ros2 param set /qda_mujoco render_images true; ros2 param set /qda_mujoco demo_camera true; ros2 param set /qda_mujoco mapping_rgbd true'
if ($LASTEXITCODE -ne 0) { throw 'Unable to enable observation cameras.' }
docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/run_navigation_showcase.py --out $outPath --mode $Mode"
if ($LASTEXITCODE -ne 0) { throw "Tour did not pass. Inspect $outPath/report.json" }
docker exec $ContainerName ffmpeg -hide_banner -loglevel error -y -i "$outPath/live.avi" -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p -movflags +faststart "$outPath/navigation.mp4"
if ($LASTEXITCODE -ne 0) { throw 'Video conversion failed.' }
docker exec -e ROS_DOMAIN_ID=8 $ContainerName bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; python3 /ws/scripts/audit_color_map.py; python3 /ws/scripts/render_colored_map.py'
if ($LASTEXITCODE -ne 0) { throw 'Map artifact export failed.' }
Write-Host "Validated: $outPath/navigation.mp4 ; map: /ws/output/mapping/colored_map.html"
