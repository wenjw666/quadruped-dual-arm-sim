# A thin dispatcher preserves native PowerShell parameter binding in each command.
# Usage: .\scripts\windows\qda.ps1 sim -Brain -Mapping -Background
if ($args.Count -eq 0) { $action = 'help' } else { $action = $args[0] }
$commands = @{
    sim='run_sim.ps1'; stop='stop_sim.ps1'; brain='start_brain.ps1'
    pick='run_pick_place.ps1'; navigation='run_navigation_demo.ps1'
    rviz='show_rviz.ps1'; model='show_robot_model.ps1'; compare='run_compare.ps1'
    scenarios='run_scenarios.ps1'; train='run_v2_training.ps1'; bench='run_v2_rl_bench.ps1'
    voice='synthesize_offline_voice.ps1'; image='image.ps1'
    'product-render'='product_render.ps1'; 'product-narrate'='product_narrate.ps1'
}
if ($action -eq 'help') {
    Write-Output 'Usage: .\scripts\windows\qda.ps1 <command> [original parameters]'
    Write-Output ('Commands: ' + (($commands.Keys | Sort-Object) -join ', '))
    Write-Output 'Optional QDA_CONTAINER environment variable selects a different container.'
    exit 0
}
if (-not $commands.ContainsKey($action)) { throw "Unknown command: $action. Use help." }
$forwarded = @($args | Select-Object -Skip 1)
$engine = (Get-Process -Id $PID).Path
& $engine -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot $commands[$action]) @forwarded
exit $LASTEXITCODE
