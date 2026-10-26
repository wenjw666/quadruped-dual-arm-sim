# Run after prepare.py and the model / RViz clips have been produced.
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
$priorTemp = $env:TEMP
$priorTmp = $env:TMP
try {
    $renderTemp = Join-Path $PSScriptRoot '../../output/video_tmp'
    New-Item -ItemType Directory -Force $renderTemp | Out-Null
    $env:TEMP = [System.IO.Path]::GetFullPath($renderTemp)
    $env:TMP = $env:TEMP
    foreach ($name in @('hero','replay','rl','left','right','rviz')) {
        New-Item -ItemType Directory -Force "public/frames/$name" | Out-Null
        ffmpeg -y -v error -i "public/$name.mp4" -vf fps=15 -q:v 3 -start_number 0 "public/frames/$name/%05d.jpg"
        if ($LASTEXITCODE -ne 0) { throw "Frame extraction failed: $name" }
    }
    & .\node_modules\.bin\remotion.cmd render src/index.ts ProductDemo out/qda-v2-product-demo-raw.mp4 --codec h264 --pixel-format yuv420p --crf 20 --concurrency 4
    if ($LASTEXITCODE -ne 0) { throw 'Remotion rendering failed' }
    python finalize_audio.py
    if ($LASTEXITCODE -ne 0) { throw 'Final audio mix failed' }
} finally { $env:TEMP=$priorTemp; $env:TMP=$priorTmp; Pop-Location }
