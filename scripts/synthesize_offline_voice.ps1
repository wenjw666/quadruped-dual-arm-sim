param(
    [string]$Text = '查询状态',
    [string]$OutputFile = '',
    [switch]$TestSet
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$speaker = New-Object System.Speech.Synthesis.SpeechSynthesizer
$chineseVoice = $speaker.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture.Name -eq 'zh-CN' } | Select-Object -First 1
if (-not $chineseVoice) { throw 'No installed zh-CN SAPI voice. Install a Windows Chinese voice before offline synthesis.' }
$speaker.SelectVoice($chineseVoice.VoiceInfo.Name)
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(16000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$outDir = Join-Path (Split-Path $PSScriptRoot -Parent) 'output/v2_brain/voice'
[System.IO.Directory]::CreateDirectory($outDir) | Out-Null
$cases = if ($TestSet) {
    @(@{Name='status';Text='查询状态'}, @{Name='home';Text='左臂回位'}, @{Name='pause';Text='暂停'}, @{Name='north';Text='前往北侧'})
} else { @(@{Name='speech';Text=$Text}) }
$records = @()
try {
    foreach ($case in $cases) {
        $target = if ($OutputFile -and -not $TestSet) { [System.IO.Path]::GetFullPath($OutputFile) } else { Join-Path $outDir ($case.Name + '.wav') }
        $speaker.SetOutputToWaveFile($target,$format)
        $speaker.Speak([string]$case.Text)
        $speaker.SetOutputToNull()
        $records += @{ file=$target; text=$case.Text; source='Windows SAPI synthetic audio'; voice=$chineseVoice.VoiceInfo.Name; sample_rate=16000 }
    }
} finally { $speaker.Dispose() }
$records | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $outDir 'synthesis.json') -Encoding UTF8
$records | Format-Table
