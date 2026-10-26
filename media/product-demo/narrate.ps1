$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$voice = New-Object System.Speech.Synthesis.SpeechSynthesizer
$cn = $voice.GetInstalledVoices() | Where-Object { $_.VoiceInfo.Culture.Name -eq 'zh-CN' } | Select-Object -First 1
if (-not $cn) { throw 'Chinese offline SAPI voice required' }
$voice.SelectVoice($cn.VoiceInfo.Name)
$voice.Rate = 1
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(24000, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$dir = Join-Path $PSScriptRoot 'public/narration'
[System.IO.Directory]::CreateDirectory($dir) | Out-Null
$script = Get-Content (Join-Path $PSScriptRoot 'script.json') -Raw | ConvertFrom-Json
try {
    foreach ($scene in $script) {
        $i = 0
        foreach ($line in $scene.lines) {
            $file = Join-Path $dir ($scene.id + '-' + $i + '.wav')
            $voice.SetOutputToWaveFile($file,$format)
            $voice.Speak([string]$line)
            $voice.SetOutputToNull()
            $i++
        }
    }
} finally { $voice.Dispose() }
Write-Host 'Offline synthetic Chinese narration rendered; no microphone or network used.'
