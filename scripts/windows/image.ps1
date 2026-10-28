param(
    [ValidateSet('load','start','snapshot','save')][string]$Mode = 'load',
    [string]$Image = 'qda-ros:humble-v0.2.0-container',
    [string]$Archive = 'qda-ros-humble-v0.2.0-container.tar'
)
. (Join-Path $PSScriptRoot 'common.ps1')
switch ($Mode) {
    load {
        if (-not (Test-Path -LiteralPath $Archive)) { throw "Archive missing: $Archive" }
        $checksumFile = "$Archive.sha256"
        if (Test-Path -LiteralPath $checksumFile) {
            $expected = ((Get-Content -LiteralPath $checksumFile -Raw).Trim() -split '\s+')[0]
            if ((Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash -ne $expected) { throw 'Archive checksum mismatch.' }
        }
        docker load --input $Archive
        if ($LASTEXITCODE -ne 0) { throw 'Image load failed.' }
    }
    start {
        $previous = $env:QDA_IMAGE
        try {
            $env:QDA_IMAGE = $Image
            docker compose -f (Join-Path $WorkspaceRoot 'compose.yaml') up -d --no-build
            if ($LASTEXITCODE -ne 0) { throw 'Container start failed.' }
        } finally { $env:QDA_IMAGE = $previous }
    }
    snapshot {
        # Pause briefly for filesystem consistency; stop simulation tasks first.
        docker commit $ContainerName $Image
        if ($LASTEXITCODE -ne 0) { throw 'Container snapshot failed.' }
    }
    save {
        if (Test-Path -LiteralPath $Archive) { throw "Refusing to replace existing archive: $Archive" }
        docker save --output $Archive $Image
        if ($LASTEXITCODE -ne 0) { throw 'Image export failed. Any partial archive must not be used for deployment.' }
        $hash = (Get-FileHash -LiteralPath $Archive -Algorithm SHA256).Hash.ToLowerInvariant()
        "$hash  $([IO.Path]::GetFileName($Archive))" | Set-Content -LiteralPath "$Archive.sha256" -Encoding ASCII
        Get-Item -LiteralPath $Archive
    }
}
