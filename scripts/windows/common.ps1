$ErrorActionPreference = 'Stop'
$WorkspaceRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$ContainerName = if ($env:QDA_CONTAINER) { $env:QDA_CONTAINER } else { 'qda-dev' }
Set-Location -LiteralPath $WorkspaceRoot
