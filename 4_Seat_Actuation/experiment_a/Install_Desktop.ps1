param(
    [Parameter(Mandatory)][string]$PythonPath,      # e.g. <venv>\Scripts\python.exe
    [Parameter(Mandatory)][string]$CollectorRoot,   # BeamNG_Collector_v0_3 folder (contains desktop_app.py)
    [string]$OutputRoot = (Join-Path ([Environment]::GetFolderPath('Desktop')) 'BeamNG Bolster A Data'),
    [switch]$ResetSettings
)
$ErrorActionPreference = 'Stop'
$packageRoot = Split-Path -Parent $PSScriptRoot
$pythonWindowPath = Join-Path (Split-Path -Parent $PythonPath) 'pythonw.exe'
if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) { throw 'Python executable not found' }
if (-not (Test-Path -LiteralPath $pythonWindowPath -PathType Leaf)) { throw 'pythonw.exe not found' }
if (-not (Test-Path -LiteralPath (Join-Path $CollectorRoot 'desktop_app.py') -PathType Leaf)) { throw 'BeamNG collector not found' }
$installDirectory = Join-Path $env:LOCALAPPDATA 'OASIS\ExperimentA'
[IO.Directory]::CreateDirectory($installDirectory) | Out-Null
$profilePath = Join-Path $installDirectory 'desktop_profile.json'
Push-Location -LiteralPath $packageRoot
try {
    $deploymentArguments = @('-X','utf8','-m','experiment_a.deployment','--source-root',$packageRoot,'--install-root',$installDirectory,'--collector-root',$CollectorRoot,'--output-root',$OutputRoot)
    if ($ResetSettings) { $deploymentArguments += '--reset-settings' }
    & $PythonPath @deploymentArguments
    if ($LASTEXITCODE -ne 0) { throw 'Versioned application installation failed' }
}
finally {
    Pop-Location
}
$desktopDirectory = [Environment]::GetFolderPath('Desktop')
$shortcutPath = Join-Path $desktopDirectory 'BeamNG Bolster A.lnk'
$shortcutShell = New-Object -ComObject WScript.Shell
$shortcut = $shortcutShell.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $pythonWindowPath
$shortcut.Arguments = '"' + (Join-Path $installDirectory 'bootstrap.py') + '" --profile "' + $profilePath + '"'
$shortcut.WorkingDirectory = $installDirectory
$shortcut.Description = 'BeamNG auto motor control with vehicle and 12-channel pressure recording'
$shortcut.WindowStyle = 1
$shortcut.Save()
[ordered]@{ shortcut=$shortcutPath; profile=$profilePath; launched=$false } | ConvertTo-Json
