param(
    [ValidateSet('Build', 'Upload', 'List')]
    [string]$Action = 'Build',
    [string]$StLinkSerial = '',
    [string]$ToolchainPath = '',
    [string]$CubePath = '',
    [switch]$PressureOnly,
    [switch]$FallbackControl,
    [string]$CMakePath = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin\cmake.exe',
    [string]$NinjaPath = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin\ninja.exe',
    [string]$ProgrammerPath = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
)

$ErrorActionPreference = 'Stop'
if ($PressureOnly -and $FallbackControl) { throw 'PressureOnly and FallbackControl are mutually exclusive.' }
$workspacePath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$buildPath = Join-Path $workspacePath 'build'
$firmwarePath = Join-Path $buildPath 'fsr_mux_h723.elf'
if ($PressureOnly) {
    $buildPath = Join-Path $workspacePath 'build_pressure_only'
    $firmwarePath = Join-Path $buildPath 'pressure_only_h723.elf'
}
if ($FallbackControl) {
    $buildPath = Join-Path $workspacePath 'build_fallback'
    $firmwarePath = Join-Path $buildPath 'fallback_control_h723.elf'
}

function Assert-Tool([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Required tool was not found: $Path"
    }
}

function Invoke-Checked([string]$Program, [string[]]$ProgramArguments) {
    & $Program @ProgramArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $Program"
    }
}

try {
    if ($Action -eq 'Upload' -and [string]::IsNullOrWhiteSpace($StLinkSerial)) {
        throw 'Upload requires -StLinkSerial. Use -Action List to identify the intended board.'
    }
    if ($Action -eq 'List') {
        Assert-Tool $ProgrammerPath
        Invoke-Checked $ProgrammerPath @('-l', 'st-link')
        exit 0
    }

    Assert-Tool $CMakePath
    Assert-Tool $NinjaPath
    if ($Action -eq 'Upload') {
        Assert-Tool $ProgrammerPath
    }

    # This ARM GCC installation cannot use a non-ASCII temporary path.
    # These variables affect only this PowerShell process and its children.
    $taskTempPath = Join-Path $env:SystemDrive 'Temp\seat_control_data_acquisition'
    New-Item -ItemType Directory -Path $taskTempPath -Force | Out-Null
    $env:TMP = $taskTempPath
    $env:TEMP = $taskTempPath

    $configureArguments = @(
        '-S', $workspacePath, '-B', $buildPath, '-G', 'Ninja',
        "-DCMAKE_MAKE_PROGRAM=$NinjaPath", '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
    )
    if ($ToolchainPath) { $configureArguments += "-DTOOLCHAIN=$ToolchainPath" }
    if ($CubePath) { $configureArguments += "-DCUBE=$CubePath" }
    $configureArguments += "-DOASIS_PRESSURE_ONLY=$([int][bool]$PressureOnly)"
    $configureArguments += "-DOASIS_FALLBACK_CONTROL=$([int][bool]$FallbackControl)"
    Invoke-Checked $CMakePath $configureArguments
    Invoke-Checked $CMakePath @('--build', $buildPath)

    if ($Action -eq 'Upload') {
        # Always build successfully before writing the selected ST-LINK target.
        Invoke-Checked $ProgrammerPath @(
            '-c', 'port=SWD', 'mode=UR', 'reset=HWrst', "sn=$StLinkSerial",
            '-d', $firmwarePath, '-v', '-rst'
        )
        Write-Host 'Upload finished: firmware written, verified and target reset.'
    } else {
        Write-Host "Build finished: $firmwarePath"
    }
    exit 0
} catch {
    Write-Error $_ -ErrorAction Continue
    exit 1
}
