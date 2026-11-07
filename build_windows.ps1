# SPDX-License-Identifier: GPL-3.0-or-later
# Builds the OpenBeam core and MuJoCo plugin on Windows (MSVC + CMake), installs
# them to build\install and runs the core's tests. See README.md.
#
#   python tools\fetch_mujoco.py
#   powershell -ExecutionPolicy Bypass -File build_windows.ps1
param(
    [string]$MujocoRoot = '',
    [string]$BuildDir = '',
    [string]$Config = 'Release',
    [string]$VcVarsVer = '',   # e.g. 14.44 to pin the MSVC toolset; default: the newest installed
    [switch]$NoMujoco,
    [switch]$NoTests
)
$ErrorActionPreference = 'Stop'
$repo = $PSScriptRoot
if (-not $BuildDir) { $BuildDir = Join-Path $repo 'build' }
if (-not $MujocoRoot) { $MujocoRoot = Join-Path $repo 'third_party\mujoco-4.0.0' }

# The MSVC x64 environment, unless this shell already has one.
if (-not $env:VCINSTALLDIR) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vsPath) { throw 'No Visual Studio or Build Tools with the x64 C++ tools found' }
    $vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvarsall.bat'
    $ver = if ($VcVarsVer) { "-vcvars_ver=$VcVarsVer" } else { '' }
    foreach ($line in (cmd /c "`"$vcvars`" x64 $ver >nul 2>&1 && set")) {
        if ($line -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($matches[1])" -Value $matches[2] }
    }
}

$cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
if (-not $cmake) {  # winget installs CMake per user without always putting it on PATH
    $cmake = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\Kitware.CMake*\*\bin\cmake.exe" -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $cmake) { throw 'cmake not found: install CMake 3.24 or newer' }
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
$generator = if (Get-Command ninja -ErrorAction SilentlyContinue) { @('-G', 'Ninja', "-DCMAKE_BUILD_TYPE=$Config") }
             else { @('-G', 'Visual Studio 17 2022', '-A', 'x64') }

$configure = @('-S', $repo, '-B', $BuildDir) + $generator
if ($NoMujoco) { $configure += '-DOPENBEAM_BUILD_MUJOCO_PLUGIN=OFF' }
else {
    if (-not (Test-Path (Join-Path $MujocoRoot 'include\mujoco\mujoco.h'))) {
        throw "No MuJoCo release at ${MujocoRoot}: run python tools\fetch_mujoco.py, pass -MujocoRoot, or -NoMujoco"
    }
    $configure += @('-DOPENBEAM_BUILD_MUJOCO_PLUGIN=ON', "-DMUJOCO_ROOT=$MujocoRoot")
}

& $cmake @configure
if ($LASTEXITCODE -ne 0) { throw "configure failed ($LASTEXITCODE)" }
& $cmake --build $BuildDir --config $Config
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }
& $cmake --install $BuildDir --config $Config --prefix (Join-Path $BuildDir 'install')
if ($LASTEXITCODE -ne 0) { throw "install failed ($LASTEXITCODE)" }
if (-not $NoTests) {
    & $ctest --test-dir $BuildDir --output-on-failure -C $Config
    if ($LASTEXITCODE -ne 0) { throw "tests failed ($LASTEXITCODE)" }
}
