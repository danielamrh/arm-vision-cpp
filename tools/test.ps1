# Builds the project on Windows and runs the unit tests, optionally filtered.
#
#   powershell -ExecutionPolicy Bypass -File tools\test.ps1                 # all tests
#   powershell -ExecutionPolicy Bypass -File tools\test.ps1 "Calibration.*" # only some
#
# Finds the CMake that ships with Visual Studio, so no Developer PowerShell is needed.
# Uses the "local" preset from CMakeUserPresets.json if there is one, else "msvc".
param([string]$Filter = "*")

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if ($cmake) {
    $cmake = $cmake.Source
} else {
    $cmake = Get-ChildItem "C:\Program Files*\Microsoft Visual Studio\2022\*\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $cmake) { throw "cmake not found (install Visual Studio 2022 or the Build Tools)" }

$userPresets = Join-Path $repo "CMakeUserPresets.json"
$hasLocal = (Test-Path $userPresets) -and ((Get-Content $userPresets -Raw) -match '"name":\s*"local"')
$configure = if ($hasLocal) { "local" } else { "msvc" }
$build = if ($hasLocal) { "local-debug" } else { "debug" }

Push-Location $repo
try {
    if (-not (Test-Path (Join-Path $repo "build\$configure\CMakeCache.txt"))) {
        & $cmake --preset $configure
        if ($LASTEXITCODE -ne 0) { throw "configure failed" }
    }
    & $cmake --build --preset $build --target armvision_tests
    if ($LASTEXITCODE -ne 0) { throw "build failed" }

    $env:OPENCV_LOG_LEVEL = "ERROR"  # hide OpenCV's plugin-loading messages
    & (Join-Path $repo "build\$configure\tests\Debug\armvision_tests.exe") --gtest_filter=$Filter --gtest_brief=1
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
