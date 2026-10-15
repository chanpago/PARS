param(
    [ValidateSet('Debug', 'Profile', 'Release')]
    [string]$Configuration = 'Debug',
    [string]$VisualStudioPath,
    [string]$ToolsetVersion,
    [string]$WindowsSDKVersion,
    [switch]$RunSmokeTest
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$buildDir = Join-Path $repoRoot '.tools\physx-build\vs2022-x64'

if (-not $VisualStudioPath) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) {
        throw 'Install Visual Studio 2022 with Desktop development with C++.'
    }
    $VisualStudioPath = & $vswhere -latest -products '*' -version '[17.0,18.0)' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $VisualStudioPath) { throw 'Visual Studio 2022 C++ tools were not found.' }
}
$VisualStudioPath = [IO.Path]::GetFullPath($VisualStudioPath).TrimEnd('\', '/')
if (-not $ToolsetVersion) {
    $ToolsetVersion = (Get-Content -LiteralPath (Join-Path $VisualStudioPath 'VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt') -Raw).Trim()
}
$cmake = Join-Path $VisualStudioPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
if (-not (Test-Path -LiteralPath $cmake)) {
    $command = Get-Command cmake.exe -ErrorAction SilentlyContinue
    if (-not $command) {
        throw 'Install C++ CMake tools for Windows in Visual Studio Installer, or CMake 3.25+ on PATH.'
    }
    $cmake = $command.Source
}

$toolset = 'v143,host=x64'
if ($ToolsetVersion) { $toolset += ",version=$ToolsetVersion" }
$configureArgs = @('-S', $PSScriptRoot, '-B', $buildDir,
    '-G', 'Visual Studio 17 2022', '-A', 'x64', '-T', $toolset,
    "-DCMAKE_GENERATOR_INSTANCE=$VisualStudioPath")
if ($WindowsSDKVersion) { $configureArgs += "-DCMAKE_SYSTEM_VERSION=$WindowsSDKVersion" }

& $cmake @configureArgs
if ($LASTEXITCODE -ne 0) { throw "PhysX CMake configuration failed ($LASTEXITCODE)." }

$sdkConfig = $Configuration.ToLowerInvariant()
$target = if ($RunSmokeTest) { 'PARSPhysXSmoke' } else { 'PARSPhysXSDK' }
# Bound both project and per-project compilation parallelism.
& $cmake --build $buildDir --config $sdkConfig --target $target -- /m:2 /p:CL_MPCount=4 /nr:false /verbosity:minimal
if ($LASTEXITCODE -ne 0) { throw "PhysX build failed ($LASTEXITCODE)." }

if ($RunSmokeTest) {
    & (Join-Path $buildDir "tests\$sdkConfig\PARSPhysXSmoke.exe")
    if ($LASTEXITCODE -ne 0) { throw "PhysX smoke test failed ($LASTEXITCODE)." }
}
