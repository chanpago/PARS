# Bootstrap a checkout-local vcpkg. The normal MSBuild integration restores the
# libraries afterwards. No administrator access or persistent environment changes.
[CmdletBinding()]
param([string] $VisualStudioRoot)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$toolsDirectory = Join-Path $repositoryRoot '.tools'
$vcpkgDirectory = Join-Path $toolsDirectory 'vcpkg'
$stagingDirectory = Join-Path $toolsDirectory 'vcpkg-bootstrap'
$revision = (Get-Content -LiteralPath (Join-Path $PSScriptRoot 'vcpkg-version.txt') -Raw).Trim()
$revisionFile = Join-Path $vcpkgDirectory '.directx-vcpkg-revision'

if ($revision -notmatch '^[0-9a-f]{40}$') {
    throw 'vcpkg-version.txt must contain a full Git commit ID.'
}

function Test-VcpkgReady {
    return ((Test-Path -LiteralPath $revisionFile) -and
        ((Get-Content -LiteralPath $revisionFile -Raw).Trim() -eq $revision) -and
        (Test-Path -LiteralPath (Join-Path $vcpkgDirectory 'vcpkg.exe')) -and
        (Test-Path -LiteralPath (Join-Path $vcpkgDirectory 'scripts\buildsystems\msbuild\vcpkg.targets')))
}

function Invoke-CheckedCommand([string] $Executable, [string[]] $Arguments) {
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Executable failed with exit code $LASTEXITCODE. Check the output above and retry the build."
    }
}

if (Test-VcpkgReady) { exit 0 }

$gitCommand = Get-Command git.exe -ErrorAction SilentlyContinue
$gitExecutable = if ($gitCommand) { $gitCommand.Source } else { $null }
if (!$gitExecutable -and $VisualStudioRoot) {
    $bundledGit = Join-Path $VisualStudioRoot 'Common7\IDE\CommonExtensions\Microsoft\TeamFoundation\Team Explorer\Git\cmd\git.exe'
    if (Test-Path -LiteralPath $bundledGit) { $gitExecutable = $bundledGit }
}
if (!$gitExecutable) {
    throw 'Git was not found. Install Git for Windows (or the Visual Studio Git component), then build again.'
}

New-Item -ItemType Directory -Path $toolsDirectory -Force | Out-Null
# Multiple projects may start their first build together. Only one may create or
# update this checkout; vcpkg itself serializes the subsequent package installs.
$lock = $null
$timer = [Diagnostics.Stopwatch]::StartNew()
try {
    while ($null -eq $lock) {
        try {
            $lock = [IO.File]::Open((Join-Path $toolsDirectory 'vcpkg-bootstrap.lock'),
                [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
        }
        catch [IO.IOException] {
            if ($timer.Elapsed.TotalMinutes -ge 15) {
                throw 'Timed out waiting for another vcpkg setup. Let that build finish, then retry.'
            }
            Start-Sleep -Milliseconds 250
        }
    }

    if (Test-VcpkgReady) { exit 0 }

    if (!(Test-Path -LiteralPath $vcpkgDirectory)) {
        # An interrupted download may leave this staging directory. Never remove
        # the installed checkout or anything outside this repository's .tools.
        if (Test-Path -LiteralPath $stagingDirectory) {
            $stagingItem = Get-Item -LiteralPath $stagingDirectory -Force
            $expectedStagingPath = [IO.Path]::GetFullPath((Join-Path $toolsDirectory 'vcpkg-bootstrap'))
            if ($stagingItem.FullName -ne $expectedStagingPath -or
                ($stagingItem.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
                throw "Refusing to remove an unexpected staging directory: $stagingDirectory"
            }
            Remove-Item -LiteralPath $stagingDirectory -Recurse -Force
        }
        Write-Host 'Preparing vcpkg for this checkout (first build only)...'
        # Keep the Git history: the manifest pins older port versions that a
        # shallow or partial clone might not have when vcpkg checks them out.
        Invoke-CheckedCommand $gitExecutable @('clone', '--no-checkout', 'https://github.com/microsoft/vcpkg.git', $stagingDirectory)
        $checkoutDirectory = $stagingDirectory
    }
    else {
        $checkoutDirectory = $vcpkgDirectory
        if (!(Test-Path -LiteralPath (Join-Path $checkoutDirectory '.git'))) {
            throw "The local vcpkg directory is not a Git checkout: $checkoutDirectory"
        }
        Invoke-CheckedCommand $gitExecutable @('-C', $checkoutDirectory, 'fetch', '--no-tags', 'origin', $revision)
    }

    Invoke-CheckedCommand $gitExecutable @('-C', $checkoutDirectory, 'config', 'core.longpaths', 'true')
    Invoke-CheckedCommand $gitExecutable @('-C', $checkoutDirectory, 'checkout', '--detach', $revision)
    $env:VCPKG_ROOT = $checkoutDirectory
    Invoke-CheckedCommand (Join-Path $checkoutDirectory 'bootstrap-vcpkg.bat') @('-disableMetrics')

    if ($checkoutDirectory -eq $stagingDirectory) {
        # Both absolute paths are fixed children of this repository's .tools.
        if ([IO.Path]::GetFullPath($stagingDirectory) -ne (Join-Path $toolsDirectory 'vcpkg-bootstrap') -or
            [IO.Path]::GetFullPath($vcpkgDirectory) -ne (Join-Path $toolsDirectory 'vcpkg')) {
            throw 'Unexpected vcpkg staging or destination path.'
        }
        Move-Item -LiteralPath $stagingDirectory -Destination $vcpkgDirectory
    }
    [IO.File]::WriteAllText($revisionFile, $revision + [Environment]::NewLine)
    Write-Host 'vcpkg is ready. Restoring the DirectX libraries...'
}
finally {
    if ($null -ne $lock) { $lock.Dispose() }
}
