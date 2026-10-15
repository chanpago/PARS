<#
.SYNOPSIS
Installs the NVIDIA PhysX Visual Debugger used by PARS.
.DESCRIPTION
Run with Windows PowerShell 5.1 or newer. The authenticated NVIDIA download is
completed in the user's browser; credentials are never handled by this script.
Exit codes: 0 = ready, 2 = download needed, 1 = validation/installation failure.
See docs/PVD_SETUP.md for the human and AI setup workflow.
#>
[CmdletBinding()]
param(
    [string]$InstallerPath,
    [switch]$OpenDownloadPage
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$pvdDirectory = Join-Path $repositoryRoot '.tools\pvd'
$appDirectory = Join-Path $pvdDirectory 'app'
$pvdExecutable = Join-Path $appDirectory 'PVD3.exe'
$downloadPage = 'https://developer.nvidia.com/downloads/physx-visual-debugger-3.2021.01.29532135'
$installerSha256 = 'FC6B9EC5B3196864A2E03818648C2C1AD1950F943D69958037BEE2299064DEB5'
$executableSha256 = '2371E7123C9159677C13CC8B070047A763BDB01F96A0F09E313CAA5177A679BB'
$requiredFiles = @(
    'PVD3.exe', 'Qt5Core.dll', 'Qt5Gui.dll', 'Qt5Widgets.dll', 'Qt5OpenGL.dll',
    'Qt5PrintSupport.dll', 'Qt5Svg.dll', 'qwt.dll', 'd3dcompiler_47.dll',
    'libEGL.dll', 'libGLESV2.dll', 'opengl32sw.dll', 'platforms\qwindows.dll'
)

function Assert-NvidiaSignature([string]$Path) {
    $signature = Get-AuthenticodeSignature -LiteralPath $Path
    if ($signature.Status -ne 'Valid' -or $null -eq $signature.SignerCertificate -or
        $signature.SignerCertificate.Subject -notmatch '(^|,\s*)O=NVIDIA Corporation(,|$)') {
        throw "A valid NVIDIA Corporation signature is required: $Path (status: $($signature.Status))."
    }
}

function Test-PvdReady {
    foreach ($relativePath in $requiredFiles) {
        if (-not (Test-Path -LiteralPath (Join-Path $appDirectory $relativePath) -PathType Leaf)) {
            return $false
        }
    }
    return ((Get-FileHash -LiteralPath $pvdExecutable -Algorithm SHA256).Hash -eq $executableSha256)
}

try {
    if ($env:OS -ne 'Windows_NT' -or -not [Environment]::Is64BitOperatingSystem) {
        throw 'This setup supports 64-bit Windows only.'
    }

    if (Test-PvdReady) {
        Assert-NvidiaSignature $pvdExecutable
        Write-Host "PVD_READY: $pvdExecutable (already installed; no changes)."
        exit 0
    }

    if ($InstallerPath) {
        $installer = Get-Item -LiteralPath $InstallerPath
        if ($installer.PSIsContainer -or $installer.Extension -ne '.exe') {
            throw 'InstallerPath must point to the downloaded pvd_setup.exe file.'
        }
    }
    else {
        # Respect a relocated Windows Downloads known folder (including OneDrive).
        $downloadsDirectory = Get-ItemPropertyValue -LiteralPath 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders' `
            -Name '{374DE290-123F-4565-9164-39C4925E467B}' -ErrorAction SilentlyContinue
        if ($downloadsDirectory) {
            $downloadsDirectory = [Environment]::ExpandEnvironmentVariables($downloadsDirectory)
        }
        else {
            $downloadsDirectory = Join-Path $env:USERPROFILE 'Downloads'
        }

        $searchDirectories = @((Join-Path $pvdDirectory 'downloads'), $downloadsDirectory)
        $candidates = @(
            foreach ($directory in $searchDirectories) {
                if (Test-Path -LiteralPath $directory -PathType Container) {
                    Get-ChildItem -LiteralPath $directory -Filter 'pvd_setup*.exe' -File
                }
            }
        )

        $installer = $null
        foreach ($candidate in ($candidates | Sort-Object -Property LastWriteTime -Descending)) {
            if ((Get-FileHash -LiteralPath $candidate.FullName -Algorithm SHA256).Hash -eq $installerSha256) {
                $installer = $candidate
                break
            }
        }

        if ($null -eq $installer) {
            Write-Host 'PVD_DOWNLOAD_REQUIRED: Download the supported NVIDIA installer, then rerun this command.'
            Write-Host "Download: $downloadPage"
            Write-Host 'Sign in to NVIDIA in your browser and complete any download/license prompts yourself.'
            Write-Host "Save pvd_setup.exe in: $downloadsDirectory"
            Write-Host 'For another location, rerun with -InstallerPath "C:\path\to\pvd_setup.exe".'
            if (@($candidates).Count -gt 0) {
                Write-Warning 'Existing download candidates do not match the supported installer SHA256. None were executed.'
            }
            if ($OpenDownloadPage) {
                # A browser is interactive: show it so the user can sign in.
                Start-Process -FilePath $downloadPage | Out-Null
            }
            exit 2
        }
    }

    $actualHash = (Get-FileHash -LiteralPath $installer.FullName -Algorithm SHA256).Hash
    if ($actualHash -ne $installerSha256) {
        throw "Unsupported or incomplete installer: $($installer.FullName). Expected SHA256 $installerSha256; got $actualHash. See docs/PVD_SETUP.md."
    }
    Assert-NvidiaSignature $installer.FullName

    # Do not replace files under a running debugger or terminate a user's session.
    if (Get-Process -Name PVD3 -ErrorAction SilentlyContinue) {
        throw 'Close PVD before installing or repairing it, then rerun this command.'
    }

    New-Item -ItemType Directory -Path $pvdDirectory -Force | Out-Null
    $logPath = Join-Path $pvdDirectory 'install.log'
    # Inno Setup expects quoted /DIR and /LOG values, including paths with spaces.
    $installerArguments = @(
        '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/SP-', '/NOICONS',
        ('/DIR="{0}"' -f $appDirectory), ('/LOG="{0}"' -f $logPath)
    )
    Write-Host "Installing PVD to $appDirectory. Accept the Windows UAC prompt if one appears."
    $process = Start-Process -FilePath $installer.FullName -ArgumentList $installerArguments `
        -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -notin @(0, 3010)) {
        throw "PVD installer exited with code $($process.ExitCode). See $logPath."
    }
    if (-not (Test-PvdReady)) {
        throw "Installation did not produce the supported PVD executable and required files. See $logPath."
    }
    Assert-NvidiaSignature $pvdExecutable

    [ordered]@{
        catalogVersion = '3.2021.01.29532135'
        downloadPage = $downloadPage
        installerSha256 = $actualHash
        executableSha256 = $executableSha256
        installedAtUtc = [DateTime]::UtcNow.ToString('o')
        installerExitCode = $process.ExitCode
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $pvdDirectory 'setup-info.json') -Encoding UTF8

    if ($process.ExitCode -eq 3010) {
        Write-Warning 'The installer requested a Windows restart. This script will not restart the PC.'
    }
    Write-Host "PVD_READY: $pvdExecutable"
    Write-Host 'Run ModelViewer in Debug or Profile and click Launch / connect PVD, then Start drop.'
    exit 0
}
catch {
    Write-Error -Message $_.Exception.Message -ErrorAction Continue
    exit 1
}
