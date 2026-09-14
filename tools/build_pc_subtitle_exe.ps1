<#
.SYNOPSIS
    Build tools/pc_subtitle.py into a standalone Windows .exe (no Python needed).

.DESCRIPTION
    Wraps PyInstaller to produce a single double-clickable GUI executable for the
    PC-side subtitle sender (Type-C / F12, 9600 8N1). The .py source and this
    script are tracked; the produced .exe and all intermediates are build
    artifacts and are gitignored (regenerate with this script).

    Flags chosen deliberately:
      --onefile    one self-contained .exe (extracts to temp on launch); matches
                   "one exe" -- a single file to copy to the demo laptop.
      --windowed   no console window (this is a tkinter GUI). Consequence: the
                   exe is GUI-only. The --selftest protocol unit test prints to
                   stdout and is meant to run from SOURCE
                   (python tools/pc_subtitle.py --selftest), not from the exe.
      --collect-all serial
                   pyserial ships a PyInstaller hook, but list_ports pulls in the
                   Windows backend (serial.win32 / serial.serialwin32) lazily;
                   collecting the whole package guarantees the port dropdown is
                   populated on a machine that has only the CH340 driver.

    Everything is written under tools/ so a single .gitignore rule covers it:
      dist  -> tools/dist/pc_subtitle.exe   (the deliverable)
      work  -> tools/build/                 (PyInstaller intermediates + .spec)

.PARAMETER Run
    After a successful build, launch the produced .exe (smoke test the GUI opens).

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_pc_subtitle_exe.ps1
#>
[CmdletBinding()]
param(
    [switch]$Run
)

$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Script   = Join-Path $PSScriptRoot 'pc_subtitle.py'
$DistDir  = Join-Path $PSScriptRoot 'dist'
$WorkDir  = Join-Path $PSScriptRoot 'build'
$ExeName  = 'pc_subtitle'
$Exe      = Join-Path $DistDir "$ExeName.exe"

if (-not (Test-Path $Script)) {
    Write-Error "source not found: $Script"
    exit 1
}

# --- gate: protocol selftest must pass from source before we package ---------
Write-Host "---- selftest (source) ----"
& python $Script --selftest
if ($LASTEXITCODE -ne 0) {
    Write-Error "selftest failed (rc=$LASTEXITCODE); refusing to package a broken exe"
    exit 1
}

# --- ensure PyInstaller is present -------------------------------------------
& python -c "import PyInstaller" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "PyInstaller not found; installing..."
    & python -m pip install pyinstaller
    if ($LASTEXITCODE -ne 0) { Write-Error "pip install pyinstaller failed"; exit 1 }
}

# --- package -----------------------------------------------------------------
Write-Host "`n---- pyinstaller ----"
if (Test-Path $Exe) { Remove-Item $Exe -Force }

& python -m PyInstaller `
    --noconfirm --clean `
    --onefile --windowed `
    --name $ExeName `
    --collect-all serial `
    --distpath $DistDir `
    --workpath $WorkDir `
    --specpath $WorkDir `
    $Script

if ($LASTEXITCODE -ne 0) {
    Write-Error "PyInstaller failed (rc=$LASTEXITCODE)"
    exit 1
}

if (-not (Test-Path $Exe)) {
    Write-Error "expected exe not produced: $Exe"
    exit 1
}

$size = (Get-Item $Exe).Length
Write-Host ("`n==== BUILD OK ====")
Write-Host ("exe   : {0}" -f $Exe)
Write-Host ("size  : {0:N1} MB" -f ($size / 1MB))

if ($Run) {
    Write-Host "`nlaunching $Exe ..."
    Start-Process $Exe
}
